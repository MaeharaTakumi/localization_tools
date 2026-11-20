% whill_common_matlab の CR_debug（cr_debug ブランチ）の購読と、/tf から引く姿勢（後輪間中心は最新の map → base、
% 点群は stamp の時刻の map → velodyne）を、ROS_DOMAIN_ID=99 で再生している録画で確かめる（dev_log.md の 62・68）。
% ホストで MATLAB を -batch で動かす。コンテナで eval/replay_outputs.py を動かしている間に（dev_log.md の 62）：
%   REPO=<whill_common_matlab のフォルダ> TEST_DURATION=70 TEST_OUT=<結果の .mat> \
%     matlab -batch "run('<localization_tools>/matlab/check_cr_debug.m')"
% 結果は標準出力と TEST_OUT の rows（周期ごとに [経過時間, Plant.X, Plant.Y, Plant.Yaw, 点群の stamp, LIDAR_pose の stamp,
% 新しい点群（姿勢付き）か, LIDAR_pose の X, Y, Yaw, 姿勢の無い点群か, 点群の stamp 以外の時刻の TF か]）。図は plot/plot_cr_debug_pairs.py。
% 後ろの 3 つは data から求める（2026-11-19 に SensorFetcher の LIDAR_new・LIDAR_waiting・LIDAR_latest_tf をやめた）。
% 環境変数（省略可）：
%   TEST_LIDAR_THRESHOLD  SensorFetcher の lidarStampThreshold（小さくすると全部の周期が今の時刻の TF になる）
%   MODE・COLOR・SIM_TIME  check_cr_debug_plot.m と同じ（MODE 3 と SIM_TIME 1 で実機の録画）
%   READY_FILE  購読を作ったらこのファイルを作る（再生を始める合図）
%   STOP_IDLE   姿勢の付いた新しい点群が来なくなってからこの秒数で止める（0 なら TEST_DURATION まで）
%   TEST_POSES  点群のある周期ごとに 1 行書くテキスト（rviz2 と比べる。rviz/compare_rviz_cr_debug.py。dev_log.md の 107）：
%     <getSensorData の前・後の CLOCK_MONOTONIC [ns]> <点群の stamp の sec・nanosec> <LIDAR_pose の X Y Z Roll Pitch Yaw Stamp>
%     （姿勢が無ければ X〜Stamp は nan）。CLOCK_MONOTONIC は Java の System.nanoTime（コンテナの rviz2 のフックの steady_clock と同じ時計）
repo = getenv("REPO");
addpath(repo)
setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
setenv("FASTRTPS_DEFAULT_PROFILES_FILE", fullfile(repo, "fastdds_localhost.xml"))
setenv("ROS_LOCALHOST_ONLY", "0")

mode = envNumber("MODE", 2);
color = getenv("COLOR");
if isempty(color), color = 'red'; end
vehicleInfo.type = "CR_debug";
vehicleInfo.color = color;
vehicleInfo.sensor = logical([1 0 0 0 1 0]);   % LiDAR, Matching（/tf の Plant と LIDAR_pose，/localization_debug）
vehicleInfo.base_sensor = 1;
cfg = struct("modeNumber", mode, "RID", 99, "rosNamespace", "matlab_cr_debug_test", "vehicleInfo", vehicleInfo, ...
    "useSimTime", envNumber("SIM_TIME", 0) == 1);

threshold = str2double(getenv("TEST_LIDAR_THRESHOLD"));
if ~isnan(threshold), cfg.lidarStampThreshold = threshold; end
comm = bridge.CR_debug.ROS2CommManager(cfg, mode);
comm.generateSensorSubscribers();
comm.generateVehicleSubscribers();
poses = -1;
if ~isempty(getenv("TEST_POSES"))
    poses = fopen(getenv("TEST_POSES"), "w");
end
if ~isempty(getenv("READY_FILE"))
    fclose(fopen(getenv("READY_FILE"), "w"));
end

duration = envNumber("TEST_DURATION", 100);
stopIdle = envNumber("STOP_IDLE", 0);
rate = rateControl(20);
rows = [];   % [t, Plant.X, Plant.Y, Plant.Yaw, 点群の stamp, 姿勢の stamp, 新しい点群（姿勢付き）, LiDAR の X, Y, Yaw, 姿勢なし, 点群の stamp 以外の TF]
lastCloud = NaN;   % 前の周期の点群の stamp
lastNew = NaN;     % 最後に姿勢の付いた新しい点群を受け取った時刻
tic
while toc < duration
    m0 = java.lang.System.nanoTime();
    [data, Plant] = comm.getSensorData();
    m1 = java.lang.System.nanoTime();
    if isempty(data.LIDAR)
        rows(end+1, :) = [toc, Plant.X, Plant.Y, Plant.Yaw, NaN, NaN, 0, NaN, NaN, NaN, 0, 0]; %#ok<SAGROW>
    else
        st = data.LIDAR.header.stamp;
        cs = double(st.sec) + double(st.nanosec) * 1e-9;
        isNew = cs ~= lastCloud;
        lastCloud = cs;
        lp = data.LIDAR_pose;
        if isempty(lp)
            rows(end+1, :) = [toc, Plant.X, Plant.Y, Plant.Yaw, cs, NaN, 0, NaN, NaN, NaN, 1, 0]; %#ok<SAGROW>
            v = nan(1, 7);
        else
            rows(end+1, :) = [toc, Plant.X, Plant.Y, Plant.Yaw, cs, lp.Stamp, isNew, lp.X, lp.Y, lp.Yaw, ...
                0, abs(lp.Stamp - cs) > 1e-9]; %#ok<SAGROW>
            v = [lp.X, lp.Y, lp.Z, lp.Roll, lp.Pitch, lp.Yaw, lp.Stamp];
            if isNew, lastNew = toc; end
        end
        if poses >= 0
            fprintf(poses, "%d %d %d %d", m0, m1, double(st.sec), double(st.nanosec));
            fprintf(poses, " %.17g", v);
            fprintf(poses, "\n");
        end
    end
    if stopIdle > 0 && ~isnan(lastNew) && toc - lastNew > stopIdle
        break
    end
    waitfor(rate);
end
if poses >= 0, fclose(poses); end
duration = min(duration, toc);

paired = ~isnan(rows(:, 5)) & ~isnan(rows(:, 6));
fprintf("cycles: %d (%.1f Hz), with a pair: %d\n", size(rows, 1), size(rows, 1) / duration, nnz(paired));
fprintf("pairs with different stamps (cloud vs LiDAR pose): %d\n", nnz(abs(rows(paired, 5) - rows(paired, 6)) > 1e-9));
fprintf("new point clouds with a pose: %d, distinct cloud stamps: %d\n", nnz(rows(:, 7)), numel(unique(rows(paired, 5))));
fprintf("cycles with a point cloud but no pose (no TF at all): %d\n", nnz(rows(:, 11)));
fprintf("cycles using the TF at now or the latest (not at the cloud stamp): %d\n", nnz(rows(:, 12)));
moving = rows(:, 2) ~= 0 | rows(:, 3) ~= 0;
fprintf("cycles with Plant (/tf) set: %d, distinct Plant.X: %d\n", nnz(moving), numel(unique(rows(moving, 2))));
if any(paired)
    d = hypot(rows(paired, 2) - rows(paired, 8), rows(paired, 3) - rows(paired, 9));
    fprintf("Plant (base, now) - LIDAR_pose (LiDAR, cloud stamp) horizontal: median %.3f m, max %.3f m\n", median(d), max(d));
    fprintf("LIDAR_pose.Stamp - cloud stamp: max %.3g s\n", max(abs(rows(paired, 6) - rows(paired, 5))));
end
if ~isempty(getenv("TEST_OUT"))
    save(getenv("TEST_OUT"), "rows")
end


function v = envNumber(name, default)
    % 環境変数の数値（無いか数値でなければ default）
    v = str2double(getenv(name));
    if isnan(v), v = default; end
end
