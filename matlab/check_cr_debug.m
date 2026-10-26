% whill_common_matlab の CR_debug（cr_debug ブランチ）の購読と、/tf から引く姿勢（後輪間中心は最新の map → base、
% 点群は stamp の時刻の map → velodyne）を、ROS_DOMAIN_ID=99 で再生している録画で確かめる（dev_log.md の 62・68）。
% ホストで MATLAB を -batch で動かす。コンテナで eval/replay_outputs.py を動かしている間に（dev_log.md の 62）：
%   REPO=<whill_common_matlab のフォルダ> TEST_DURATION=70 TEST_OUT=<結果の .mat> \
%     matlab -batch "run('<localization_tools>/matlab/check_cr_debug.m')"
% 結果は標準出力と TEST_OUT の rows（周期ごとに [経過時間, Plant.X, Plant.Y, Plant.Yaw, 点群の stamp, LIDAR_pose の stamp,
% LIDAR_new, LIDAR_pose の X, Y, Yaw, LIDAR_waiting]）。図は plot/plot_cr_debug_pairs.py。
repo = getenv("REPO");
addpath(repo)
setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
setenv("FASTRTPS_DEFAULT_PROFILES_FILE", fullfile(repo, "fastdds_localhost.xml"))
setenv("ROS_LOCALHOST_ONLY", "0")

vehicleInfo.type = "CR_debug";
vehicleInfo.color = 'red';
vehicleInfo.sensor = logical([1 0 0 0 1 0 1 1]);   % LiDAR, Matching (/tf), Localization Debug, LiDAR pose (/tf)
vehicleInfo.base_sensor = 1;
cfg = struct("modeNumber", 2, "RID", 99, "rosNamespace", "matlab_cr_debug_test", "vehicleInfo", vehicleInfo);

comm = bridge.CR_debug.ROS2CommManager(cfg, 2);
comm.generateSensorSubscribers();
comm.generateVehicleSubscribers();

duration = str2double(getenv("TEST_DURATION"));
if isnan(duration), duration = 100; end
rate = rateControl(20);
rows = [];   % [t, Plant.X, Plant.Y, Plant.Yaw, 点群の stamp, 姿勢の stamp, LIDAR_new, LiDAR の X, Y, Yaw, LIDAR_waiting]
tic
while toc < duration
    [data, Plant] = comm.getSensorData();
    if ~isempty(data.LIDAR)
        cs = double(data.LIDAR.header.stamp.sec) + double(data.LIDAR.header.stamp.nanosec) * 1e-9;
        lp = data.LIDAR_pose;
        rows(end+1, :) = [toc, Plant.X, Plant.Y, Plant.Yaw, cs, lp.Stamp, data.LIDAR_new, lp.X, lp.Y, lp.Yaw, ...
            data.LIDAR_waiting]; %#ok<SAGROW>
    else
        rows(end+1, :) = [toc, Plant.X, Plant.Y, Plant.Yaw, NaN, NaN, 0, NaN, NaN, NaN, data.LIDAR_waiting]; %#ok<SAGROW>
    end
    waitfor(rate);
end

paired = ~isnan(rows(:, 5));
fprintf("cycles: %d (%.1f Hz), with a pair: %d\n", size(rows, 1), size(rows, 1) / duration, nnz(paired));
fprintf("pairs with different stamps (cloud vs LiDAR pose): %d\n", nnz(abs(rows(paired, 5) - rows(paired, 6)) > 1e-9));
fprintf("new pairs (LIDAR_new): %d, distinct cloud stamps: %d\n", nnz(rows(:, 7)), numel(unique(rows(paired, 5))));
fprintf("cycles waiting for the TF at the newest cloud stamp (LIDAR_waiting): %d\n", nnz(rows(:, 11)));
moving = rows(:, 2) ~= 0 | rows(:, 3) ~= 0;
fprintf("cycles with Plant (/tf) set: %d, distinct Plant.X: %d\n", nnz(moving), numel(unique(rows(moving, 2))));
if any(paired)
    d = hypot(rows(paired, 2) - rows(paired, 8), rows(paired, 3) - rows(paired, 9));
    fprintf("Plant (base, now) - LIDAR_pose (LiDAR, cloud stamp) horizontal: median %.3f m, max %.3f m\n", median(d), max(d));
    fprintf("LIDAR_pose.Stamp - cloud stamp: max %.3g s\n", max(abs(rows(paired, 6) - rows(paired, 5))));
end
save(getenv("TEST_OUT"), "rows")
