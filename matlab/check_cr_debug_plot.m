% whill_common_matlab の CR_debug を main.m と同じ流れ（Estimate2・Control2・DataLogger）で記録し、
% plotter.DataPlotter・plotter.myplotResult で図を、plotter.myplotPointCloudVideo で動画を作る。指令は配信しない。
% ホストで MATLAB を -batch で動かす。録画の再生（コンテナの eval/replay_outputs.py、ROS_DOMAIN_ID=99）と一緒に動かすのは
% matlab/replay_matlab.sh（dev_log.md の 63〜65）。単独では：
%   REPO=<whill_common_matlab のフォルダ> TEST_DURATION=40 TEST_OUT_DIR=<記録と図のフォルダ> MAP=<点群地図（.pcd）> \
%     matlab -batch "run('<localization_tools>/matlab/check_cr_debug_plot.m')"
% 環境変数（省略可）：
%   MODE       2（Gazebo、既定）か 3（実機の録画。購読するトピックは実機と同じ）
%   COLOR      実機の車椅子の色（red・blue・white。既定 red）
%   SIM_TIME   1 なら ROS の時刻を /clock にする（録画の再生。MODE 2 は常に /clock）
%   STOP_IDLE  新しい組（点群と LiDAR の姿勢）が来なくなってからこの秒数で止める（0 なら TEST_DURATION まで）
%   READY_FILE 購読を作ったらこのファイルを作る（replay_matlab.sh が再生を始める合図）
% 記録は TEST_OUT_DIR の userLocal_tmp.mat・userLocal.mat、図と動画は TEST_OUT_DIR/result（MAP が空なら地図なしの動画）。
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
vehicleInfo.sensor = logical([1 0 0 0 1 0 1 1]);   % LiDAR, Matching (base_pose), Localization Debug, LiDAR pose
vehicleInfo.base_sensor = 1;
cfg = struct("modeNumber", mode, "isParallel", false, "RID", 99, "rosNamespace", "matlab_cr_debug_plot", ...
    "vehicleInfo", vehicleInfo, "useSimTime", envNumber("SIM_TIME", 0) == 1);

comm = bridge.CR_debug.ROS2CommManager(cfg, mode);
comm.generateSensorSubscribers();
comm.generateVehicleSubscribers();
est = estimator.Estimate2(mode, "");
ctrl = controller.Control2();
outDir = getenv("TEST_OUT_DIR");
if ~exist(outDir, "dir"), mkdir(outDir); end
dlog = logger.DataLogger(outDir, 'tmp');
if ~isempty(getenv("READY_FILE"))
    fclose(fopen(getenv("READY_FILE"), "w"));
end

duration = envNumber("TEST_DURATION", 40);
stopIdle = envNumber("STOP_IDLE", 0);
timer = rateControl(20);
reset(timer);
cnt = 0;
lastNew = NaN;
while timer.TotalElapsedTime < duration
    cnt = cnt + 1;
    [sens, Plant] = comm.getSensorData();
    [resEst, sendData] = est.main(sens, Plant, timer.TotalElapsedTime);
    resEst.T = posixtime(datetime('now','TimeZone','local'));
    resCtrl = ctrl.main(sendData, timer.TotalElapsedTime);
    resCtrl.T = posixtime(datetime('now','TimeZone','local'));
    resEst.send = sendData;
    resEst.sequence = cnt;
    resCtrl.sequence = cnt;
    dlog.addData(struct('Estimator', resEst, 'Controller', resCtrl));
    if sens.LIDAR_new
        lastNew = timer.TotalElapsedTime;
    end
    if stopIdle > 0 && ~isnan(lastNew) && timer.TotalElapsedTime - lastNew > stopIdle
        break
    end
    waitfor(timer);
end
dlog.stop();
fprintf("cycles: %d\n", cnt);
plotter.DataPlotter(outDir, cfg)
plotter.myplotResult(outDir, cfg)
plotter.myplotPointCloudVideo(outDir, getenv("MAP"))
disp(dir(fullfile(outDir, "result", "jpeg")))


function v = envNumber(name, default)
    % 環境変数の数値（無いか数値でなければ default）
    v = str2double(getenv(name));
    if isnan(v), v = default; end
end
