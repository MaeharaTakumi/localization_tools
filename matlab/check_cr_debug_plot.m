% whill_common_matlab の CR_debug を main.m と同じ流れ（Estimate2・Control2・DataLogger）で記録し、
% plotter.DataPlotter・plotter.myplotResult で図を、plotter.myplotPointCloudVideo で動画を作る。指令は配信しない。
% ホストで MATLAB を -batch で動かす。コンテナで eval/replay_outputs.py を動かしている間に（dev_log.md の 63）：
%   REPO=<whill_common_matlab のフォルダ> TEST_DURATION=40 TEST_OUT_DIR=<記録と図のフォルダ> MAP=<点群地図（.pcd）> \
%     matlab -batch "run('<localization_tools>/matlab/check_cr_debug_plot.m')"
% 記録は TEST_OUT_DIR の userLocal_tmp.mat・userLocal.mat、図と動画は TEST_OUT_DIR/result（MAP が空なら地図なしの動画）。
repo = getenv("REPO");
addpath(repo)
setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
setenv("FASTRTPS_DEFAULT_PROFILES_FILE", fullfile(repo, "fastdds_localhost.xml"))
setenv("ROS_LOCALHOST_ONLY", "0")

vehicleInfo.type = "CR_debug";
vehicleInfo.color = 'red';
vehicleInfo.sensor = logical([1 0 0 0 1 0 1 1]);   % LiDAR, Matching (base_pose), Localization Debug, LiDAR pose
vehicleInfo.base_sensor = 1;
cfg = struct("modeNumber", 2, "isParallel", false, "RID", 99, "rosNamespace", "matlab_cr_debug_plot", ...
    "vehicleInfo", vehicleInfo);

comm = bridge.CR_debug.ROS2CommManager(cfg, 2);
comm.generateSensorSubscribers();
comm.generateVehicleSubscribers();
est = estimator.Estimate2(2, "");
ctrl = controller.Control2();
outDir = getenv("TEST_OUT_DIR");
if ~exist(outDir, "dir"), mkdir(outDir); end
dlog = logger.DataLogger(outDir, 'tmp');

duration = str2double(getenv("TEST_DURATION"));
if isnan(duration), duration = 40; end
timer = rateControl(20);
reset(timer);
cnt = 0;
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
    waitfor(timer);
end
dlog.stop();
fprintf("cycles: %d\n", cnt);
plotter.DataPlotter(outDir, cfg)
plotter.myplotResult(outDir, cfg)
plotter.myplotPointCloudVideo(outDir, getenv("MAP"))
disp(dir(fullfile(outDir, "result", "jpeg")))
