% whill_common_matlab の SIP 版（ブランチ cr_debug_sip．WheelchairSIP の main の推定器 Estimate2・制御器 Control3・図 myplotResult を
% CR_debug の /tf の姿勢で動かす版）を main.m と同じ流れで動かし，記録と図を作る．ホストで MATLAB を -batch で動かす
% （-batch では A* の後の Enter を待たない）．
% 録画の再生と一緒に動かす（指令は配信しない．ROS_DOMAIN_ID=99）のは matlab/replay_matlab.sh：
%   CHECK=check_sip.m REPO=<SIP 版のフォルダ> replay_matlab.sh <bag> <出力のフォルダ>
% 動いている Gazebo と自己位置推定で閉ループにする（指令を配信する）のは：
%   REPO=<SIP 版のフォルダ> RID=11 SEND_CMD=1 TEST_DURATION=60 TEST_OUT_DIR=<記録と図のフォルダ> \
%     matlab -batch "run('<localization_tools>/matlab/check_sip.m')"
% 環境変数（省略可）：
%   MODE       2（Gazebo、既定）か 3（実機の録画．購読するトピックは実機と同じ）
%   COLOR      実機の車椅子の色（red・blue・white．既定 white）
%   SIM_TIME   1 なら ROS の時刻を /clock にする（録画の再生．MODE 2 は常に /clock）
%   RID        ROS_DOMAIN_ID（既定 99．replay_outputs.py の再生と同じ）
%   SEND_CMD   1 なら指令を配信する（終わりに 0 を送る）
%   STOP_IDLE  姿勢の付いた新しい点群が来なくなってからこの秒数で止める（0 なら TEST_DURATION まで）
%   READY_FILE 購読を作ったらこのファイルを作る（replay_matlab.sh が再生を始める合図）
%   PLOT       0 なら図（DataPlotter・myplotResult）を描かない
%   GET_NEW_TF 1 なら main.m の getNewTf を true にする（LIDAR_pose も最新の TF。既定 0）
% 記録は TEST_OUT_DIR の userLocal_tmp.mat・userLocal.mat・setting_*.mat・pms.mat・GridMap.mat，図と動画は TEST_OUT_DIR/result．
repo = getenv("REPO");
addpath(repo)
cd(repo)   % config.m の格子地図（GridMapSC10_4_BIM.mat）をこのフォルダから読む
addpath(fullfile(repo, "RealTimePlotter"))
setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
setenv("FASTRTPS_DEFAULT_PROFILES_FILE", fullfile(repo, "fastdds_localhost.xml"))
setenv("ROS_LOCALHOST_ONLY", "0")
parallel.gpu.enableCUDAForwardCompatibility(true)

mode = envNumber("MODE", 2);
color = getenv("COLOR");
if isempty(color), color = 'white'; end
sendCmd = envNumber("SEND_CMD", 0) == 1;

vehicleInfo.type = "CR_debug";
vehicleInfo.color = color;
vehicleInfo.sensor = logical([1 0 0 0 1 0]);   % LiDAR, Matching（/tf の Plant と LIDAR_pose，/localization_debug）
vehicleInfo.base_sensor = 1;
cfg = struct("modeNumber", mode, "isParallel", false, "RID", envNumber("RID", 99), "rosNamespace", "matlab_sip", ...
    "vehicleInfo", vehicleInfo, "useSimTime", envNumber("SIM_TIME", 0) == 1, "getNewTf", envNumber("GET_NEW_TF", 0) == 1);

outDir = getenv("TEST_OUT_DIR");
if ~exist(outDir, "dir"), mkdir(outDir); end
comm = bridge.CR_debug.ROS2CommManager(cfg, mode);
comm.generateSensorSubscribers();
comm.generateVehicleSubscribers();
if sendCmd
    comm.generateVehiclePublishers();
end
gmm = GridMap.GridMapManager;
est = estimator.Estimate2(mode, outDir, gmm);
ctrl = controller.Control3(outDir, gmm);
dlog = logger.DataLogger(outDir, 'tmp');
if ~isempty(getenv("READY_FILE"))
    fclose(fopen(getenv("READY_FILE"), "w"));
end

duration = envNumber("TEST_DURATION", 40);
stopIdle = envNumber("STOP_IDLE", 0);
timer = rateControl(20);
reset(timer);
cnt = 0;
lastNew = NaN;     % 最後に姿勢の付いた新しい点群を受け取った時刻
lastCloud = NaN;   % その点群の stamp
n = struct("pose", 0, "astar", NaN, "track", 0, "obstacles", 0, "mcmpc", 0);
V = zeros(2, 0);
stepTime = [];
c = onCleanup(@() stopVehicle(comm, sendCmd));
while timer.TotalElapsedTime < duration
    cnt = cnt + 1;
    [sens, Plant] = comm.getSensorData();
    T = timer.TotalElapsedTime;
    t0 = tic;
    [resEst, sendData] = est.main(sens, Plant, T);
    resEst.T = posixtime(datetime('now','TimeZone','local'));
    resCtrl = ctrl.main(sendData, T);
    resCtrl.T = posixtime(datetime('now','TimeZone','local'));
    stepTime(end+1) = toc(t0); %#ok<SAGROW>
    if sendCmd
        comm.sendVehicleCommand(struct('V', resCtrl.V, 'sequence', cnt));
    end
    resEst.send = sendData;
    resEst.sequence = cnt;
    resCtrl.sequence = cnt;
    dlog.addData(struct('Estimator', resEst, 'Controller', resCtrl));
    % 数える：姿勢が来た周期，A* の周期，障害物を追跡した周期と障害物の数，MCMPC の周期と指令
    n.pose = n.pose + ~isempty(Plant.Stamp);
    if resEst.do_AStar, n.astar = cnt; end
    n.track = n.track + (est.T_track == T);
    n.obstacles = n.obstacles + size(sendData.BBoxes, 2);
    if ~isempty(resCtrl.mcmpc)
        n.mcmpc = n.mcmpc + 1;
        V(:, end+1) = resCtrl.V; %#ok<SAGROW>
    end
    if ~isempty(sens.LIDAR) && ~isempty(sens.LIDAR_pose)
        cs = double(sens.LIDAR.header.stamp.sec) + double(sens.LIDAR.header.stamp.nanosec) * 1e-9;
        if cs ~= lastCloud
            lastNew = timer.TotalElapsedTime;
            lastCloud = cs;
        end
    end
    if stopIdle > 0 && ~isnan(lastNew) && timer.TotalElapsedTime - lastNew > stopIdle
        break
    end
    waitfor(timer);
end
stopVehicle(comm, sendCmd);
dlog.stop();
fprintf("cycles: %d, with pose: %d, A* at cycle %d, tracked: %d, obstacles per cycle: %.2f, MCMPC: %d\n", ...
    cnt, n.pose, n.astar, n.track, n.obstacles / cnt, n.mcmpc);
fprintf("estimator + controller time [ms]: median %.1f, 95%% %.1f, max %.1f\n", ...
    1e3 * median(stepTime), 1e3 * prctile(stepTime, 95), 1e3 * max(stepTime));
if ~isempty(V)
    fprintf("command v [m/s]: %.3f .. %.3f, omega [rad/s]: %.3f .. %.3f\n", min(V(1,:)), max(V(1,:)), min(V(2,:)), max(V(2,:)));
end
if envNumber("PLOT", 1) == 1
    plotter.DataPlotter(outDir, cfg)
    addpath(fullfile(repo, "myPlotter"))
    myplotResult(outDir, cfg)
    disp(dir(fullfile(outDir, "result", "video")))
end


function stopVehicle(comm, sendCmd)
    % 指令を配信したときは，止まる指令を何回か送る
    if sendCmd
        for k = 1:5
            comm.sendVehicleCommand(struct('V', [0; 0], 'sequence', 0));
            pause(0.05);
        end
    end
end

function v = envNumber(name, default)
    % 環境変数の数値（無いか数値でなければ default）
    v = str2double(getenv(name));
    if isnan(v), v = default; end
end
