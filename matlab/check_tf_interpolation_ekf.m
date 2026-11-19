% 実際の ekf_localizer の /tf で、ros2tf の getTransform(tftree, 親, 子, stamp)（CR_debug が点群の stamp で引くのと同じ）が
% 前後の TF の線形補間（位置は線形、回転は最短経路の slerp）になっているかを確かめる（dev_log.md の 105）。
% ekf_localizer は 20 ms 刻みの TF を 1 メッセージに 3 つ入れて毎周期送り、同じ刻みを新しい値で送り直す（dev_log.md の 72）。
% 録画を再生している間（コンテナ test の eval/replay_outputs.py、ROS_DOMAIN_ID=99）に、ros2tf に溜めながら /tf をそのまま記録し、
% 再生が終わったら ros2tf から（1）点群の stamp（2）刻みの中点（3）刻みちょうど の時刻で map → base を引いて、記録した TF から
% 自分で計算した補間と比べる。同じ刻みが何度も届くので、比べる補間は 3 つ：最後に届いた値どうし、最初に届いた値どうし、
% 前の刻みは最後・後の刻みは最初に届いた値（tf2 は同じ時刻を置き換えずに重ねて持つので、こうなる。check_tf_interpolation.m の送り直しの確認）。
% 点群の stamp では map → LiDAR も引き、map → base の補間に取付（/tf_static）を掛けたものと比べる。
% ふつうは matlab/replay_tf_interpolation.sh から動かす（MATLAB を先に起動し、購読ができてから再生する）。
% 環境変数：REPO（fastdds_localhost.xml のある whill_common_matlab のフォルダ）、TEST_DOMAIN（既定 99）、TEST_DURATION（最長 [s]。既定 300）、
%   STOP_IDLE（/tf が来なくなってから止まるまで [s]。既定 5）、READY_FILE（購読ができたら作る）、LIDAR_FRAME（既定 velodyne）、
%   TEST_OUT（結果の .mat）、TEST_FIG（図の .png）

global TFLOG CLOUDLOG
here = fileparts(mfilename("fullpath"));
addpath(here)   % TfMath
repo = getenv("REPO");
if repo == ""
    repo = fullfile(here, "..", "..", "whill_common_matlab-main _ndtDebug");
end
setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
setenv("FASTRTPS_DEFAULT_PROFILES_FILE", fullfile(repo, "fastdds_localhost.xml"))
setenv("ROS_LOCALHOST_ONLY", "0")
domain = envNum("TEST_DOMAIN", 99);
duration = envNum("TEST_DURATION", 300);
stopIdle = envNum("STOP_IDLE", 5);
lidarFrame = string(getenv("LIDAR_FRAME"));
if lidarFrame == "", lidarFrame = "velodyne"; end
deg = pi/180;
fprintf("MATLAB %s, ROS_DOMAIN_ID %d\n", version, domain);

%% 購読（ros2tf に溜めながら、/tf の map → * と点群の stamp をそのまま記録する）
TFLOG = struct("n", 0, "data", zeros(50000, 9), "child", strings(50000, 1));   % data：[sec nanosec x y z qw qx qy qz]（届いた順）
CLOUDLOG = zeros(0, 2);                                                         % [sec nanosec]
node = ros2node("tf_interp_ekf_check", domain);
tftree = ros2tf(node);
tftree.BufferTime = duration + 60;   % 再生の全体を溜めておき、終わってから引く
tfSub = ros2subscriber(node, "/tf", "tf2_msgs/TFMessage", @onTf, ...
    "Reliability", "reliable", "Durability", "volatile", "Depth", 100);
cloudSub = ros2subscriber(node, "/velodyne_points", "sensor_msgs/PointCloud2", @onCloud, ...
    "Reliability", "besteffort", "Depth", 5);
pause(2)
ready = getenv("READY_FILE");
if ready ~= "", fclose(fopen(ready, "w")); end
fprintf("購読を始めた（/tf・/velodyne_points）。再生を待つ\n");

lastN = 0; idle = 0; started = false; t0 = tic;
while toc(t0) < duration
    pause(0.5)
    if TFLOG.n > lastN
        lastN = TFLOG.n; idle = 0; started = true;
    else
        idle = idle + 0.5;
    end
    if started && idle >= stopIdle, break; end
end
clear tfSub cloudSub
if TFLOG.n == 0
    error("/tf の map → * が 1 つも届かなかった（再生・ROS_DOMAIN_ID を確かめる）")
end
fprintf("受け取り終わり（%.0f s）。ros2tf から引く\n", toc(t0));

%% 記録した TF を刻みごとにまとめる（同じ刻みは何度も届く）
child = TFLOG.child(1:TFLOG.n);
[names, ~, ic] = unique(child);
[~, im] = max(accumarray(ic, 1));
baseFrame = names(im);                                  % map の子で一番多いもの（Gazebo base_footprint、実機 base_link）
D = TFLOG.data(child == baseFrame, :);
ns = int64(D(:, 1)) * int64(1e9) + int64(D(:, 2));
[u, ~, g] = unique(ns);                                 % 刻み（昇順）
order = (1:numel(ns))';
firstIdx = accumarray(g, order, [], @min);
lastIdx = accumarray(g, order, [], @max);
sends = accumarray(g, 1);
Vfirst = D(firstIdx, 3:9);                              % [x y z qw qx qy qz]
Vlast = D(lastIdx, 3:9);
m = numel(u);
gapsMs = double(diff(u)) * 1e-6;
diffFL_P = vecnorm(Vlast(:, 1:3) - Vfirst(:, 1:3), 2, 2);
diffFL_R = arrayfun(@(k) TfMath.qangle(Vfirst(k, 4:7), Vlast(k, 4:7)), (1:m)') / deg;

%% ros2tf から引いて比べる
cloud = int64(CLOUDLOG(:, 1)) * int64(1e9) + int64(CLOUDLOG(:, 2));
cloud = unique(cloud);
nCloudOutside = sum(cloud < u(1) | cloud > u(end));    % EKF の初期化前・最後の TF の後
cloud = cloud(cloud >= u(1) & cloud <= u(end));
mid = u(1:end-1) + idivide(diff(u), int64(2));
% 比べる補間：前の刻みの値 old、後の刻みの値 new、刻みちょうどの値 exact
VexactRule = Vlast;  VexactRule(1, :) = Vfirst(1, :);   % tf2 は一番古い刻みちょうどだけ最初に入った値を返す
models = struct("name", {"最後に届いた値どうし", "最初に届いた値どうし", "前の刻みは最後・後の刻みは最初の値"}, ...
    "label", {"Last sent values", "First sent values", "Older stamp: last sent, newer stamp: first sent"}, ...
    "old", {Vlast, Vfirst, Vlast}, "new", {Vlast, Vfirst, Vfirst}, "exact", {Vlast, Vfirst, VexactRule});
sets = struct("name", {"点群の stamp", "刻みの中点", "刻みちょうど"}, "label", {"Point cloud stamps", "Midpoints", "TF stamps"}, ...
    "t", {cloud, mid, u});
tic
for k = 1:numel(sets)
    t = sets(k).t;
    got = zeros(numel(t), 7);
    for i = 1:numel(t)
        got(i, :) = TfMath.lookup(tftree, "map", baseFrame, TfMath.stampFromNs(t(i)));
    end
    sets(k).got = got;
    for j = 1:numel(models)
        [sets(k).errP(:, j), sets(k).errR(:, j)] = poseErr(got, interpAt(u, models(j), t));
    end
end
% map → LiDAR（点群の stamp）：map → base の補間（前は最後・後は最初の値）に取付を掛けたもの
mount = TfMath.lookup(tftree, baseFrame, lidarFrame, TfMath.stampMsg(0, 0));   % 時刻 0 は最新（static）
lidar = struct("got", [], "errP", [], "errR", []);
if all(~isnan(mount))
    E = interpAt(u, models(3), cloud);
    lidar.got = zeros(numel(cloud), 7);
    Ev = zeros(numel(cloud), 7);
    for i = 1:numel(cloud)
        lidar.got(i, :) = TfMath.lookup(tftree, "map", lidarFrame, TfMath.stampFromNs(cloud(i)));
        [Ev(i, 1:3), Ev(i, 4:7)] = TfMath.compose(E(i, 1:3), E(i, 4:7), mount(1:3), mount(4:7));
    end
    [lidar.errP, lidar.errR] = poseErr(lidar.got, Ev);
end
fprintf("引いた時間 %.0f s\n", toc);

%% 表示
tolP = 1e-6;  tolR = 1e-4;
sendCounts = accumarray(sends, 1);
fprintf("\nekf_localizer の TF：map → %s、刻み %d 個（%.1f s）、受け取った TF %d 個\n", baseFrame, m, double(u(end) - u(1)) * 1e-9, size(D, 1));
fprintf("  1 刻みが届いた回数：%s\n", strjoin(compose("%d 回 %d 個", find(sendCounts), sendCounts(sendCounts > 0)), "、"));
fprintf("  刻みの間隔：20 ms が %d 個、それ以外 %d 個（最大 %.1f ms）\n", sum(abs(gapsMs - 20) < 1e-6), sum(abs(gapsMs - 20) >= 1e-6), max(gapsMs));
fprintf("  同じ刻みの最初と最後の値の差：位置 中央値 %.2e・最大 %.2e m、回転 中央値 %.2e・最大 %.2e deg\n", ...
    median(diffFL_P), max(diffFL_P), median(diffFL_R), max(diffFL_R));
fprintf("  点群の stamp %d 個（TF の範囲の外 %d 個は除いた）\n", numel(cloud), nCloudOutside);
fprintf("\nros2tf の getTransform と、記録した TF から計算した補間（lerp + slerp）の差（最大）：\n");
match = true(1, numel(models));
for k = 1:numel(sets)
    s = sets(k);
    if isempty(s.t)
        fprintf("  map → %s、%s：0 個\n", baseFrame, s.name);
        continue
    end
    nFail = sum(any(isnan(s.got), 2));
    fprintf("  map → %s、%s（%d 個、引けなかった %d 個）\n", baseFrame, s.name, numel(s.t), nFail);
    for j = 1:numel(models)
        nOver = sum(s.errP(:, j) >= tolP | s.errR(:, j) >= tolR);
        fprintf("    %-34s 位置 %.2e m、回転 %.2e deg（ずれる %d 個）\n", models(j).name + "：", max(s.errP(:, j)), max(s.errR(:, j)), nOver);
        match(j) = match(j) && nFail == 0 && nOver == 0;
    end
end
if ~isempty(lidar.got) && ~isempty(cloud)
    fprintf("  map → %s、点群の stamp（前は最後・後は最初の値の補間に取付を掛けたもの）：位置 %.2e m、回転 %.2e deg（引けなかった %d 個）\n", ...
        lidarFrame, max(lidar.errP), max(lidar.errR), sum(any(isnan(lidar.got), 2)));
else
    fprintf("  %s → %s が引けなかったので map → %s は比べていない\n", baseFrame, lidarFrame, lidarFrame);
end
if any(match)
    fprintf("→ getTransform は「%s」の lerp + slerp と一致（%.0e m・%.0e deg 未満）\n", strjoin([models(match).name], "」・「"), tolP, tolR);
else
    fprintf("→ どの補間とも一致しない\n");
end

%% 保存と図
results = struct("matlab", version, "baseFrame", baseFrame, "lidarFrame", lidarFrame, "stampsNs", u, "sends", sends, ...
    "valueFirst", Vfirst, "valueLast", Vlast, "models", {rmfield(models, ["old", "new", "exact"])}, "sets", {sets}, "lidar", lidar, "mount", mount, "cloudOutside", nCloudOutside);
out = getenv("TEST_OUT");
if out ~= ""
    save(out, "-struct", "results");
    fprintf("\n結果：%s\n", out);
end
figPath = getenv("TEST_FIG");
if figPath ~= ""
    plot_tf_interpolation_ekf(results, figPath);
    fprintf("図：%s\n", figPath);
end
clear tftree node

%% ローカル関数
function onTf(msg)
    global TFLOG
    for k = 1:numel(msg.transforms)
        tr = msg.transforms(k);
        if ~strcmp(tr.header.frame_id, 'map'), continue; end
        TFLOG.n = TFLOG.n + 1;
        t = tr.transform.translation;
        q = tr.transform.rotation;
        TFLOG.data(TFLOG.n, :) = [double(tr.header.stamp.sec), double(tr.header.stamp.nanosec), t.x, t.y, t.z, q.w, q.x, q.y, q.z];
        TFLOG.child(TFLOG.n) = string(tr.child_frame_id);
    end
end

function onCloud(msg)
    global CLOUDLOG
    CLOUDLOG(end + 1, :) = [double(msg.header.stamp.sec), double(msg.header.stamp.nanosec)];
end

function v = envNum(name, default)
    v = str2double(getenv(name));
    if isnan(v), v = default; end
end

function V = interpAt(u, model, t)
    % 刻み u（int64 ns、昇順）の時刻 t の値。刻みちょうどなら model.exact、間なら前の刻みの model.old と後の刻みの model.new を lerp + slerp
    relU = double(u - u(1));
    relT = double(t - u(1));
    a = discretize(relT, relU);                          % relU(a) <= relT < relU(a+1)（最後の刻みちょうどは最後の区間）
    s = (relT - relU(a)) ./ (relU(a + 1) - relU(a));
    V = zeros(numel(t), 7);
    for i = 1:numel(t)
        if s(i) == 0 || s(i) == 1
            V(i, :) = model.exact(a(i) + (s(i) == 1), :);
            continue
        end
        A = model.old(a(i), :);
        B = model.new(a(i) + 1, :);
        V(i, 1:3) = (1 - s(i)) * A(1:3) + s(i) * B(1:3);
        V(i, 4:7) = TfMath.slerp(A(4:7), B(4:7), s(i));
    end
end

function [eP, eR] = poseErr(got, ref)
    % 位置の差 [m] と回転の差 [deg]（引けなかったものは NaN）
    eP = vecnorm(got(:, 1:3) - ref(:, 1:3), 2, 2);
    eR = nan(size(got, 1), 1);
    for i = 1:size(got, 1)
        if ~any(isnan(got(i, :)))
            eR(i) = TfMath.qangle(got(i, 4:7), ref(i, 4:7)) * 180 / pi;
        end
    end
end
