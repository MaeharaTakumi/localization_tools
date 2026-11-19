% ros2tf の getTransform で時刻を指定して引いた TF が、前後の TF の線形補間（位置は線形、回転は最短経路の slerp）になって
% いるかを確かめる（CR_debug の SensorFetcher.lookup と同じ canTransform → getTransform(tftree, 親, 子, stamp)）。
% MATLAB の中で既知の TF を送り（map → base_link を 1 s おきに 3 つ、base_link → velodyne を /tf_static）、その間の時刻で引いた値を
% 自分で計算した補間と比べる。2 つ目の区間は yaw が 150° → -120°（±180° をまたぐ）。ROS の録画も Gazebo も要らない。
% ホストで（R2024b の例）：
%   /usr/local/MATLAB/R2024b/bin/matlab -batch "run('<localization_tools>/matlab/check_tf_interpolation.m')"
% 環境変数：REPO（fastdds_localhost.xml のある whill_common_matlab のフォルダ。既定は localization_tools と同じ src/ の中）、
%   TEST_DOMAIN（ROS_DOMAIN_ID。既定 98。Gazebo の 11・再生の 99 と混ざらない）、TEST_FIG（図の .png）、TEST_OUT（結果の .mat）。
% 比べる補間：lerp+slerp（期待）、lerp+nlerp（四元数を線形に混ぜて正規化）、Euler 線形（roll・pitch・yaw を線形。±180° を遠回り）、
% 前の TF のまま（0 次ホールド）。最大誤差が 1e-6 m・1e-4° 未満なら一致とする。区間の外（最初より前・最後より後）は引けるかも見る。

here = fileparts(mfilename("fullpath"));
repo = getenv("REPO");
if repo == ""
    repo = fullfile(here, "..", "..", "whill_common_matlab-main _ndtDebug");
end
setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
setenv("FASTRTPS_DEFAULT_PROFILES_FILE", fullfile(repo, "fastdds_localhost.xml"))
setenv("ROS_LOCALHOST_ONLY", "0")
domain = str2double(getenv("TEST_DOMAIN"));
if isnan(domain), domain = 98; end
fprintf("MATLAB %s, ROS_DOMAIN_ID %d\n", version, domain);

%% 送る TF（map → base_link のキーフレーム 3 つと、static の base_link → velodyne）
deg = pi/180;
keyP = [0 0 0; 2 1 0.3; 3 3 0.1];                         % 位置 [m]
keyRpy = [0 0 0; 10 -5 150; 0 5 -120] * deg;              % roll, pitch, yaw [rad]
keyQ = cell2mat(arrayfun(@(i) rpy2q(keyRpy(i, :)), (1:3)', "UniformOutput", false));   % [w x y z]
mountP = [-0.17 0 1.1377];
mountQ = rpy2q([0.01 -0.02 0.03]);

subNode = ros2node("tf_interp_check_sub", domain);
pubNode = ros2node("tf_interp_check_pub", domain);
tftree = ros2tf(subNode);
pubTree = ros2tf(pubNode);
pause(2)   % 発見を待つ（/tf は volatile なので、つながる前に送ると届かない）

nowTime = ros2time(subNode, "now");
t0 = int64(nowTime.sec);                                      % キーフレームは t0, t0+1, t0+2 [s]（ナノ秒 0）
msgs = arrayfun(@(i) tfMsg("map", "base_link", t0 + i - 1, 0, keyP(i, :), keyQ(i, :)), 1:3);
sendTransform(pubTree, tfMsg("base_link", "velodyne", t0, 0, mountP, mountQ), UseStatic=true)
for i = 1:numel(msgs)   % 1 つずつ送る（sendTransform は配列を渡しても最初の 1 つしか /tf に出さない。R2024b・R2026a。dev_log.md の 104）
    sendTransform(pubTree, msgs(i))
    pause(0.1)
end
lastStamp = stampMsg(t0 + 2, 0);
ok = false;
for k = 1:50
    if canTransform(tftree, "map", "base_link", lastStamp) && canTransform(tftree, "map", "velodyne", lastStamp)
        ok = true;
        break
    end
    pause(0.1)
end
if ~ok
    error("送った TF が 5 s 待っても ros2tf に届かない（TEST_DOMAIN・DDS の設定を確かめる）")
end

%% 時刻を指定して引く（0〜2 s を 0.05 s おき + キーフレームの近く）
relNs = unique([int64(round((0:0.05:2) * 1e9)), int64([1 999 1001 1999] * 1e6)]);
n = numel(relNs);
gotB = zeros(n, 7);   % map → base_link [x y z w qx qy qz]
gotV = zeros(n, 7);   % map → velodyne
for i = 1:n
    st = stampMsg(t0 + idivide(relNs(i), int64(1e9)), mod(relNs(i), int64(1e9)));
    gotB(i, :) = lookup(tftree, "map", "base_link", st);
    gotV(i, :) = lookup(tftree, "map", "velodyne", st);
end

%% 自分で計算した補間と比べる
models = ["Lerp + slerp (expected)", "Lerp + nlerp", "Euler linear", "Previous TF (hold)"];
errP = zeros(n, numel(models));
errR = zeros(n, numel(models));
for i = 1:n
    tSec = double(relNs(i)) * 1e-9;
    seg = min(floor(tSec), 1);                            % 区間 0：キー 1→2、区間 1：キー 2→3
    s = tSec - seg;
    a = seg + 1;  b = seg + 2;
    p = (1 - s) * keyP(a, :) + s * keyP(b, :);
    cand = {p, slerp(keyQ(a, :), keyQ(b, :), s); ...
            p, nlerp(keyQ(a, :), keyQ(b, :), s); ...
            p, rpy2q((1 - s) * keyRpy(a, :) + s * keyRpy(b, :)); ...
            keyP(a, :), keyQ(a, :)};
    for m = 1:numel(models)
        errP(i, m) = norm(gotB(i, 1:3) - cand{m, 1});
        errR(i, m) = qangle(gotB(i, 4:7), cand{m, 2}) / deg;
    end
end
% map → velodyne は「map → base_link の lerp+slerp」に static の取付を掛けたものと比べる
errVP = zeros(n, 1);
errVR = zeros(n, 1);
for i = 1:n
    tSec = double(relNs(i)) * 1e-9;
    seg = min(floor(tSec), 1);
    s = tSec - seg;
    qB = slerp(keyQ(seg + 1, :), keyQ(seg + 2, :), s);
    pB = (1 - s) * keyP(seg + 1, :) + s * keyP(seg + 2, :);
    errVP(i) = norm(gotV(i, 1:3) - (pB + qrot(qB, mountP)));
    errVR(i) = qangle(gotV(i, 4:7), qmul(qB, mountQ)) / deg;
end

tolP = 1e-6;  tolR = 1e-4;
fprintf("\n%d 個の時刻で引いた map → base_link と各補間の最大誤差：\n", n);
fprintf("  %-24s %14s %14s\n", "Model", "Position [m]", "Rotation [deg]");
for m = 1:numel(models)
    fprintf("  %-24s %14.3e %14.3e\n", models(m), max(errP(:, m)), max(errR(:, m)));
end
fprintf("  map → velodyne（lerp+slerp に static の取付を掛けたもの）：位置 %.3e m、回転 %.3e deg\n", max(errVP), max(errVR));
passB = max(errP(:, 1)) < tolP && max(errR(:, 1)) < tolR;
passV = max(errVP) < tolP && max(errVR) < tolR;
if passB && passV
    fprintf("→ 線形補間（位置は線形、回転は最短経路の slerp）と一致（%.0e m・%.0e deg 未満）\n", tolP, tolR);
else
    fprintf("→ 線形補間と一致しない（上の表で誤差が最も小さい補間を見る）\n");
end

%% 区間の外（最初より前・最後より後）
fprintf("\n区間の外：\n");
outside = {"最初の TF の 10 ms 前", t0 - 1, int64(990e6); "最後の TF の 10 ms 後", t0 + 2, int64(10e6)};
for k = 1:size(outside, 1)
    st = stampMsg(outside{k, 2}, outside{k, 3});
    can = canTransform(tftree, "map", "base_link", st);
    try
        getTransform(tftree, "map", "base_link", st);
        res = "getTransform は値を返した";
    catch ex
        res = "getTransform はエラー（" + ex.identifier + "）";
    end
    fprintf("  %s：canTransform = %d、%s\n", outside{k, 1}, can, res);
end

%% 結果の保存と図
results = struct("relSec", double(relNs') * 1e-9, "gotBase", gotB, "gotVelodyne", gotV, "models", models, ...
    "errPos", errP, "errRotDeg", errR, "errVelodynePos", errVP, "errVelodyneRotDeg", errVR, ...
    "keyPos", keyP, "keyRpy", keyRpy, "mountPos", mountP, "mountQ", mountQ, "matlab", version);
out = getenv("TEST_OUT");
if out ~= ""
    save(out, "-struct", "results");
    fprintf("\n結果：%s\n", out);
end
figPath = getenv("TEST_FIG");
if figPath ~= ""
    plotResult(results, keyQ, figPath);
    fprintf("図：%s\n", figPath);
end
clear tftree pubTree subNode pubNode

%% ローカル関数
function v = lookup(tftree, parent, child, st)
    % parent → child を st の時刻で引いて [x y z w qx qy qz] にする（CR_debug の SensorFetcher.lookup と同じ呼び方）
    if ~canTransform(tftree, parent, child, st)
        error("%s → %s が %d.%09d s で引けない", parent, child, st.sec, st.nanosec)
    end
    tf = getTransform(tftree, parent, child, st);
    t = tf.transform.translation;
    q = tf.transform.rotation;
    v = [t.x t.y t.z q.w q.x q.y q.z];
end

function st = stampMsg(sec, nanosec)
    st = ros2message("builtin_interfaces/Time");
    st.sec = int32(sec);
    st.nanosec = uint32(nanosec);
end

function msg = tfMsg(parent, child, sec, nanosec, p, q)
    msg = ros2message("geometry_msgs/TransformStamped");
    msg.header.frame_id = char(parent);
    msg.header.stamp = stampMsg(sec, nanosec);
    msg.child_frame_id = char(child);
    msg.transform.translation.x = p(1);
    msg.transform.translation.y = p(2);
    msg.transform.translation.z = p(3);
    msg.transform.rotation.w = q(1);
    msg.transform.rotation.x = q(2);
    msg.transform.rotation.y = q(3);
    msg.transform.rotation.z = q(4);
end

function q = rpy2q(rpy)
    % roll, pitch, yaw（ZYX）→ [w x y z]
    cr = cos(rpy(1)/2); sr = sin(rpy(1)/2);
    cp = cos(rpy(2)/2); sp = sin(rpy(2)/2);
    cy = cos(rpy(3)/2); sy = sin(rpy(3)/2);
    q = [cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy];
end

function yaw = q2yaw(q)
    yaw = atan2(2*(q(1)*q(4) + q(2)*q(3)), 1 - 2*(q(3)^2 + q(4)^2));
end

function q = qmul(a, b)
    q = [a(1)*b(1) - a(2)*b(2) - a(3)*b(3) - a(4)*b(4), ...
         a(1)*b(2) + a(2)*b(1) + a(3)*b(4) - a(4)*b(3), ...
         a(1)*b(3) - a(2)*b(4) + a(3)*b(1) + a(4)*b(2), ...
         a(1)*b(4) + a(2)*b(3) - a(3)*b(2) + a(4)*b(1)];
end

function v = qrot(q, p)
    r = qmul(qmul(q, [0 p]), [q(1) -q(2:4)]);
    v = r(2:4);
end

function a = qangle(q1, q2)
    % 2 つの回転の間の角度 [rad]（小さい角度でも精度が落ちない atan2 で）
    d = qmul([q1(1) -q1(2:4)], q2);
    a = 2 * atan2(norm(d(2:4)), abs(d(1)));
end

function q = slerp(a, b, s)
    % 最短経路の球面線形補間（tf2::Quaternion::slerp と同じく内積が負なら b の符号を反転）
    if dot(a, b) < 0, b = -b; end
    th = acos(min(1, dot(a, b)));
    if th < 1e-12
        q = a;
    else
        q = (sin((1 - s)*th) * a + sin(s*th) * b) / sin(th);
    end
end

function q = nlerp(a, b, s)
    if dot(a, b) < 0, b = -b; end
    q = (1 - s) * a + s * b;
    q = q / norm(q);
end

function plotResult(r, keyQ, figPath)
    % 上：yaw（getTransform と期待の slerp、Euler 線形）。下：getTransform と各補間の回転の差（対数）
    series = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"];
    truthColor = "#a3a29c";
    t = r.relSec;
    tt = linspace(0, 2, 401)';
    yawSlerp = zeros(size(tt));  yawEuler = zeros(size(tt));
    for i = 1:numel(tt)
        seg = min(floor(tt(i)), 1);  s = tt(i) - seg;
        yawSlerp(i) = q2yaw(slerp(keyQ(seg + 1, :), keyQ(seg + 2, :), s));
        yawEuler(i) = (1 - s) * r.keyRpy(seg + 1, 3) + s * r.keyRpy(seg + 2, 3);
    end
    yawGot = arrayfun(@(i) q2yaw(r.gotBase(i, 4:7)), (1:numel(t))');

    fig = figure("Visible", "off", "Position", [0 0 760 620], "Color", "w");
    tl = tiledlayout(fig, 2, 1, "TileSpacing", "compact", "Padding", "compact");
    title(tl, "TF looked up at a specified time with ros2tf getTransform (MATLAB R" + version("-release") + ")", "FontWeight", "bold", "FontSize", 11)

    ax = nexttile(tl);
    hold(ax, "on")
    plot(ax, tt, rad2deg(unwrap(yawSlerp)), "Color", truthColor, "LineWidth", 3.6, "DisplayName", "Expected: lerp + slerp")
    plot(ax, tt, rad2deg(yawEuler), "Color", series(2), "LineWidth", 1.8, "DisplayName", "Euler linear (not shortest path)")
    plot(ax, t, rad2deg(unwrap(yawGot)), "o", "Color", series(1), "MarkerFaceColor", series(1), "MarkerSize", 4, ...
        "LineStyle", "none", "DisplayName", "MATLAB getTransform")
    xline(ax, [0 1 2], "Color", [0.75 0.75 0.75], "HandleVisibility", "off")
    ylabel(ax, "Yaw of map \rightarrow base\_link, unwrapped [deg]")
    legend(ax, "Location", "southwest", "Box", "off")
    styleAxes(ax)

    ax = nexttile(tl);
    floorVal = 1e-12;
    widths = linspace(2.6, 1.1, 4);
    for m = 1:4
        semilogy(ax, t, max(r.errRotDeg(:, m), floorVal), "-o", "Color", series(m), "LineWidth", widths(m), ...
            "MarkerSize", 3, "MarkerFaceColor", series(m), "DisplayName", r.models(m))
        hold(ax, "on")
    end
    xline(ax, [0 1 2], "Color", [0.75 0.75 0.75], "HandleVisibility", "off")
    ylim(ax, [floorVal / 3, 1e3])
    text(ax, 0.02, 0.97, "Differences below 1e-12 deg are drawn at 1e-12", "Units", "normalized", ...
        "VerticalAlignment", "top", "FontSize", 8, "Color", [0.32 0.32 0.31])
    xlabel(ax, "Time from first TF [s]")
    ylabel(ax, "Rotation difference from getTransform [deg]")
    legend(ax, "Location", "southeast", "Box", "off", "NumColumns", 2)
    styleAxes(ax)
    exportgraphics(fig, figPath, "Resolution", 150)
    close(fig)
end

function styleAxes(ax)
    % 枠は 4 辺とも黒（box on）、格子は薄く
    box(ax, "on")
    set(ax, "XColor", "k", "YColor", "k", "LineWidth", 0.8, "FontSize", 9)
    grid(ax, "on")
    ax.GridColor = [0.89 0.89 0.87];
    ax.GridAlpha = 1;
    xlim(ax, [-0.05 2.05])
end
