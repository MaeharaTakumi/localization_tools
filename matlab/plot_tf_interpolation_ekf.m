function plot_tf_interpolation_ekf(r, figPath)
% check_tf_interpolation_ekf.m の結果の図（dev_log.md の 105）。r は結果の構造体か .mat のパス。
% 上：同じ刻みの最初と最後の値の差（yaw）が一番大きいところの 120 ms（ekf_localizer の刻みと getTransform）。
% 中・下：getTransform と 3 つの補間の差（対数）。.mat から描き直すとき（ホスト）：
%   matlab -batch "addpath('<localization_tools>/matlab'); plot_tf_interpolation_ekf('<.mat>', '<.png>')"
if ~isstruct(r)
    r = load(r);
end
series = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"];
truthColor = "#a3a29c";
u = r.stampsNs;
tSec = @(t) double(t - u(1)) * 1e-9;
m = numel(u);
dYaw = arrayfun(@(k) TfMath.qangle(r.valueFirst(k, 4:7), r.valueLast(k, 4:7)), (1:m)');
[~, kc] = max(dYaw(4:end-3));
kc = kc + 3;
win = kc - 3:kc + 3;
t0w = u(win(1));
yaw0 = TfMath.q2yaw(r.valueLast(win(1), 4:7));
yawOf = @(V) arrayfun(@(k) atan2(sin(TfMath.q2yaw(V(k, 4:7)) - yaw0), cos(TfMath.q2yaw(V(k, 4:7)) - yaw0)) / pi * 180, (1:size(V, 1))');
msOf = @(t) double(t - t0w) * 1e-6;

fig = figure("Visible", "off", "Position", [0 0 780 900], "Color", "w");
tl = tiledlayout(fig, 3, 1, "TileSpacing", "compact", "Padding", "compact");
title(tl, "TF from ekf_localizer looked up with ros2tf getTransform (MATLAB R" + extractBetween(string(r.matlab), "(R", ")") + ")", ...
    "FontWeight", "bold", "FontSize", 11, "Interpreter", "none")

% 上：刻みの値と getTransform
ax = nexttile(tl);
hold(ax, "on")
yLast = yawOf(r.valueLast(win, :));
yFirst = yawOf(r.valueFirst(win, :));
h = gobjects(0);
h(end + 1) = plot(ax, msOf(u(win)), yLast, "-o", "Color", truthColor, "LineWidth", 3.6, "MarkerSize", 7, ...
    "MarkerFaceColor", truthColor, "DisplayName", "EKF TF stamps: last sent value (linear between)");
for k = 1:numel(win) - 1   % 前の刻みの最後の値から後の刻みの最初の値へ
    seg = plot(ax, msOf(u(win(k:k+1))), [yLast(k) yFirst(k+1)], "-", "Color", series(1), "LineWidth", 1.2);
end
seg.DisplayName = "Older stamp last value to newer stamp first value";
h(end + 1) = seg;
h(end + 1) = plot(ax, msOf(u(win)), yFirst, "s", "Color", series(2), "MarkerFaceColor", series(2), "MarkerSize", 6, ...
    "LineStyle", "none", "DisplayName", "EKF TF stamps: first sent value");
mk = ["o", "d"];
for k = 1:2
    s = r.sets(k);
    in = s.t > u(win(1)) & s.t < u(win(end));
    if ~any(in), continue; end
    h(end + 1) = plot(ax, msOf(s.t(in)), yawOf(s.got(in, :)), mk(k), "Color", series(1), "MarkerFaceColor", series(1), ...
        "MarkerSize", 5, "LineStyle", "none", "DisplayName", "MATLAB getTransform at " + lower(s.label)); %#ok<AGROW>
end
xlabel(ax, "Time from " + sprintf("%.2f", tSec(t0w)) + " s [ms]")
ylabel(ax, "Yaw change [deg]")
% 凡例は線の無い側の角に置き、その分だけ縦に余白をとる（yaw が増えるなら右下、減るなら右上）
yAll = [yLast; yFirst];
yr = [min(yAll) max(yAll)];
pad = 0.9 * diff(yr);
if yLast(end) >= yLast(1)
    ylim(ax, [yr(1) - pad, yr(2) + 0.05 * diff(yr)]);  loc = "southeast";
else
    ylim(ax, [yr(1) - 0.05 * diff(yr), yr(2) + pad]);  loc = "northeast";
end
legend(ax, h, "Location", loc, "Box", "off", "FontSize", 8)
styleAxes(ax)

% 中・下：getTransform と 3 つの補間の差
floorVal = 1e-12;
names = ["Rotation difference [deg]", "Position difference [m]"];
colors = [series(3), series(2), series(1)];   % 最後どうし：緑、最初どうし：橙、前は最後・後は最初：青
order = [2 1 3];                              % 後ろから描く（後ろほど大きい点）
sizes = [11 8 5];
for p = 1:2
    ax = nexttile(tl);
    hold(ax, "on")
    tAll = [];  e = [];
    for k = 1:numel(r.sets)
        s = r.sets(k);
        tAll = [tAll; tSec(s.t)]; %#ok<AGROW>
        if p == 1, e = [e; s.errR]; else, e = [e; s.errP]; end %#ok<AGROW>
    end
    hl = gobjects(0);
    for n = 1:3
        j = order(n);
        plot(ax, tAll, max(e(:, j), floorVal), ".", "Color", colors(j), "MarkerSize", sizes(n), "HandleVisibility", "off")
        hl(end + 1) = plot(ax, nan, nan, "o", "Color", colors(j), "MarkerFaceColor", colors(j), "MarkerSize", 6, ...
            "DisplayName", "Vs. " + r.models(j).label); %#ok<AGROW>
    end
    set(ax, "YScale", "log")
    ylim(ax, [floorVal / 3, 10])
    ylabel(ax, names(p))
    text(ax, 0.01, 0.12, "Differences below 1e-12 are drawn at 1e-12", "Units", "normalized", ...
        "VerticalAlignment", "bottom", "FontSize", 8, "Color", [0.32 0.32 0.31])
    styleAxes(ax)
end
xlabel(ax, "Time from first EKF TF [s]")
lg = legend(ax, hl, "Box", "off", "FontSize", 8, "NumColumns", 1);
lg.Layout.Tile = "south";
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
end
