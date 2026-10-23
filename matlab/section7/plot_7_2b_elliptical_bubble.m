% plot_7_2b_elliptical_bubble.m
% Section 7.2 - Elliptical Reactive Bubble
%     R_react(theta) = a b / sqrt( (b cos(theta))^2 + (a sin(theta))^2 )
% Left : top-down picture of the bubble (ellipse) versus a plain circular bubble.
% Right: the reaction radius as a function of the bearing theta.

a = 1.10;                                    % semi-axis straight ahead   (m)
b = 0.32;                                    % semi-axis directly sideways (m)
wallY = 0.60;                                % half-width of an example corridor (m)

th = linspace(-pi, pi, 721);
R  = a*b ./ sqrt((b*cos(th)).^2 + (a*sin(th)).^2);   % R_react(theta)
ex = R .* cos(th);   ey = R .* sin(th);              % ellipse outline (x forward, y left)

blue = [0.00 0.45 0.74];  orange = [0.85 0.33 0.10];  red = [0.80 0.10 0.10];  gray = [0.45 0.45 0.45];

fig = figure('Color', 'w', 'Position', [100 100 1100 520]);
try, theme(fig, 'light'); catch, end   % force light colours (R2025a+ follows the OS dark mode)
tl  = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% ---------- (a) top-down view -------------------------------------------------
nexttile; hold on; box on; grid on;
fill(ex, ey, blue, 'FaceAlpha', 0.20, 'EdgeColor', blue, 'LineWidth', 2.2);
plot(a*cos(th), a*sin(th), '--', 'Color', gray, 'LineWidth', 1.4);          % naive circular bubble

xw = sqrt(a^2 - wallY^2);                                                   % wall part inside the circle
for s = [-1 1]
    plot([-1.35 1.5], s*wallY*[1 1], '-', 'Color', [0.25 0.25 0.25], 'LineWidth', 4);
    plot([-xw xw],    s*wallY*[1 1], '-', 'Color', red, 'LineWidth', 4);    % false-trigger zone
end
fill([0.95 1.30 1.30 0.95], [-0.25 -0.25 0.25 0.25], orange, 'FaceAlpha', 0.85, 'EdgeColor', 'k');
text(1.125, 0.33, 'obstacle', 'HorizontalAlignment', 'center', 'FontSize', 10);

for ang = [0 45 90 -45 -90]                                                  % sample bearings
    r = a*b / sqrt((b*cosd(ang))^2 + (a*sind(ang))^2);
    plot([0 r*cosd(ang)], [0 r*sind(ang)], '-', 'Color', [0.2 0.2 0.2], 'LineWidth', 1);
    plot(r*cosd(ang), r*sind(ang), 'o', 'MarkerFaceColor', blue, 'MarkerEdgeColor', 'k', 'MarkerSize', 6);
end
text(0.40, 0.05, 'R = 1.10 m', 'FontSize', 10, 'VerticalAlignment', 'bottom');
text(0.42, 0.45, sprintf('R = %.2f m', a*b/sqrt((b*cosd(45))^2 + (a*sind(45))^2)), 'FontSize', 10);
text(-0.05, 0.40, 'R = 0.32 m', 'FontSize', 10, 'HorizontalAlignment', 'right');

plot(0, 0, 's', 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k', 'MarkerSize', 9);   % drone
quiver(0, 0, 0.30, 0, 'AutoScale', 'off', 'Color', 'k', 'LineWidth', 2, 'MaxHeadSize', 1.2);

hh(1) = fill(NaN, NaN, blue, 'FaceAlpha', 0.20, 'EdgeColor', blue, 'LineWidth', 2.2);
hh(2) = plot(NaN, NaN, '--', 'Color', gray, 'LineWidth', 1.4);
hh(3) = plot(NaN, NaN, '-', 'Color', [0.25 0.25 0.25], 'LineWidth', 4);
hh(4) = plot(NaN, NaN, '-', 'Color', red, 'LineWidth', 4);
lgd = legend(hh, {'elliptical bubble  R_{react}(\theta)', 'circular bubble of radius a (naive)', ...
                  'corridor wall (illustrative, y = \pm 0.60 m)', 'wall inside the circle: false trigger'}, ...
             'NumColumns', 4);
lgd.Layout.Tile = 'south';
axis equal; xlim([-1.4 1.55]); ylim([-0.95 0.95]);
xlabel('x, forward  (m)'); ylabel('y, left  (m)');
title('Bubble seen from above');
set(gca, 'FontSize', 11);

% ---------- (b) R_react versus bearing ---------------------------------------
nexttile; hold on; box on; grid on;
tdeg = rad2deg(th(th >= 0));
plot(tdeg, R(th >= 0), '-', 'Color', blue, 'LineWidth', 2.5);
yline(a, '--', 'Color', gray, 'LineWidth', 1.4);
yline(b, ':',  'Color', gray, 'LineWidth', 1.4);
mk = [0 45 90];
rk = a*b ./ sqrt((b*cosd(mk)).^2 + (a*sind(mk)).^2);
plot(mk, rk, 'o', 'MarkerFaceColor', orange, 'MarkerEdgeColor', 'k', 'MarkerSize', 8);
for q = 1:3
    text(mk(q) + 4, rk(q) + 0.05, sprintf('%d^{\\circ}: %.2f m', mk(q), rk(q)), 'FontSize', 11);
end
text(178, a + 0.04, 'a = 1.10 m (circle)', 'HorizontalAlignment', 'right', 'Color', gray, 'FontSize', 10);
text(178, b + 0.04, 'b = 0.32 m', 'HorizontalAlignment', 'right', 'Color', gray, 'FontSize', 10);
xlim([0 180]); ylim([0 1.3]); xticks(0:30:180);
xlabel('bearing from heading  \theta  (deg)'); ylabel('reaction radius  R_{react}(\theta)  (m)');
title('R_{react}(\theta) = a b / \surd((b cos\theta)^2 + (a sin\theta)^2)');
set(gca, 'FontSize', 11);

outDir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'assets', 'section7');
if ~exist(outDir, 'dir'), mkdir(outDir); end
exportgraphics(fig, fullfile(outDir, 'fig_7_2b_elliptical_bubble.png'), 'Resolution', 200);
