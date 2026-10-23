% plot_7_1_goal_direction.m
% Section 7.1 - Horizontal unit direction toward the goal
%     d_g = (p_goal - p) / || p_goal - p ||_2        (xy-plane only)
% Whatever the drone position, d_g always points at the goal and has length 1.

pGoal = [6.0, 0.0];                                  % goal (x, y) in metres
[X, Y] = meshgrid(0:1:5, -2:1:2);                    % grid of possible drone positions

Dx  = pGoal(1) - X;   Dy = pGoal(2) - Y;             % horizontal part of p_goal - p
Nrm = hypot(Dx, Dy);                                 % || p_goal - p ||_2
Ux  = Dx ./ Nrm;      Uy = Dy ./ Nrm;                % unit direction d_g at every grid point

% Worked example from the text: drone at p = [2.0, 0.5, 1.0]
p  = [2.0, 0.5];
dg = (pGoal - p) / norm(pGoal - p);                  % -> [0.9923, -0.1240]

blue = [0.00 0.45 0.74];  orange = [0.85 0.33 0.10];  gray = [0.55 0.55 0.55];

fig = figure('Color', 'w', 'Position', [100 100 900 620]);
try, theme(fig, 'light'); catch, end   % force light colours (R2025a+ follows the OS dark mode)
hold on; grid on; box on;
h1 = quiver(X, Y, 0.5*Ux, 0.5*Uy, 'AutoScale', 'off', 'Color', gray, 'LineWidth', 1);
h2 = plot([p(1) pGoal(1)], [p(2) pGoal(2)], '--', 'Color', blue, 'LineWidth', 1.4);
h3 = quiver(p(1), p(2), dg(1), dg(2), 'AutoScale', 'off', 'Color', orange, ...
            'LineWidth', 2.8, 'MaxHeadSize', 0.8);
plot(p(1), p(2), 'o', 'MarkerFaceColor', orange, 'MarkerEdgeColor', 'k', 'MarkerSize', 8);
h4 = plot(pGoal(1), pGoal(2), 'p', 'MarkerFaceColor', [0.93 0.69 0.13], ...
          'MarkerEdgeColor', 'k', 'MarkerSize', 16);
text(p(1) - 0.15, p(2), {sprintf('p = (%.1f, %.1f)', p), sprintf('d_g = [%.4f, %.4f]', dg)}, ...
     'FontSize', 11, 'HorizontalAlignment', 'right');
text(pGoal(1) - 0.05, pGoal(2) + 0.45, 'p_{goal}', 'FontSize', 11, 'HorizontalAlignment', 'center');

axis equal; xlim([-1.2 6.6]); ylim([-2.7 2.7]);
xlabel('x  (m)'); ylabel('y  (m)');
title('d_g = (p_{goal} - p) / || p_{goal} - p ||_2   (horizontal plane)');
legend([h1 h2 h3 h4], {'d_g at other drone positions (drawn at half length)', ...
       'd_{goal} = p_{goal} - p (worked example)', ...
       'd_g at the worked-example point (length = 1)', 'goal'}, ...
       'Location', 'southoutside', 'NumColumns', 1);
set(gca, 'FontSize', 11);

outDir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'assets', 'section7');
if ~exist(outDir, 'dir'), mkdir(outDir); end
exportgraphics(fig, fullfile(outDir, 'fig_7_1_goal_direction.png'), 'Resolution', 200);
