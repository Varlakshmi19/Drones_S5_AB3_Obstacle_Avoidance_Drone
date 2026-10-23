% plot_7_6_frustration.m
% Section 7.6 - Frustration leaky integrator and the goal-detour angle
%     dF/dt = +alpha_rise    if t_noprogress > 1 s and t_open < 1 s
%           = -alpha_decay   if t_open >= 1 s
%           =  0             otherwise                 (F clipped to [0, 1])
%     theta_detour = sigma * 80 deg * F
% Scenario: the drone makes no progress for 8 s (front blocked), then the way opens.

aRise  = 0.35;   aDecay = 0.50;               % 1/s
thMax  = 80;                                  % maximum detour angle (deg)
sigma  = +1;                                  % committed side (+1 = left)
dt     = 1/48;
t      = 0:dt:14;
tOpen0 = 8;                                   % the front becomes free at this time (s)

tNoProg = t .* (t < tOpen0) + (t - tOpen0) .* (t >= tOpen0);   % time since last progress
tOpen   = max(t - tOpen0, 0);                                  % time the front has been free

F = zeros(size(t));
for k = 1:numel(t)-1
    if tNoProg(k) > 1 && tOpen(k) < 1
        dF = +aRise;
    elseif tOpen(k) >= 1
        dF = -aDecay;
    else
        dF = 0;
    end
    F(k+1) = min(max(F(k) + dt*dF, 0), 1);
end
thetaDetour = sigma * thMax * F;              % detour angle (deg)
fprintf('F reaches 1 at t = %.2f s (analytic: 1 + 1/%.2f = %.2f s)\n', t(find(F >= 1 - 1e-9, 1)), aRise, 1 + 1/aRise);

blue = [0.00 0.45 0.74];  orange = [0.85 0.33 0.10];

fig = figure('Color', 'w', 'Position', [100 100 900 520]);
try, theme(fig, 'light'); catch, end   % force light colours (R2025a+ follows the OS dark mode)
hold on; grid on; box on;
fill([0 tOpen0 tOpen0 0], [-0.05 -0.05 1.1 1.1], [0.90 0.90 0.90], 'EdgeColor', 'none');
fill([tOpen0 14 14 tOpen0], [-0.05 -0.05 1.1 1.1], [0.88 0.95 0.88], 'EdgeColor', 'none');
yyaxis left
plot(t, F, '-', 'Color', blue, 'LineWidth', 2.8);
ylabel('frustration  F'); ylim([-0.05 1.1]); set(gca, 'YColor', blue);
yyaxis right
plot(t, thetaDetour, '--', 'Color', orange, 'LineWidth', 2);
ylabel('detour angle  \theta_{detour}  (deg)'); ylim([-0.05 1.1] * thMax); set(gca, 'YColor', orange);
text(4, 0.12*thMax, 'stalled: no progress, front blocked', 'HorizontalAlignment', 'center', 'FontSize', 10);
text(12.5, 0.12*thMax, 'front free', 'HorizontalAlignment', 'center', 'FontSize', 10);
text(2.4, 0.62*thMax, {'rise 0.35 s^{-1}', 'once stalled', 'for more than 1 s'}, 'HorizontalAlignment', 'right', 'FontSize', 10);
text(10.2, 0.62*thMax, {'decay 0.50 s^{-1}', 'once the front has', 'been free for 1 s'}, 'HorizontalAlignment', 'left', 'FontSize', 10);
xlabel('time  (s)'); xlim([0 14]);
title('Frustration F(t) and the resulting goal-detour angle');
set(gca, 'FontSize', 11);

outDir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'assets', 'section7');
if ~exist(outDir, 'dir'), mkdir(outDir); end
exportgraphics(fig, fullfile(outDir, 'fig_7_6_frustration.png'), 'Resolution', 200);
