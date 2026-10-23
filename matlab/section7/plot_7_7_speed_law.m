% plot_7_7_speed_law.m
% Section 7.7 - Speed neuron: forward speed as a function of the frontal clearance
%     s = clip( (d_front - d_stop) / (d_slow - d_stop), -1, 1 )
%     v = V_max s (1 - 0.5 |omega|/Omega_max)          if s >= 0
%       = V_rev s                                      if s <  0 and d_rear > 0.35 m
%       = 0                                            if s <  0 and d_rear <= 0.35 m

Vmax = 0.40;   Vrev = 0.18;                   % cruise / reverse speed (m/s)
dStop = 0.28;  dSlow = 0.85;                  % braking window (m)
OmgMax = 2.60;                                % maximum yaw rate (rad/s)
dRearMin = 0.35;                              % minimum rear clearance for backing out (m)

dFront = linspace(0, 1.2, 601);
s      = min(max((dFront - dStop) / (dSlow - dStop), -1), 1);
speed  = @(omega, dRear) (s >= 0) .* Vmax .* s .* (1 - 0.5*abs(omega)/OmgMax) ...
                       + ((s < 0) & (dRear > dRearMin)) .* Vrev .* s;

vStraight = speed(0,      1.0);               % no turning, rear clear
vTurning  = speed(OmgMax, 1.0);               % turning at the maximum yaw rate, rear clear
vBlocked  = speed(0,      0.2);               % rear blocked: cannot back out
vBlocked(s >= 0) = NaN;                       % show only the part that differs

blue = [0.00 0.45 0.74];  orange = [0.85 0.33 0.10];  red = [0.80 0.10 0.10];  gray = [0.45 0.45 0.45];

fig = figure('Color', 'w', 'Position', [100 100 900 620]);
try, theme(fig, 'light'); catch, end   % force light colours (R2025a+ follows the OS dark mode)
hold on; grid on; box on;
fill([dStop dSlow dSlow dStop], [-0.12 -0.12 0.46 0.46], [0.93 0.93 0.93], 'EdgeColor', 'none');
h2 = plot(dFront, vTurning,  '-',  'Color', orange, 'LineWidth', 2.8);
h1 = plot(dFront, vStraight, '-',  'Color', blue,   'LineWidth', 2.8);   % drawn last: reverse branch is identical
h3 = plot(dFront, vBlocked,  '--', 'Color', red,    'LineWidth', 2.8);
xline(dStop, ':k', 'd_{stop} = 0.28 m', 'LabelVerticalAlignment', 'bottom');
xline(dSlow, ':k', 'd_{slow} = 0.85 m', 'LabelVerticalAlignment', 'bottom');
yline(0, '-', 'Color', gray);
text((dStop + dSlow)/2, 0.43, 'braking ramp', 'HorizontalAlignment', 'center', 'FontSize', 10);
text(0.14, -0.105, 'reverse', 'HorizontalAlignment', 'center', 'FontSize', 10);
text(1.02, 0.425, 'V_{max} = 0.40 m/s', 'HorizontalAlignment', 'center', 'FontSize', 10, 'Color', blue);
text(1.02, 0.225, '0.5 V_{max} when \omega = \Omega_{max}', 'HorizontalAlignment', 'center', 'FontSize', 10, 'Color', orange);
xlabel('frontal clearance  d_{front}  (m)'); ylabel('commanded forward speed  v  (m/s)');
xlim([0 1.2]); ylim([-0.12 0.46]);
title('Speed neuron:  v(d_{front})');
legend([h1 h2 h3], {'straight flight (\omega = 0), rear clear', ...
                    'turning at \omega = \Omega_{max}, rear clear', ...
                    'rear blocked (d_{rear} \leq 0.35 m): no reversing'}, 'Location', 'southoutside');
set(gca, 'FontSize', 11);

outDir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'assets', 'section7');
if ~exist(outDir, 'dir'), mkdir(outDir); end
exportgraphics(fig, fullfile(outDir, 'fig_7_7_speed_law.png'), 'Resolution', 200);
