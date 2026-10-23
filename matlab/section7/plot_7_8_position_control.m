% plot_7_8_position_control.m
% Section 7.8 - Cascaded position control law (one horizontal axis)
%     a_des = Kp e_p + Kd e_v + Ki * integral(e_p dt)          (+ g z_W on the vertical axis only)
% The drone is modelled as a unit-mass double integrator, v' = a_des + a_dist, with a constant
% disturbance (e.g. wind).  The gains are ILLUSTRATIVE - they are not the tuned DSLPID gains.

Kp = 6;  Kd = 4;  Ki = 2;                     % illustrative per-axis gains
aDist = -0.5;                                 % constant disturbance acceleration (m/s^2)
dt = 1/48;
t  = 0:dt:12;
xT = double(t >= 0.5);                        % target position: 1 m step at t = 0.5 s

blue = [0.00 0.45 0.74];  orange = [0.85 0.33 0.10];  green = [0.13 0.55 0.13];  red = [0.80 0.10 0.10];

KiSet = [0 Ki];                               % PD only, then full PID
X = zeros(numel(KiSet), numel(t));
for c = 1:numel(KiSet)
    x = 0;  v = 0;  eInt = 0;
    for k = 1:numel(t)
        eP   = xT(k) - x;                     % position error
        eV   = 0 - v;                         % velocity error (target velocity = 0)
        eInt = eInt + eP*dt;                  % integral of the position error
        aP = Kp*eP;   aD = Kd*eV;   aI = KiSet(c)*eInt;
        aDes = aP + aD + aI;                  % control law
        X(c,k) = x;
        if c == 2, comp(:,k) = [aP; aD; aI]; end   %#ok<SAGROW>
        v = v + (aDes + aDist)*dt;            % semi-implicit Euler step
        x = x + v*dt;
    end
end
fprintf('Final position:  PD = %.3f m,  PID = %.3f m  (target 1 m)\n', X(1,end), X(2,end));

fig = figure('Color', 'w', 'Position', [100 100 900 640]);
try, theme(fig, 'light'); catch, end   % force light colours (R2025a+ follows the OS dark mode)
tl  = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile; hold on; grid on; box on;
stairs(t, xT, '-', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.4);
plot(t, X(1,:), '--', 'Color', red,  'LineWidth', 2.4);
plot(t, X(2,:), '-',  'Color', blue, 'LineWidth', 2.8);
ylabel('position  x  (m)'); ylim([-0.1 1.3]);
title('a_{des} = K_p e_p + K_d e_v + K_i \int e_p dt   (unit mass, constant disturbance -0.5 m/s^2)');
legend({'target', 'P + D only: steady-state offset', 'P + I + D: offset removed'}, 'Location', 'southeast');
set(gca, 'FontSize', 11, 'XTickLabel', []);

nexttile; hold on; grid on; box on;
plot(t, comp(1,:), '-', 'Color', blue,   'LineWidth', 2);
plot(t, comp(2,:), '-', 'Color', orange, 'LineWidth', 2);
plot(t, comp(3,:), '-', 'Color', green,  'LineWidth', 2);
plot(t, sum(comp,1), 'k--', 'LineWidth', 1.4);
xlabel('time  (s)'); ylabel('acceleration command  (m/s^2)');
legend({'K_p e_p', 'K_d e_v', 'K_i \int e_p dt', 'a_{des} (sum)'}, 'Location', 'northeast');
set(gca, 'FontSize', 11);

outDir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'assets', 'section7');
if ~exist(outDir, 'dir'), mkdir(outDir); end
exportgraphics(fig, fullfile(outDir, 'fig_7_8_position_control.png'), 'Resolution', 200);
