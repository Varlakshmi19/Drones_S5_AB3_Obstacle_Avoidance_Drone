% plot_7_4_membrane_response.m
% Section 7.4 - Leaky membrane ODE of a steering neuron
%     tau dx/dt = -x(t) + drive(t),     a = (tanh(x) + 1) / 2
% Response to an obstacle that appears at 0.2 s and disappears at 1.2 s
% (only the sensory part of the drive is modelled: w_in * I + bias).

tau   = 0.18;                                 % membrane time constant (s)
dt    = 1/48;                                 % control step (s)
bias  = -2.20;   wIn = 4.00;                  % constant bias and sensory gain
tEnd  = 2.0;
tOn   = 0.2;     tOff = 1.2;                  % obstacle visible between these times (I = 1)

% ---- forward-Euler neuron, exactly as discretised in the text -------------------
t      = 0:dt:tEnd;
drive  = bias + wIn * (t >= tOn & t < tOff);
xEul   = zeros(size(t));   xEul(1) = bias;
for k = 1:numel(t)-1
    xEul(k+1) = xEul(k) + (dt/tau) * (-xEul(k) + drive(k));
end

% ---- exact continuous solution (piecewise-constant drive) -----------------------
tf     = linspace(0, tEnd, 2000);
xEx    = zeros(size(tf));
brk    = [0 tOn tOff tEnd];   lvl = [bias, bias + wIn, bias];
xStart = bias;
for seg = 1:3
    m = tf >= brk(seg) & tf <= brk(seg+1);
    xEx(m) = lvl(seg) + (xStart - lvl(seg)) * exp(-(tf(m) - brk(seg)) / tau);
    xStart = lvl(seg) + (xStart - lvl(seg)) * exp(-(brk(seg+1) - brk(seg)) / tau);
end
act = @(x) (tanh(x) + 1) / 2;                 % normalised firing rate a

blue = [0.00 0.45 0.74];  orange = [0.85 0.33 0.10];  gray = [0.45 0.45 0.45];

fig = figure('Color', 'w', 'Position', [100 100 900 600]);
try, theme(fig, 'light'); catch, end   % force light colours (R2025a+ follows the OS dark mode)
tl  = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile; hold on; grid on; box on;
stairs(t, drive, '-', 'Color', gray, 'LineWidth', 1.4);
plot(tf, xEx, '-', 'Color', blue, 'LineWidth', 2.4);
plot(t, xEul, 'o', 'Color', orange, 'MarkerSize', 4, 'MarkerFaceColor', orange);
xTau = bias + (1 - exp(-1)) * wIn;           % value reached one time constant after the step
plot(tOn + tau, xTau, 'ks', 'MarkerSize', 9, 'LineWidth', 1.5);
text(tOn + tau + 0.04, xTau - 0.35, '63.2 % of the step after t = \tau = 0.18 s', 'FontSize', 10);
ylabel('membrane potential  x'); ylim([-3 2.5]);
title('\tau dx/dt = -x + drive :  step response of one steering neuron');
legend({'drive = w_{in} I + bias', 'exact solution', 'forward Euler, \Delta t = 1/48 s'}, ...
       'Location', 'east');
set(gca, 'FontSize', 11, 'XTickLabel', []);

nexttile; hold on; grid on; box on;
plot(tf, act(xEx), '-', 'Color', blue, 'LineWidth', 2.4);
plot(t, act(xEul), 'o', 'Color', orange, 'MarkerSize', 4, 'MarkerFaceColor', orange);
xlabel('time  (s)'); ylabel('firing rate  a = (tanh x + 1)/2'); ylim([-0.05 1.05]);
set(gca, 'FontSize', 11);

outDir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'assets', 'section7');
if ~exist(outDir, 'dir'), mkdir(outDir); end
exportgraphics(fig, fullfile(outDir, 'fig_7_4_membrane_response.png'), 'Resolution', 200);
