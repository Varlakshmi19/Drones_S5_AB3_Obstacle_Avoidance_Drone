% plot_7_5_hebbian_plasticity.m
% Section 7.5 - Activity-dependent Hebbian plasticity of the self-weight w_ii
%     dw/dt = mu (1 + 2F) I a - gamma (w - w0),      w clipped to [w0, w_max]
% Simplification: the neuron is fully active (a = 1) and the obstacle signal is
% I = 1 between 1 s and 6 s, I = 0 otherwise.  Three frustration levels F are compared.

mu = 0.90;  gamma = 0.15;                     % growth and relaxation rates (1/s)
w0 = 1.40;  wMax  = 3.20;                     % baseline and ceiling of the self-weight
dt = 1/48;
t  = 0:dt:20;
I  = double(t >= 1 & t < 6);                  % obstacle present
a  = ones(size(t));                           % neuron fully active
Fv = [0 0.5 1];                               % frustration levels

blue = [0.00 0.45 0.74];  orange = [0.85 0.33 0.10];  red = [0.80 0.10 0.10];
col  = [blue; orange; red];

fig = figure('Color', 'w', 'Position', [100 100 900 520]);
try, theme(fig, 'light'); catch, end   % force light colours (R2025a+ follows the OS dark mode)
hold on; grid on; box on;
fill([1 6 6 1], [1.2 1.2 3.4 3.4], [0.85 0.85 0.85], 'FaceAlpha', 0.5, 'EdgeColor', 'none');
for j = 1:numel(Fv)
    w = zeros(size(t));   w(1) = w0;
    for k = 1:numel(t)-1
        dw     = mu * (1 + 2*Fv(j)) * I(k) * a(k) - gamma * (w(k) - w0);
        w(k+1) = min(max(w(k) + dt*dw, w0), wMax);        % clip to [w0, w_max]
    end
    hp(j) = plot(t, w, '-', 'Color', col(j,:), 'LineWidth', 2.4);
    tHit  = t(find(w >= wMax - 1e-9, 1));
    if ~isempty(tHit)
        fprintf('F = %.1f : w reaches w_max = %.1f at t = %.2f s (%.2f s after the obstacle appears)\n', ...
                Fv(j), wMax, tHit, tHit - 1);
    end
end
yline(wMax, '--k', 'w_{self,max} = 3.2', 'LabelHorizontalAlignment', 'left');
yline(w0,   '--k', 'w_{self,0} = 1.4',   'LabelHorizontalAlignment', 'right');
text(3.5, 1.28, 'obstacle present (I = 1)', 'HorizontalAlignment', 'center', 'FontSize', 10);
xlabel('time  (s)'); ylabel('self-weight  w_{ii}');
xlim([0 20]); ylim([1.2 3.4]);
title('Hebbian growth while the obstacle is seen, slow decay (1/\gamma = 6.7 s) afterwards');
legend(hp, {'F = 0', 'F = 0.5', 'F = 1'}, 'Location', 'northeast');
set(gca, 'FontSize', 11);

outDir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'assets', 'section7');
if ~exist(outDir, 'dir'), mkdir(outDir); end
exportgraphics(fig, fullfile(outDir, 'fig_7_5_hebbian_plasticity.png'), 'Resolution', 200);
