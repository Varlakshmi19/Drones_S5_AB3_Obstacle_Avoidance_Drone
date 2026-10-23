% plot_7_9_sindy_stlsq.m
% Section 7.9 - SINDy:  X_dot = Theta(X, U) * Xi,  solved by sequentially thresholded least squares.
% Synthetic demonstration (NOT the repository logs): data are generated from the Model A form
%     dvz/dt = -9.796 + 1.170 T cos(roll) cos(pitch)  + noise
% and STLSQ has to rediscover it from a nine-term candidate library.

rng(1);
M     = 3000;                                 % number of snapshots
T     = 6 + 5*rand(M, 1);                     % thrust feature
roll  = 0.5*(2*rand(M, 1) - 1);               % rad
pitch = 0.5*(2*rand(M, 1) - 1);               % rad
vz    = 0.5*randn(M, 1);                      % m/s
dvz   = -9.796 + 1.170*T.*cos(roll).*cos(pitch) + 0.10*randn(M, 1);   % "measured" derivative

Theta = [ones(M,1), T.*cos(roll).*cos(pitch), T.*sin(pitch), T.*sin(roll), ...
         vz, roll, pitch, T, vz.^2];          % candidate library (M x p)
names = {'1', 'T cos\phi cos\theta', 'T sin\theta', 'T sin\phi', 'v_z', '\phi', '\theta', 'T', 'v_z^2'};
p     = size(Theta, 2);
truth = [-9.796; 1.170; 0; 0; 0; 0; 0; 0; 0];

alpha  = 1e-6;                                % ridge penalty
lambda = 0.05;                                % keep |xi| >= lambda * max|xi|
active = true(p, 1);
xi     = zeros(p, 1);
for it = 1:10                                 % STLSQ loop
    A = Theta(:, active);
    xi = zeros(p, 1);
    xi(active) = (A'*A + alpha*eye(nnz(active))) \ (A'*dvz);        % ridge on active columns
    if it == 1, xiRidge = xi; end
    keep = active & (abs(xi) >= lambda * max(abs(xi)));              % drop the small ones
    if isequal(keep, active), break; end
    active = keep;
end
pred = Theta * xi;
R2   = 1 - sum((dvz - pred).^2) / sum((dvz - mean(dvz)).^2);
fprintf('STLSQ finished after %d iterations, %d active terms, R^2 = %.5f\n', it, nnz(active), R2);
for q = find(active)'
    fprintf('  recovered term %-22s xi = %9.4f\n', names{q}, xi(q));
end

blue = [0.00 0.45 0.74];  orange = [0.85 0.33 0.10];  gray = [0.65 0.65 0.65];

fig = figure('Color', 'w', 'Position', [100 100 1100 520]);
try, theme(fig, 'light'); catch, end   % force light colours (R2025a+ follows the OS dark mode)
tl  = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile; hold on; grid on; box on;
Y = abs([truth, xiRidge, xi]);   Y(Y == 0) = NaN;       % exact zeros cannot be drawn on a log axis
hb = bar(1:p, Y, 'grouped');
set(hb, {'FaceColor'}, {[0.20 0.20 0.20]; gray; blue}, 'BaseValue', 1e-4);
set(gca, 'YScale', 'log', 'XTick', 1:p, 'XTickLabel', names, 'FontSize', 11); xtickangle(35);
ylim([1e-4 30]);
ylabel('|\xi_j|   (log scale)');
legend({'true model', 'first ridge fit (all terms)', 'after STLSQ'}, 'Location', 'northeast');
title('(a) Coefficient magnitudes for dv_z/dt  (no bar = exactly 0)');

nexttile; hold on; grid on; box on;
plot(dvz, pred, '.', 'Color', blue, 'MarkerSize', 6);
lim = [min(dvz) max(dvz)];
plot(lim, lim, 'k--', 'LineWidth', 1.6);
axis equal; xlim(lim); ylim(lim);
xlabel('"measured"  dv_z/dt  (m/s^2)'); ylabel('predicted  \Theta \xi  (m/s^2)');
text(lim(1) + 0.05*diff(lim), lim(2) - 0.08*diff(lim), sprintf('R^2 = %.5f', R2), ...
     'FontSize', 12, 'FontWeight', 'bold');
title('(b) Predicted versus actual acceleration');
set(gca, 'FontSize', 11);

outDir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'assets', 'section7');
if ~exist(outDir, 'dir'), mkdir(outDir); end
exportgraphics(fig, fullfile(outDir, 'fig_7_9_sindy_stlsq.png'), 'Resolution', 200);
