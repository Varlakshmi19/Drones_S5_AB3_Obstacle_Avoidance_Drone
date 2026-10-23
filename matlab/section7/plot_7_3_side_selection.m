% plot_7_3_side_selection.m
% Section 7.3 - Side selection from the free-space scores, with hysteresis
%     S_left  = 0.65 d_left  + 0.35 d_far-left      S_right = 0.65 d_right + 0.35 d_far-right
%     Side*   = previous side            if |S_left - S_right| < 0.08 m and a previous side exists
%             = LEFT                     if S_left >= S_right
%             = RIGHT                    otherwise
% (a) the decision rule as a function of dS = S_left - S_right
% (b) the same rule fed with a noisy dS: it flips far less often than a plain "larger score wins".

delta = 0.08;                                 % hysteresis dead-band (m)
LEFT  = +1;   RIGHT = -1;
side  = @(dS, prev) ((abs(dS) < delta) & (prev ~= 0)) .* prev + ...
                    ~((abs(dS) < delta) & (prev ~= 0)) .* (2*(dS >= 0) - 1);

% ---- (a) decision rule ---------------------------------------------------------
dS = linspace(-0.25, 0.25, 1001);
fromLeft  = side(dS, LEFT);                   % previous side = LEFT
fromRight = side(dS, RIGHT);                  % previous side = RIGHT

% ---- (b) noisy example, 10 s at 48 Hz -------------------------------------------
rng(7);
t     = (0:1/48:10)';
dSsig = 0.03*sin(2*pi*0.4*t) + 0.025*randn(size(t));   % S_left - S_right measured with noise
plain = 2*(dSsig >= 0) - 1;                   % no hysteresis: larger score wins
held  = zeros(size(t));
prev  = 0;                                    % no side chosen yet
for k = 1:numel(t)
    held(k) = side(dSsig(k), prev);
    prev    = held(k);
end
nPlain = nnz(diff(plain) ~= 0);
nHeld  = nnz(diff(held)  ~= 0);
fprintf('Side switches in 10 s:  without hysteresis = %d,  with hysteresis = %d\n', nPlain, nHeld);

blue = [0.00 0.45 0.74];  orange = [0.85 0.33 0.10];  red = [0.80 0.10 0.10];  gray = [0.55 0.55 0.55];

fig = figure('Color', 'w', 'Position', [100 100 1100 560]);
try, theme(fig, 'light'); catch, end   % force light colours (R2025a+ follows the OS dark mode)
tl  = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile(1, [2 1]); hold on; box on; grid on;
fill([-delta delta delta -delta], [-1.3 -1.3 1.3 1.3], gray, 'FaceAlpha', 0.20, 'EdgeColor', 'none');
h1 = plot(dS, fromLeft  + 0.04, '-',  'Color', blue,   'LineWidth', 2.6);
h2 = plot(dS, fromRight - 0.04, '--', 'Color', orange, 'LineWidth', 2.6);
text(0, 0.55, {'dead-band', '|dS| < 0.08 m', 'keep previous side'}, 'HorizontalAlignment', 'center', 'FontSize', 10);
yticks([-1 1]); yticklabels({'RIGHT', 'LEFT'}); ylim([-1.3 1.3]); xlim([-0.25 0.25]);
xlabel('dS = S_{left} - S_{right}  (m)'); ylabel('chosen side  Side^*');
title('(a) Decision rule');
legend([h1 h2], {'previous side = LEFT', 'previous side = RIGHT'}, 'Location', 'southeast');
set(gca, 'FontSize', 11);

nexttile(2); hold on; box on; grid on;
plot(t, dSsig, '-', 'Color', [0.35 0.35 0.35], 'LineWidth', 1);
yline( delta, '--', 'Color', red); yline(-delta, '--', 'Color', red); yline(0, '-', 'Color', gray);
ylabel('dS  (m)'); title('(b) Noisy score difference and the resulting decisions');
set(gca, 'FontSize', 11, 'XTickLabel', []);

nexttile(4); hold on; box on; grid on;
h3 = plot(t, plain + 0.06, '-', 'Color', red,  'LineWidth', 1);
h4 = plot(t, held  - 0.06, '-', 'Color', blue, 'LineWidth', 2.2);
yticks([-1 1]); yticklabels({'RIGHT', 'LEFT'}); ylim([-1.4 1.4]);
xlabel('time  (s)'); ylabel('chosen side');
legend([h3 h4], {sprintf('no hysteresis (%d switches)', nPlain), ...
                 sprintf('with 0.08 m dead-band (%d switches)', nHeld)}, 'Location', 'east');
set(gca, 'FontSize', 11);

outDir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'assets', 'section7');
if ~exist(outDir, 'dir'), mkdir(outDir); end
exportgraphics(fig, fullfile(outDir, 'fig_7_3_side_selection.png'), 'Resolution', 200);
