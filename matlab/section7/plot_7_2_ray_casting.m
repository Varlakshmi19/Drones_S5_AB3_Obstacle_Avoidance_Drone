% plot_7_2_ray_casting.m
% Section 7.2 - Ray parametrisation and distance extraction
%     r_i(s) = r_start,i + s (r_end,i - r_start,i),   s in [0, 1]
%     d_i    = R_sensor            if the ray hits nothing
%            = f_i * R_sensor      if it hits at fraction f_i
% Top-down view of the 6-beam FSM fan meeting one rectangular obstacle.

p      = [0 0];                              % drone position (x, y); heading = +x
angles = [0 35 -35 70 -70 180];              % FSM beam bearings (deg), relative to heading
dOff   = 0.18;                               % ray start offset d_offset (m)
Rs     = 1.60;                               % maximum sensor range R_sensor (m)
obs    = [1.0 1.4 -0.35 0.85];               % obstacle [xmin xmax ymin ymax] (m)

u      = [cosd(angles(:)), sind(angles(:))]; % unit direction of every beam
rStart = p + dOff * u;                       % r_start,i
rEnd   = rStart + Rs * u;                    % r_end,i
n      = numel(angles);
f      = ones(n, 1);                         % hit fraction (1 = no hit)
hit    = false(n, 1);
lo     = obs([1 3]);  hi = obs([2 4]);

for k = 1:n                                  % slab test of each ray against the box
    D = rEnd(k,:) - rStart(k,:);
    tNear = 0;  tFar = 1;  ok = true;
    for a = 1:2
        if abs(D(a)) < 1e-12
            if rStart(k,a) < lo(a) || rStart(k,a) > hi(a), ok = false; end
        else
            t1 = (lo(a) - rStart(k,a)) / D(a);
            t2 = (hi(a) - rStart(k,a)) / D(a);
            tNear = max(tNear, min(t1, t2));
            tFar  = min(tFar,  max(t1, t2));
        end
    end
    if ok && tNear <= tFar, f(k) = tNear; hit(k) = true; end
end
d = f * Rs;                                  % measured distances d_i
fprintf('Beam %+4d deg : d = %.2f m\n', [angles(:), d]');

blue = [0.00 0.45 0.74];  orange = [0.85 0.33 0.10];  green = [0.13 0.55 0.13];

fig = figure('Color', 'w', 'Position', [100 100 900 560]);
try, theme(fig, 'light'); catch, end   % force light colours (R2025a+ follows the OS dark mode)
hold on; grid on; box on;
fill(obs([1 2 2 1]), obs([3 3 4 4]), [0.78 0.78 0.78], 'EdgeColor', 'k', 'LineWidth', 1.5);
text(1.2, 0.97, 'obstacle', 'HorizontalAlignment', 'center', 'FontSize', 11);
th = linspace(0, 2*pi, 200);
plot(dOff*cos(th), dOff*sin(th), '--', 'Color', [0.5 0.5 0.5]);        % start-offset ring

for k = 1:n
    pHit = rStart(k,:) + f(k) * (rEnd(k,:) - rStart(k,:));
    if hit(k), c = orange; else, c = green; end
    plot([rStart(k,1) pHit(1)], [rStart(k,2) pHit(2)], '-', 'Color', c, 'LineWidth', 2.2);
    if hit(k)
        plot([pHit(1) rEnd(k,1)], [pHit(2) rEnd(k,2)], ':', 'Color', c, 'LineWidth', 1.2);
        plot(pHit(1), pHit(2), 'o', 'MarkerFaceColor', c, 'MarkerEdgeColor', 'k', 'MarkerSize', 7);
    end
    plot(rStart(k,1), rStart(k,2), '.', 'Color', 'k', 'MarkerSize', 12);
    if hit(k)                                              % label to the right of the obstacle
        lab = [obs(2) + 0.10, pHit(2) + 0.10];  al = 'left';
    elseif abs(u(k,2)) < 1e-9                              % rear beam: label above its middle
        lab = (rStart(k,:) + rEnd(k,:)) / 2 + [0 0.12];  al = 'center';
    else                                                   % label beyond the ray tip
        lab = rEnd(k,:) + 0.08 * sign(u(k,:));  al = 'left';
    end
    text(lab(1), lab(2), sprintf('%d^{\\circ}: d = %.2f m', angles(k), d(k)), 'FontSize', 10, ...
         'Color', c * 0.8, 'FontWeight', 'bold', ...
         'HorizontalAlignment', al, 'VerticalAlignment', 'middle');
end

plot(0, 0, 's', 'MarkerFaceColor', blue, 'MarkerEdgeColor', 'k', 'MarkerSize', 10);
quiver(0, 0, 0.35, 0, 'AutoScale', 'off', 'Color', blue, 'LineWidth', 2, 'MaxHeadSize', 1);
text(-0.25, -0.22, 'drone', 'HorizontalAlignment', 'right', 'FontSize', 10, 'Color', blue);
text(-0.15, 0.30, 'd_{offset}', 'HorizontalAlignment', 'right', 'FontSize', 10, 'Color', [0.4 0.4 0.4]);

hh(1) = plot(NaN, NaN, '-', 'Color', orange, 'LineWidth', 2.2);
hh(2) = plot(NaN, NaN, '-', 'Color', green,  'LineWidth', 2.2);
hh(3) = plot(NaN, NaN, 'o', 'MarkerFaceColor', orange, 'MarkerEdgeColor', 'k');
legend(hh, {'ray with a hit:  d_i = f_i R_{sensor} < R_{sensor}', ...
            'ray with no hit:  d_i = R_{sensor}', 'hit point  (s = f_i)'}, 'Location', 'southwest');
axis equal; xlim([-2.0 3.0]); ylim([-2.0 2.0]);
xlabel('x  (m)'); ylabel('y  (m)');
title('Ray-casting fan:  r_i(s) = r_{start,i} + s (r_{end,i} - r_{start,i}),  s \in [0, 1]');
set(gca, 'FontSize', 11);

outDir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'assets', 'section7');
if ~exist(outDir, 'dir'), mkdir(outDir); end
exportgraphics(fig, fullfile(outDir, 'fig_7_2_ray_casting.png'), 'Resolution', 200);
