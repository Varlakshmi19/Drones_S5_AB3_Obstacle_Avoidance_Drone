% run_all_section7_plots.m
% Regenerates every figure of README Section 7 into ../../assets/section7/.
% Usage (from anywhere):  run('matlab/section7/run_all_section7_plots.m')

s7_dir  = fileparts(mfilename('fullpath'));
s7_list = {'plot_7_1_goal_direction', 'plot_7_2_ray_casting', 'plot_7_2b_elliptical_bubble', ...
           'plot_7_3_side_selection', 'plot_7_4_membrane_response', 'plot_7_5_hebbian_plasticity', ...
           'plot_7_6_frustration', 'plot_7_7_speed_law', 'plot_7_8_position_control', ...
           'plot_7_9_sindy_stlsq'};
for s7_idx = 1:numel(s7_list)
    fprintf('--- %s ---\n', s7_list{s7_idx});
    run(fullfile(s7_dir, [s7_list{s7_idx} '.m']));
    close all;
end
fprintf('Done: %d figures written to %s\n', numel(s7_list), fullfile(s7_dir, '..', '..', 'assets', 'section7'));
