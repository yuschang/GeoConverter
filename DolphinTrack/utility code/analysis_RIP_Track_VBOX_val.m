clc
clear all

cd('G:\[Drone_recreational_ship_RNL_project]\analysis')

topFolder = 'G:\[Drone_recreational_ship_RNL_project]\ship_tracking_validation\rip_tracking_validation_backup\DJI_202606041653_001';
trackDataList = load_all_track_coordinates(topFolder);

%%

clear global vbo;
global vbo;


% VBOX code download
% https://www.vboxautomotive.co.uk/index.php/en/customer-area/downloads#analysis
F_vboload; % manually define the vbo file

% vbograph - demo graphing a loaded VBO file, time vs velocity

xvar = 'time';
yvar = 'velocity kmh';

global vbo;
if (0 == size(vbo,1))
	error 'Load a VBO file first.';
end


%% **** analysis and plto figure 


kmh2kt = 0.539956803;   % 1 km/h = 0.539956803 kt

delay = seconds(30);
startEndBuffer = 100;   % unit time points

% Drone data index
droneDataidx = [1 2 3 4];

% Define altitude groups
idx_100m  = [1 2 3 4 ];

% ---------------------------------------------------------
% Time axis: convert to elapsed minutes from VBOX start time
% ---------------------------------------------------------
t0 = new_time_vbox(1);

x_vbox_min = minutes(new_time_vbox - t0);
x_vbox_min_col = x_vbox_min(:);

vbox_kmh = vbo.channels(ychan).data;
vbox_kmh_col = vbox_kmh(:);

% ---------------------------------------------------------
% Storage for error analysis
% ---------------------------------------------------------

vbox_100m_all  = {};
drone_100m_all = {};
error_100m_all = {};


% ---------------------------------------------------------
% Figure 1: Time-series comparison
% ---------------------------------------------------------
figure('Position', [100 100 650 500]); clf
hold on

yyaxis left

hVBOX = plot(x_vbox_min_col, vbox_kmh_col, ...
    'Color', [0.65 0.65 0.65], ...
    'LineWidth', 1.2, ...
    'Marker', 'none'); 
hold on

hDrone = gobjects(length(droneDataidx), 1);

% Different colors for each drone data
colorList = lines(length(droneDataidx));
idx_cnt_50 = 0;
idx_cnt_100 = 0;


for n = 1:length(droneDataidx)

    i = droneDataidx(n);

    timeraw = trackDataList(i).timestamp;     % datetime

    % Remove NaTs from time first
    valid_time_mask = ~isnat(timeraw);

    timeVec = timeraw(valid_time_mask);
    lat = trackDataList(i).latitude(valid_time_mask);
    lon = trackDataList(i).longitude(valid_time_mask);

    % Calculate Kalman velocity
    v_kalman_kmh = calc_velocity_kalman(lat, lon, timeVec);

    timeVec.TimeZone = 'Asia/Seoul';

    newTimeAxis = timeVec' - delay;

    x_drone_min = minutes(newTimeAxis - t0);

    % Force column vectors
    x_drone_min_col = x_drone_min(:);
    v_drone_kmh_col = v_kalman_kmh(:);

    % Apply start buffer safely
    useIdx = false(size(v_drone_kmh_col));
    useIdx(startEndBuffer:end) = true;

    % -----------------------------------------------------
    % Plot drone-derived velocity
    % -----------------------------------------------------
    % droneColor = colorList(n, :);

    hDrone(n) = plot(x_drone_min_col(useIdx), ...
        v_drone_kmh_col(useIdx), ...
        'Color', [0.2 0.4 0.8], ...
        'LineWidth', 1.8, ...
        'LineStyle', '-', ...
        'Marker', 'none'); 
    hold on

    % -----------------------------------------------------
    % Interpolate VBOX velocity onto drone time axis
    % -----------------------------------------------------
    vbox_interp_kmh = interp1(x_vbox_min_col, ...
                              vbox_kmh_col, ...
                              x_drone_min_col, ...
                              'linear', ...
                              NaN);

    % -----------------------------------------------------
    % Valid comparison points
    % -----------------------------------------------------
    valid = ~isnan(vbox_interp_kmh) & ...
            ~isnan(v_drone_kmh_col) & ...
            useIdx & ...
            x_drone_min_col >= min(x_vbox_min_col) & ...
            x_drone_min_col <= max(x_vbox_min_col);

    % Optional: remove very low-speed points
    % This prevents inflated percentage error near zero speed.
    valid = valid & vbox_interp_kmh > 3;

    % -----------------------------------------------------
    % Calculate point-wise error
    % Error = Drone - VBOX
    % -----------------------------------------------------
    vbox_valid  = vbox_interp_kmh(valid)*kmh2kt;
    drone_valid = v_drone_kmh_col(valid)*kmh2kt;

    error_valid = abs(drone_valid - vbox_valid);

    if ismember(i, idx_100m)
        idx_cnt_100 = idx_cnt_100 +1;

        vbox_100m_all{idx_cnt_100}  = vbox_valid;
        drone_100m_all{idx_cnt_100} = drone_valid;
        error_100m_all{idx_cnt_100} = error_valid;

    end

end

yticks(5:10:40)

xlabel('Time (min)')
% ylabel('Velocity')

ylim([0 40])
xlim([0 8])

ax = gca;
ax.YAxis(1).Color = [0 0 0];

% ---------------------------------------------------------
% Right y-axis: same velocity converted to kt
% ---------------------------------------------------------
yyaxis right

ylimLeft_kmh = [0 40];
ylim(ylimLeft_kmh * kmh2kt)

yticks(0:5:25)
% ylabel('Velocity (kt)')

ax = gca;
ax.YAxis(2).Color = [1 0 0];

yyaxis left

% legend([hVBOX, hDrone(:)'], ...
%     [{'VBOX Sport'}, ...
%      arrayfun(@(x) sprintf('Drone %d', x), droneDataidx, 'UniformOutput', false)], ...
%     'Location', 'southeast', ...
%     'Box', 'off')

box off
set(gca, 'FontSize', 20)


% ---------------------------------------------------------
% Speed-binned error analysis for maritime target
% Bin edges in knots
% ---------------------------------------------------------

bin_edges = [3 6 9 12 15 18];  % kt
bin_labels = {'3–6', '6–9', '9–12', '12–15', '15–18'};
n_bins = length(bin_edges) - 1;

% Concatenate all valid data across runs
% 100 m altitude (excluding pan maneuver section)
vbox_100m_cat  = [];
error_100m_cat = [];

for i = 1:length(vbox_100m_all)
    if i == 1
        % Exclude pan maneuver indices 143:284
        keep_idx = [1:143, 284:length(vbox_100m_all{i})];
        vbox_100m_cat  = [vbox_100m_cat;  vbox_100m_all{i}(keep_idx)];
        error_100m_cat = [error_100m_cat; error_100m_all{i}(keep_idx)];
    else
        vbox_100m_cat  = [vbox_100m_cat;  vbox_100m_all{i}];
        error_100m_cat = [error_100m_cat; error_100m_all{i}];
    end
end


% ---------------------------------------------------------
% Compute RMSE, mean absolute error, SD per bin
% ---------------------------------------------------------

rmse_100m = nan(n_bins, 1);
mae_100m  = nan(n_bins, 1);
std_100m  = nan(n_bins, 1);
n_100m    = nan(n_bins, 1);

for b = 1:n_bins
    % 100 m
    mask_100 = vbox_100m_cat >= bin_edges(b) & ...
               vbox_100m_cat <  bin_edges(b+1);
    e100 = error_100m_cat(mask_100);
    if ~isempty(e100)
        rmse_100m(b) = sqrt(mean(e100.^2));
        mae_100m(b)  = mean(e100);
        std_100m(b)  = std(e100);
        n_100m(b)    = numel(e100);
    end

 
end

% ---------------------------------------------------------
% Display results as table
% ---------------------------------------------------------

fprintf('\n--- 100 m altitude ---\n')
fprintf('%-10s %6s %6s %6s %6s\n', 'Bin (kt)', 'RMSE', 'MAE', 'SD', 'N')
for b = 1:n_bins
    fprintf('%-10s %6.3f %6.3f %6.3f %6d\n', ...
        bin_labels{b}, rmse_100m(b), mae_100m(b), std_100m(b), n_100m(b))
end


% ---------------------------------------------------------
% Bar plot: RMSE by speed bin and altitude
% ---------------------------------------------------------

figure('Position', [100 100 600 500]); clf
x = 1:n_bins;
bar_width = 0.35;

bar(x - bar_width/2, rmse_100m, bar_width, 'FaceColor', [0.2 0.4 0.8], ...
    'DisplayName', '100 m'); hold on


set(gca, 'XTick', x, 'XTickLabel', bin_labels)
xlabel('Vessel velocity (kt)')
ylabel('RMSE (kt)')
ylim([0 1])
box off
set(gca, 'FontSize', 20)
