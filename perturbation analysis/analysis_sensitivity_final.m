%% =========================================================
%  Sensitivity Analysis v4
%  Factors: (1) Source Level RMSE, (2) Speed Estimation Error
%  Propagation model fixed at 15×log10(R) throughout
% ==========================================================
clc; clear all; close all;

%% --- 1. COEFFICIENTS & RMSE ---
sl_coeffs.SB.k    = [0.20204,  0.17298,  0.28864];
sl_coeffs.SB.b    = [145.6487, 143.7666, 137.4133];
sl_coeffs.SB.rmse = [2.0368,   2.7178,   2.1808];

sl_coeffs.MB.k    = [0.29395,  0.2447,   0.16163];
sl_coeffs.MB.b    = [138.9954, 139.0338, 137.7818];
sl_coeffs.MB.rmse = [2.07,     1.6972,   2.0163];

sl_coeffs.LB.k    = [1.5176,   1.5429,   1.7665];
sl_coeffs.LB.b    = [135.1601, 134.6946, 131.677];
sl_coeffs.LB.rmse = [1.6519,   1.4934,   2.2875];

sl_coeffs.FB.k    = [1.0961,   1.1623,   1.49];
sl_coeffs.FB.b    = [131.3139, 130.9884, 125.6486];
sl_coeffs.FB.rmse = [1.8701,   1.7952,   0.68507];

%% --- 2. SPEED ESTIMATION RMSE BY BIN (100 m altitude) ---
speed_bin_edges = [3, 6, 9, 12, 15, 18];
speed_bin_rmse  = [0.236, 0.390, 0.470, 0.798, 0.618];

%% --- 3. BACKGROUND LEVELS ---
TOL_background = [91.7364, 90.8241, 91.7343, 90.6131, 89.9198, 91.2759, 92.4359, ...
                  95.0217, 97.9105, 101.2938, 103.2478, 104.5816, 105.7295, 107.4595, ...
                  108.5833, 109.6276, 109.3522, 107.6380, 105.6315, 102.7318, 98.3414];

BL_5k = TOL_background(14);  % adjust index if needed after baseline verification

%% --- 4. FIXED PROPAGATION MODEL ---
prop_n = 15;  % 15×log10(R) fixed throughout

%% --- 5. LOAD DATA ---
cd('G:\[Drone_recreational_ship_RNL_project]\analysis')
topFolder = 'G:\[Drone_recreational_ship_RNL_project]\watching_boat_passing_hydrophone\ship_tracking_vid';
trackDataList = load_all_track_coordinates(topFolder);

vid_unique = unique({trackDataList.filename});
dolphin_boat_dist = {};
cnt = 0;

for vid_IDX = 1:length(vid_unique)
    vid_sameIdx = [];
    for n = 1:length(trackDataList)
        if strcmp(vid_unique{vid_IDX}, trackDataList(n).filename)
            vid_sameIdx = [vid_sameIdx n];
        end
    end
    if ~isempty(vid_sameIdx)
        dolphin_vid_idx = [];
        for m = 1:length(vid_sameIdx)
            if strcmp(trackDataList(vid_sameIdx(m)).calf_status, 'Dolphin Pod')
                dolphin_vid_idx = [dolphin_vid_idx vid_sameIdx(m)];
            end
        end
        if ~isempty(dolphin_vid_idx)
            allBoatIdx = setdiff(vid_sameIdx, dolphin_vid_idx);
            if ~isempty(allBoatIdx)
                for tkn = 1:length(dolphin_vid_idx)
                    dolphinTrack_timeStamp = trackDataList(dolphin_vid_idx(tkn)).timestamp;
                    dolphinTrack_latitude  = trackDataList(dolphin_vid_idx(tkn)).latitude;
                    dolphinTrack_longitude = trackDataList(dolphin_vid_idx(tkn)).longitude;
                    for btn = 1:length(allBoatIdx)
                        boat_timestamp = trackDataList(allBoatIdx(btn)).timestamp;
                        boat_lat  = trackDataList(allBoatIdx(btn)).latitude;
                        boat_lon  = trackDataList(allBoatIdx(btn)).longitude;
                        boat_type = trackDataList(allBoatIdx(btn)).calf_status;
                        result = analyzeRelativeTrack(dolphinTrack_timeStamp, dolphinTrack_latitude, ...
                            dolphinTrack_longitude, boat_timestamp, boat_lat, boat_lon);
                        if ~isempty(result)
                            nRows = height(result);
                            result.boatType    = repmat(string(boat_type), nRows, 1);
                            result.vidFileName = repmat(string(trackDataList(allBoatIdx(btn)).filename), nRows, 1);
                            cnt = cnt + 1;
                            dolphin_boat_dist{cnt} = result;
                        end
                    end
                end
            end
        end
    end
end

%% --- 6. COLLECT RAW DATA ---
boatTypeList = {'SB', 'MB', 'LB', 'FB'};

raw_data.SB = [];
raw_data.MB = [];
raw_data.LB = [];
raw_data.FB = [];

for fn = 1:numel(dolphin_boat_dist)
    T = dolphin_boat_dist{fn};
    if isempty(T) || height(T) == 0; continue; end
    typeStr = strtrim(T.boatType(1,:));
    if ~any(strcmp(typeStr, boatTypeList)); continue; end

    [minDist, k] = min(T.Distance);
    v_kts = convvel(T.TargetSpeed(k), 'm/s', 'kts');
    raw_data.(typeStr) = [raw_data.(typeStr); v_kts, minDist];
end

%% --- 7. BASELINE VERIFICATION ---
fprintf('\n=== Baseline verification ===\n');
fprintf('Expected: SB ~93.66%%, MB ~1.94%%, LB ~26.55%%, FB ~15.29%%  (>3dB)\n\n');
for bt = 1:length(boatTypeList)
    btype  = boatTypeList{bt};
    data   = raw_data.(btype);
    if isempty(data); continue; end
    coeffs = sl_coeffs.(btype);
    total  = size(data,1);
    RL_delta_all = zeros(total,1);
    for en = 1:total
        v = data(en,1); d = data(en,2);
        if v == 0
            RL_delta_all(en) = 0;
        else
            RL_delta_all(en) = max((coeffs.k(1)*v + coeffs.b(1) - prop_n*log10(d)) - BL_5k, 0);
        end
    end
    p3 = sum(RL_delta_all >= 3) / total * 100;
    p6 = sum(RL_delta_all >= 6) / total * 100;
    fprintf('%s — >3dB: %.2f%%  >6dB: %.2f%%  (n=%d)\n', btype, p3, p6, total);
end

%% --- 8. HELPER FUNCTION ---
function rmse_val = get_speed_rmse(v_kts, bin_edges, bin_rmse)
    rmse_val = bin_rmse(end);
    for b = 1:length(bin_rmse)
        if v_kts >= bin_edges(b) && v_kts < bin_edges(b+1)
            rmse_val = bin_rmse(b);
            return;
        end
    end
end

%% --- 9. SENSITIVITY SIMULATION ---
% Two independent factors:
%   (A) SL uncertainty:    SL ± 1×RMSE_regression
%   (B) Speed uncertainty: v  ± 1×RMSE_speed_bin
% Propagation: fixed at 15×log10(R)

freq_idx      = 1;        % 5 kHz
thresholds    = [3, 6];
sl_scenarios  = [-1, 0, 1];
spd_scenarios = [-1, 0, 1];
sl_labels     = {'-1 SL RMSE', 'Baseline', '+1 SL RMSE'};
spd_labels    = {'-1 Speed RMSE', 'Baseline', '+1 Speed RMSE'};

% results: [sl_scenario x spd_scenario x threshold]
results = struct();
for bt = 1:length(boatTypeList)
    results.(boatTypeList{bt}) = nan(length(sl_scenarios), length(spd_scenarios), length(thresholds));
end

for bt = 1:length(boatTypeList)
    btype  = boatTypeList{bt};
    coeffs = sl_coeffs.(btype);
    data   = raw_data.(btype);
    if isempty(data); continue; end
    n_enc = size(data,1);

    for si = 1:length(sl_scenarios)
        sl_delta = sl_scenarios(si) * coeffs.rmse(freq_idx);

        for spi = 1:length(spd_scenarios)
            for ti = 1:length(thresholds)
                thr = thresholds(ti);
                count_exceed = 0;

                for en = 1:n_enc
                    v = data(en,1);
                    d = data(en,2);

                    if v == 0
                        RL_delta = 0;
                    else
                        spd_rmse    = get_speed_rmse(v, speed_bin_edges, speed_bin_rmse);
                        v_perturbed = max(0, v + spd_scenarios(spi) * spd_rmse);

                        if v_perturbed == 0
                            RL_delta = 0;
                        else
                            SL_perturbed = coeffs.k(freq_idx)*v_perturbed + coeffs.b(freq_idx) + sl_delta;
                            RL_perturbed = SL_perturbed - prop_n * log10(d);
                            RL_delta     = max(RL_perturbed - BL_5k, 0);
                        end
                    end

                    if RL_delta >= thr
                        count_exceed = count_exceed + 1;
                    end
                end

                results.(btype)(si, spi, ti) = (count_exceed / n_enc) * 100;
            end
        end
    end
end

%% --- 10. VISUALIZATION ---
boat_colors = [0.20 0.40 0.60;   % SB
               0.80 0.30 0.30;   % MB
               0.30 0.70 0.30;   % LB
               0.60 0.40 0.80];  % FB
thr_labels = {'3 dB threshold', '6 dB threshold'};
x_pos = 1:4;
bar_width = 0.25;

% Fig 1: SL uncertainty (speed fixed at baseline)
figure('Position',[100 100 1000 500],'Color','w');
sgtitle('Sensitivity to Source Level Uncertainty (±1 RMSE) ', ...
    'FontSize',13,'FontWeight','bold');
sl_colors = [0.45 0.60 0.80; 0.20 0.40 0.60; 0.90 0.50 0.30];
for ti = 1:2
    subplot(1,2,ti); hold on;
    for si = 1:3
        vals = arrayfun(@(b) results.(boatTypeList{b})(si, 2, ti), 1:4);
        bar(x_pos+(si-2)*bar_width, vals, bar_width, ...
            'FaceColor', sl_colors(si,:), ...
            'EdgeColor',[0.2 0.2 0.2],'LineWidth',1.0,'DisplayName',sl_labels{si});
    end
    set(gca,'XTick',x_pos,'XTickLabel',boatTypeList,'FontSize',13);
    ylabel('Encounters Exceeding Threshold (%)','FontSize',12);
    title(thr_labels{ti},'FontSize',13);
    ylim([0 110]);
    legend('Location','northeast','Box','off','FontSize',11);
    box off;
end

% Fig 2: Speed uncertainty (SL fixed at baseline)
figure('Position',[100 400 1000 500],'Color','w');
sgtitle('Sensitivity to Speed Estimation Error (±1 RMSE) ', ...
    'FontSize',13,'FontWeight','bold');
spd_colors = [0.45 0.75 0.55; 0.20 0.55 0.35; 0.85 0.70 0.25];
for ti = 1:2
    subplot(1,2,ti); hold on;
    for spi = 1:3
        vals = arrayfun(@(b) results.(boatTypeList{b})(2, spi, ti), 1:4);
        bar(x_pos+(spi-2)*bar_width, vals, bar_width, ...
            'FaceColor', spd_colors(spi,:), ...
            'EdgeColor',[0.2 0.2 0.2],'LineWidth',1.0,'DisplayName',spd_labels{spi});
    end
    set(gca,'XTick',x_pos,'XTickLabel',boatTypeList,'FontSize',13);
    ylabel('Encounters Exceeding Threshold (%)','FontSize',12);
    title(thr_labels{ti},'FontSize',13);
    ylim([0 110]);
    legend('Location','northeast','Box','off','FontSize',11);
    box off;
end

% Fig 3: Combined summary — baseline + full range (both factors combined)
figure('Position',[660 380 900 500],'Color','w');
sgtitle('Combined Uncertainty Range (SL ± RMSE & Speed ± RMSE)', ...
    'FontSize',12,'FontWeight','bold');
for ti = 1:2
    subplot(1,2,ti); hold on;
    for bt = 1:4
        btype    = boatTypeList{bt};
        all_vals = results.(btype)(:,:,ti);
        all_vals = all_vals(~isnan(all_vals(:)));
        if isempty(all_vals); continue; end
        baseline = results.(btype)(2, 2, ti);
        lo = min(all_vals);
        hi = max(all_vals);
        bar(bt, baseline, 0.55, ...
            'FaceColor', boat_colors(bt,:), ...
            'FaceAlpha', 0.85, ...
            'EdgeColor',[0.15 0.15 0.15],'LineWidth',1.3);
        errorbar(bt, baseline, baseline-lo, hi-baseline, ...
            'k','LineWidth',2.2,'CapSize',12);
    end
    set(gca,'XTick',1:4,'XTickLabel',boatTypeList,'FontSize',13);
    ylabel('Encounters Exceeding Threshold (%)','FontSize',12);
    title(thr_labels{ti},'FontSize',13);
    ylim([0 110]); box off;
end

%% --- 11. SUMMARY TABLE ---
fprintf('\n=====================================================================\n');
fprintf(' SENSITIVITY SUMMARY (propagation fixed: 15×log10(R))\n');
fprintf('=====================================================================\n');

% --- Table 1: Combined range (both SL and speed vary) ---
fprintf('\n[1] Combined range (SL ± RMSE  &  Speed ± RMSE)\n');
fprintf('%-6s  %-6s  %-28s  %-28s\n', 'Type','N','3dB: Baseline [Min–Max]','6dB: Baseline [Min–Max]');
fprintf('---------------------------------------------------------------------\n');
for bt = 1:length(boatTypeList)
    btype = boatTypeList{bt};
    n_enc = size(raw_data.(btype),1);
    for ti = 1:2
        all_vals = results.(btype)(:,:,ti);
        all_vals = all_vals(~isnan(all_vals(:)));
        baseline = results.(btype)(2,2,ti);
        lo = min(all_vals); hi = max(all_vals);
        if ti==1; str3 = sprintf('%.1f%% [%.1f%%–%.1f%%]',baseline,lo,hi);
        else;      str6 = sprintf('%.1f%% [%.1f%%–%.1f%%]',baseline,lo,hi); end
    end
    fprintf('%-6s  %-6d  %-28s  %-28s\n', btype, n_enc, str3, str6);
end

% --- Table 2: SL uncertainty only (speed fixed at baseline, spi=2) ---
fprintf('\n[2] Source level uncertainty only (Speed fixed at baseline)\n');
fprintf('%-6s  %-6s  %-28s  %-28s\n', 'Type','N','3dB: Baseline [Min–Max]','6dB: Baseline [Min–Max]');
fprintf('---------------------------------------------------------------------\n');
for bt = 1:length(boatTypeList)
    btype = boatTypeList{bt};
    n_enc = size(raw_data.(btype),1);
    for ti = 1:2
        % spi=2 (speed baseline), all sl_scenarios
        sl_only_vals = squeeze(results.(btype)(:, 2, ti));
        baseline = results.(btype)(2,2,ti);
        lo = min(sl_only_vals); hi = max(sl_only_vals);
        if ti==1; str3 = sprintf('%.1f%% [%.1f%%–%.1f%%]',baseline,lo,hi);
        else;      str6 = sprintf('%.1f%% [%.1f%%–%.1f%%]',baseline,lo,hi); end
    end
    fprintf('%-6s  %-6d  %-28s  %-28s\n', btype, n_enc, str3, str6);
end

% --- Table 3: Speed uncertainty only (SL fixed at baseline, si=2) ---
fprintf('\n[3] Speed estimation uncertainty only (SL fixed at baseline)\n');
fprintf('%-6s  %-6s  %-28s  %-28s\n', 'Type','N','3dB: Baseline [Min–Max]','6dB: Baseline [Min–Max]');
fprintf('---------------------------------------------------------------------\n');
for bt = 1:length(boatTypeList)
    btype = boatTypeList{bt};
    n_enc = size(raw_data.(btype),1);
    for ti = 1:2
        % si=2 (SL baseline), all spd_scenarios
        spd_only_vals = squeeze(results.(btype)(2, :, ti));
        baseline = results.(btype)(2,2,ti);
        lo = min(spd_only_vals); hi = max(spd_only_vals);
        if ti==1; str3 = sprintf('%.1f%% [%.1f%%–%.1f%%]',baseline,lo,hi);
        else;      str6 = sprintf('%.1f%% [%.1f%%–%.1f%%]',baseline,lo,hi); end
    end
    fprintf('%-6s  %-6d  %-28s  %-28s\n', btype, n_enc, str3, str6);
end

fprintf('\n=====================================================================\n');
fprintf('Note: Baseline = si=2 (SL), spi=2 (speed) — no perturbation applied.\n');
fprintf('      Propagation model fixed at 15×log10(R) throughout.\n');
fprintf('=====================================================================\n');