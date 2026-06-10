function result = analyzeRelativeTrack(refTime, refLat, refLon, targetTime, targetLat, targetLon)
% analyzeRelativeTrack: Compare filtered reference and target tracks using WGS84 and Kalman filter.
%
% Output:
%   result - Table with matched reference and target track information

% Define WGS84 ellipsoid
wgs84 = wgs84Ellipsoid('meters');

%% === 1. Interpolate & Filter Target Track ===
% Remove duplicate timestamps from targetTime
%% === 1. Interpolate & Filter Target Track ===

% 1a) Remove any NaN/Inf in time, latitude or longitude
if isa(targetTime,'datetime')
    validTime = ~ismissing(targetTime);
else
    validTime = ~isnan(targetTime);
end
validLat = isfinite(targetLat);
validLon = isfinite(targetLon);

keep = validTime & validLat & validLon;
targetTime = targetTime(keep);
targetLat  = targetLat(keep);
targetLon  = targetLon(keep);

% 1b) Now remove duplicate timestamps
[uniqueTimes, uniqueIdx] = unique(targetTime);
targetLat = targetLat(uniqueIdx);
targetLon = targetLon(uniqueIdx);

% 1c) Build a dense, zero‐based time vector
t_common = seconds(uniqueTimes - uniqueTimes(1));
t_dense  = (0:1:round(t_common(end)))';

% 1d) Interpolate (now guaranteed finite)
targetLatInterp = interp1(t_common, targetLat, t_dense, 'linear', 'extrap');
targetLonInterp = interp1(t_common, targetLon, t_dense, 'linear', 'extrap');
timeInterp      = uniqueTimes(1) + seconds(t_dense);

% Smooth interpolated target track
[~, ~, lat_tgt_filt, lon_tgt_filt, ~, ~, vx, vy] = ...
    kalmanFilterTrack(timeInterp, targetLatInterp, targetLonInterp);

targetSpeed = sqrt(vx.^2 + vy.^2);

%% === 2. Filter Reference Track ===
[~, ~, refLat_filt, refLon_filt] = ...
    kalmanFilterTrack(refTime, refLat, refLon);

%% === 3. Compute Headings of Ref Track ===
N = numel(refTime);
headings = zeros(N,1);
for i = 2:N-1
    headings(i) = azimuth(refLat_filt(i-1), refLon_filt(i-1), ...
                          refLat_filt(i+1), refLon_filt(i+1), wgs84);
end
headings(1) = headings(2);
headings(end) = headings(end-1);
headings = mod(headings, 360);

%% === 4. Match with Time Tolerance ===
timeTolerance = seconds(1);  % maximum allowed time difference
% Preallocate matched data arrays
matchedRefIdx = [];  % index into refTime
matchedTgtIdx = [];  % index into timeInterp
distances = [];
relAngles = [];
speeds = [];

for i = 1:N
    % Find index of closest target time
    [timeDiff, idx] = min(abs(timeInterp - refTime(i)));

    if timeDiff > timeTolerance
        continue  % skip if no close match
    end

    tgtLat = lat_tgt_filt(idx);
    tgtLon = lon_tgt_filt(idx);
    speed = targetSpeed(idx);

    % Compute distance
    dist = distance(refLat_filt(i), refLon_filt(i), tgtLat, tgtLon, wgs84);

    % Azimuth and relative angle
    az = azimuth(refLat_filt(i), refLon_filt(i), tgtLat, tgtLon, wgs84);
    az = mod(az, 360);
    relAng = mod(az - headings(i), 360);

    % Store data
    matchedRefIdx(end+1, 1) = i;      %#ok<AGROW>
    matchedTgtIdx(end+1, 1) = idx;    %#ok<AGROW>
    distances(end+1, 1)     = dist;   %#ok<AGROW>
    relAngles(end+1, 1)     = relAng; %#ok<AGROW>
    speeds(end+1, 1)        = speed;  %#ok<AGROW>
end

% Build output table
result = table(refTime(matchedRefIdx), ...
               refLat_filt(matchedRefIdx), refLon_filt(matchedRefIdx), ...
               lat_tgt_filt(matchedTgtIdx), lon_tgt_filt(matchedTgtIdx), ...
               distances, relAngles, speeds, ...
               'VariableNames', {'Time', 'RefLat', 'RefLon', ...
                                 'TargetLat', 'TargetLon', ...
                                 'Distance', 'RelativeAngle', 'TargetSpeed'});

end
