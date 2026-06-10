function [xEast, yNorth, lat_filt, lon_filt, timeVec, totalDist, vx, vy] = kalmanFilterTrack(timeVec, lat, lon)
% Kalman filter on lat/lon and convert to ENU with WGS84
wgs84 = wgs84Ellipsoid('meters');
originLat = lat(1);
originLon = lon(1);
originAlt = 0;

[xEast, yNorth, ~] = geodetic2enu(lat, lon, zeros(size(lat)), ...
                                  originLat, originLon, originAlt, wgs84);

dt_vec = seconds(diff(timeVec));
dt = [dt_vec(1); dt_vec];

N = numel(timeVec);
x_est = zeros(4, N);
P = eye(4);
g = 1;
dR = 10;
x_est(:,1) = [xEast(1); 0; yNorth(1); 0];

for k = 2:N
    F = [1, dt(k), 0,     0;
         0,     1, 0,     0;
         0,     0, 1, dt(k);
         0,     0, 0,     1];
    H = [1, 0, 0, 0;
         0, 0, 1, 0];

    Q = g * [dt(k)^3/3, dt(k)^2/2,         0,           0;
             dt(k)^2/2,     dt(k),         0,           0;
                     0,         0, dt(k)^3/3, dt(k)^2/2;
                     0,         0, dt(k)^2/2,     dt(k)];

    x_pred = F * x_est(:,k-1);
    P_pred = F * P * F' + Q;

    z = [xEast(k); yNorth(k)];
    K = P_pred * H' / (H * P_pred * H' + dR * eye(2));
    x_est(:,k) = x_pred + K * (z - H * x_pred);
    P = (eye(4) - K * H) * P_pred;
end

x_filt = x_est(1,:)';
y_filt = x_est(3,:)';
vx = x_est(2,:)';
vy = x_est(4,:)';

[lat_filt, lon_filt, ~] = enu2geodetic(x_filt, y_filt, zeros(N,1), ...
                                       originLat, originLon, originAlt, wgs84);

segment_distances = sqrt(diff(x_filt).^2 + diff(y_filt).^2);
totalDist = sum(segment_distances);
end
