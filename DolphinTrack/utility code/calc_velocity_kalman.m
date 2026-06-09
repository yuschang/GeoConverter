function v_kalman_kmh = calc_velocity_kalman(lat, lon, timeVec)
    % Inputs:
    %   lat: Vector of latitudes (degrees)
    %   lon: Vector of longitudes (degrees)
    %   timeVec: Vector of timestamps (datetime or duration)
    
    n = length(lat);
    
    % 1. Convert Lat/Lon to Meters (Local XY coordinates)
    % We use the mean latitude to minimize distortion for local projections
    R = 6371000; % Earth radius in meters
    latRad = deg2rad(lat);
    lonRad = deg2rad(lon);
    
    % Project to flat surface (Equirectangular approximation)
    % Center the coordinates around 0 to avoid large numbers
    refLat = mean(latRad);
    refLon = mean(lonRad);
    
    x_obs = R * (lonRad - refLon) .* cos(refLat);
    y_obs = R * (latRad - refLat);
    
    % 2. Kalman Filter Initialization
    % State vector: [x; y; vx; vy]
    % Initialize with the first measured position and 0 velocity
    X = [x_obs(1); y_obs(1); 0; 0]; 
    
    % Initial Covariance (High uncertainty for velocity)
    P = diag([10, 10, 100, 100]); 
    
    % Measurement Matrix (We only measure position x [row 1] and y [row 2])
    H = [1 0 0 0;
         0 1 0 0];
         
    % Measurement Noise (R_mat): How much we trust the GPS position
    % VBOX GPS is accurate to ~0.5m, so 5 is a reasonable variance
    R_mat = eye(2) * 5; 
    
    % Process Noise (Q): How much we trust the Constant Velocity model
    % Lower = smoother velocity, Higher = more responsive to acceleration
    Q_base = eye(4) * 0.1;
    
    % Storage
    X_store = zeros(4, n);
    X_store(:, 1) = X; % Store initial state
    
    % 3. Kalman Filter Loop
    % Start from i=2 because we need the previous time to calculate dt
    for i = 2:n
        % Calculate dt (time step) in seconds
        dt = seconds(timeVec(i) - timeVec(i-1));
        
        % SAFETY CHECK: Handle duplicate times or zero dt
        if dt <= 0
            dt = 0.04; % Fallback to standard 25Hz interval if time is identical
        end
        
        % State Transition Matrix (Physics model: x = x0 + v*dt)
        % This MUST be updated every loop because dt changes
        F = [1 0 dt 0;
             0 1 0  dt;
             0 0 1  0;
             0 0 0  1];
             
        % Process Noise Scaling
        % If there is a huge time gap (e.g. > 1 sec), uncertainty grows
        if dt > 1.0
            current_Q = Q_base * 1000; % "Reset" filter confidence
        else
            current_Q = Q_base;
        end
        
        % --- PREDICTION STEP ---
        X_pred = F * X;
        P_pred = F * P * F' + current_Q;
        
        % --- UPDATE STEP ---
        Z = [x_obs(i); y_obs(i)]; % Actual measurement
        
        % Kalman Gain
        S = H * P_pred * H' + R_mat; % Innovation covariance
        K = P_pred * H' / S;
        
        % Update State and Covariance
        X = X_pred + K * (Z - H * X_pred);
        P = (eye(4) - K * H) * P_pred;
        
        % Store result
        X_store(:, i) = X;
    end
    
    % 4. Extract Filtered Velocity
    vx_filtered = X_store(3, :);
    vy_filtered = X_store(4, :);
    
    % Calculate speed (Magnitude of velocity vector)
    v_mps = sqrt(vx_filtered.^2 + vy_filtered.^2);
    
    % Convert to km/h
    v_kalman_kmh = v_mps' * 3.6; % Transpose to match input shape if needed
end