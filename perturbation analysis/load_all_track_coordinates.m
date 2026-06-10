function trackDataList = load_all_track_coordinates(topFolder)
    % 모든 CSV 파일 검색
    csvFiles = dir(fullfile(topFolder, '*.csv'));
    trackDataList = [];  % 결과 구조체 초기화
    entryIdx = 1;

    if isempty(csvFiles)
        warning('No CSV files found in the specified folder.');
        return;
    end

    for i = 1:length(csvFiles)
        filePath = fullfile(csvFiles(i).folder, csvFiles(i).name);
        try
            T = readtable(filePath);
        catch
            fprintf('[ERROR] Cannot read CSV: %s\n', filePath);
            continue;
        end

        % 필수 열 확인
        requiredCols = {'track', 'latitude', 'longitude', 'timestamp', 'calf_status'};
        if ~all(ismember(requiredCols, T.Properties.VariableNames))
            fprintf('[SKIP] Missing required columns in %s\n', filePath);
            continue;
        end

        trackIDs = unique(T.track);  % track is string-based (e.g., 'track01')
        [~, baseName, ~] = fileparts(filePath);
        
        for t = 1:length(trackIDs)
            tid = trackIDs{t};  % Get track name as string
        
            trackRows = T(strcmp(T.track, tid), :);  % Safe string-based comparison
        
            trackDataList(entryIdx).filename     = baseName;
            trackDataList(entryIdx).trackID      = tid;
            trackDataList(entryIdx).calf_status  = trackRows.calf_status{1};
            trackDataList(entryIdx).latitude     = trackRows.latitude;
            trackDataList(entryIdx).longitude    = trackRows.longitude;
            trackDataList(entryIdx).timestamp    = trackRows.timestamp;
        
            entryIdx = entryIdx + 1;
        end
    end
end
