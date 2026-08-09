function [time, IMU, pwm, sim_imu, sim_pwm] = csvhandling(filename)
    % --- KUSURSUZ CSV -> MATLAB & SİMULİNK AKTARIM FONKSİYONU ---
    
    % 1. Tabloyu Oku
    data = readtable(filename);
    
    % 2. Zaman Sütununu Akıllıca Bul ve Temizle
    timeCol = findColName(data, 'time');
    data = rmmissing(data, 'DataVariables', timeCol);
    [data, ~] = sortrows(data, timeCol);
    
    % Aynı zaman damgasına sahip mükerrer satırları temizle
    [~, uniqueIdx] = unique(data.(timeCol), 'stable');
    data = data(uniqueIdx, :);
    
    % 3. Ham Zamanı Saniyeye Çevir 
    raw_time = double(data.(timeCol));
    
    if mean(diff(raw_time)) > 50000
        time = (raw_time - raw_time(1)) / 1000000; % Mikrosaniye ihtimaline karşı
    else
        time = (raw_time - raw_time(1)) / 1000;    % Milisaniye
    end
    
    % Simulink'in çökmemesi için zaman vektörünün monoton artışını garanti et
    for i = 2:length(time)
        if time(i) <= time(i-1)
            time(i) = time(i-1) + 0.001; 
        end
    end
    
    % 4. IMU ve Aktüatör Sütunlarını Büyük/Küçük Harf Takıntısı Olmadan Al
    col_pitch = double(data.(findColName(data, 'pitch')));
    col_yaw   = double(data.(findColName(data, 'yaw')));
    col_roll  = double(data.(findColName(data, 'roll')));
    col_Ax    = double(data.(findColName(data, 'ax')));
    col_Ay    = double(data.(findColName(data, 'ay')));
    col_Az    = double(data.(findColName(data, 'az')));
    
    col_rpm         = double(data.(findColName(data, 'rpm')));
    col_rudderangle = double(data.(findColName(data, 'rudderangle')));
    col_sternangle  = double(data.(findColName(data, 'sternangle')));
    
    % 5. MATLAB Matris Çıktıları
    IMU = [col_pitch, col_yaw, col_roll, col_Ax, col_Ay, col_Az];
    pwm = [col_rpm, col_rudderangle, col_sternangle];
    
    % 6. Simulink "From Workspace" İçin Kesin Uyumlu Timeseries Nesneleri
    % --- SİMULİNK İÇİN EN GARANTİ YÖNTEM: 2 SÜTUNLU MATRİS ---
    % 1. Sütun: Zaman, 2. Sütun: Veri
    sim_imu = [time, col_Ax];
    sim_pwm = [time, col_rpm];
end

% --- AKILLI SÜTUN BULUCU ---
function colName = findColName(tbl, target)
    vars = tbl.Properties.VariableNames;
    idx = find(strcmpi(vars, target));
    if isempty(idx)
        targetAlt = strrep(target, '_', '');
        for i = 1:length(vars)
            if strcmpi(strrep(vars{i}, '_', ''), targetAlt)
                colName = vars{i};
                return;
            end
        end
        error('CSV içinde "%s" sütunu bulunamadı! Tablo başlıklarını kontrol et.', target);
    end
    colName = vars{idx(1)};
end