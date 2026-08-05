function [time, IMU, pwm, sim_imu, sim_pwm] = csvhandling(filename)
    
    data = readtable(filename);
    
    % Zaman sütununu temizle ve sırala
    data = rmmissing(data, 'DataVariables', 'Time');
    [data, ~] = sortrows(data, 'Time');
    [~, uniqueIdx] = unique(data.Time, 'stable');
    data = data(uniqueIdx, :);
    
    % Zamanı saniyeye çevir ve 0'dan başlat
    raw_time = double(data.Time);
    time = (raw_time - raw_time(1)) / 1000; % .Time hatası düzeltildi

    % Zaman çakışmasını önle
    for i = 2:length(time)
        if time(i) <= time(i-1)
            time(i) = time(i-1) + 0.001; 
        end
    end

    % --- ETİKETLEME VE MATRİS OLUŞTURMA ---
    % Sütunları sırasıyla net bir şekilde eşleştiriyoruz:
    % IMU sütun sırası: [pitch, yaw, roll, Ax, Ay, Az]
    col_pitch = double(data.pitch);
    col_yaw   = double(data.yaw);
    col_roll  = double(data.roll);
    col_Ax    = double(data.Ax);
    col_Ay    = double(data.Ay);
    col_Az    = double(data.Az);

    IMU = [col_pitch, col_yaw, col_roll, col_Ax, col_Ay, col_Az];
    
    % Aktüatör / PWM verileri
    col_rpm         = double(data.rpm);
    col_rudderangle = double(data.rudderangle);
    col_sternangle  = double(data.sternangle);
    
    pwm = [col_rpm, col_rudderangle, col_sternangle];

    % Simulink From Workspace için Timeseries nesneleri
    sim_imu  = timeseries(col_Ax, time);
    sim_pwm  = timeseries(pwm, time);

end