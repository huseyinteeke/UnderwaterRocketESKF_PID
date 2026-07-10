% --- SİMÜLASYON ZAMAN VE ORTAM AYARLARI ---
dt = 0.005;               % 200 Hz örnekleme süresi (Simulink ayarınızla aynı olmalı)
t = (0 : dt : 10)';       % 0'dan 10 saniyeye kadar zaman vektörü
N = length(t);            % Toplam veri sayısı (2001 adet)

% =========================================================
% 1. SENARYO: PWM SİNYALİ ÜRETİMİ (sim_pwm)
% =========================================================
% Başlangıçta motor kapalı (1000 us)
pwm_data = 1000 * ones(N, 1); 

% 2. saniye ile 6. saniye arasında gaza basıyoruz (1800 us)
pwm_data(t >= 2 & t < 6) = 1800; 

% (6. saniyeden sonra kod 1000'de kalmaya devam edecek, yani gazı kestik)

% =========================================================
% 2. SENARYO: IMU İVME VERİSİ ÜRETİMİ (sim_imu)
% =========================================================
gercek_ivme = zeros(N, 1);

% 2-6 saniye arası: Motor devrede, araç ileri atılıyor (Örn: 1.5 m/s^2 ivme)
gercek_ivme(t >= 2 & t < 6) = 1.5; 

% 6-10 saniye arası: Gaz kesildi! Araç suyun direnciyle yavaşlıyor (Süzülme/Coasting)
gercek_ivme(t >= 6 & t <= 10) = -0.8; 

% IMU'nun karakteristik hatalarını ekleyelim (Bias ve Gürültü)
imu_bias = 0.05;                        % Sensörde sabit 0.05 m/s^2'lik bir kayma (drift) var
imu_noise = 0.02 * randn(N, 1);         % Motor ve su titreşimlerini taklit eden beyaz gürültü

% Ham IMU verisini oluştur (Gerçek İvme + Bias + Gürültü)
imu_data = gercek_ivme + imu_bias + imu_noise;

% =========================================================
% 3. SIMULINK İÇİN TIMESERIES OBJELERİNE ÇEVİRME
% =========================================================
% Simulink'in doğrudan okuyabileceği formata dönüştürüyoruz
sim_pwm = timeseries(pwm_data, t);
sim_imu = timeseries(imu_data, t);

disp('Simulink verileri (sim_pwm ve sim_imu) Workspace''e yüklendi. Play tuşuna basabilirsiniz!');