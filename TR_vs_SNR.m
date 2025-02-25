clear
Rmin = [-82, -81, -79, -77, -74, -70, -66, -65]; % 最小受信感度 [dBm]
TR = [6, 9, 12, 18, 24, 36, 48, 54]; % 伝送レート [Mbps]
databit = [24, 36, 48, 72, 96, 144, 192, 216]; % OFDMシンボルごとのデータビット[bit]

Tp = 10; % 送信電力 [dBm]
f = 2.4 * 10^9; % 周波数 [Hz]
c = 3 * 10^8; % 光速 [m/s]

% 結果格納用の配列
d_adjusted = zeros(1, length(Rmin));
R = zeros(1, length(Rmin)); % 所望信号
N = zeros(1, length(Rmin)); % 雑音信号
SNRmin = zeros(1, length(Rmin));
SNR = 20 * log10(3); % SNR一定値

for i = 1:length(Rmin)
    % 距離減衰 [dB]
    Lfs = Tp - Rmin(i);
    
    % 最大伝送距離 [m]
    d_max = ((10^(Lfs / 20)) * c) / (4 * pi * f);
    
    % 最大伝送距離を50[m]区切りに調整
    d_adjusted(i) = floor(d_max / 50) * 50;

    % 調整された距離での受信信号 [dBm]
    R(i) = Tp - 20 * log10(4 * pi * d_adjusted(i) / (c / f)); % 所望信号[dBm]
    N(i) = Tp - 20 * log10(4 * pi * 3 * d_adjusted(i) / (c / f)); % 雑音信号[dBm]
    
    % SNRminの計算
    SNRmin(i) = Rmin(i) + 85;
end

% プロット
figure;
plot(TR, SNRmin, '-o', 'LineWidth', 1,'MarkerSize', 4); % SNRminをプロット
hold on;
yline(SNR, 'r--', 'LineWidth', 1); % SNR一定値の横線

xlabel('伝送レート[Mbps]');
ylabel('SNR [dB]');
title(['各伝送レートの通信に必要なSNRと端末4におけるSNR']);
legend('各伝送レートの通信に必要なSNR', '端末4におけるSNR');
grid on;
hold off;



