% パラメータ設定
Tp = 10; % 送信電力 [dBm]
f = 2.4 * 10^9; % 周波数 [Hz]
c = 3 * 10^8; % 光速 [m/s]
d = 1:400; % 1m から 500m までの距離 [m]

% 波長の計算
lambda = c / f;

% 自由空間伝搬損失 (FSPL) の計算 [dB]
Lfs = 20 * log10(4 * pi * d / lambda);

% 受信電力の計算
pt = Tp - Lfs;

% プロット設定
figure('Position', [100, 100, 800, 500]); % ウィンドウサイズ指定
hold on;

% 受信電力のプロット
plot(d, pt, 'b', 'LineWidth', 2); % 青色の線

% 軸ラベル
xlabel('Distance [m]', 'FontSize', 20, 'FontName', 'Times New Roman');
ylabel('Received Power [dBm]', 'FontSize', 20, 'FontName', 'Times New Roman');

% 軸の設定
set(gca, 'FontSize', 20, 'FontName', 'Times New Roman');
grid off; % グリッドを表示
box on; % 枠線を表示

% 軸目盛設定
yticks(-100:10:-30); % y軸の目盛を10刻み
xticks(0:100:400); % x軸の目盛を100刻み

% y軸の範囲設定
ylim([-100 -30]);

hold off;
