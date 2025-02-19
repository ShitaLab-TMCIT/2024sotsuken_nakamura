% パラメータ設定
Tp_values = [0, 10, 20]; % 送信電力 [dBm]
f = 2.4 * 10^9; % 周波数 [Hz]
c = 3 * 10^8; % 光速 [m/s]
d = 1:1000; % 1m から 1000 までの距離 [m]

% 波長の計算
lambda = c / f;

% 自由空間伝搬損失 (FSPL) の計算 [dB]
Lfs = 20 * log10(4 * pi * d / lambda);

% プロット設定
figure('Position', [100, 100, 800, 500]); % ウィンドウサイズ指定
hold on;

% 色の設定
colors = ['r', 'b', 'g']; % 赤、青、緑

% 各送信電力でループ処理
for i = 1:length(Tp_values)
    pt = Tp_values(i) - Lfs; % 受信電力の計算
    plot(d, pt, 'Color', colors(i), 'LineWidth', 2, 'DisplayName', sprintf('%d dBm', Tp_values(i)));
end

% 軸ラベル
xlabel('Distance [m]', 'FontSize', 20, 'FontName', 'Times New Roman');
ylabel('Received Power [dBm]', 'FontSize', 20, 'FontName', 'Times New Roman');

% 軸の設定
set(gca, 'FontSize', 16, 'FontName', 'Times New Roman');
grid off; % グリッドを非表示
box on; % 枠線を表示

% 軸目盛設定
yticks(-100:10:-20); % y軸の目盛を10刻み
xticks(0:100:1000); % x軸の目盛を100刻み

% y軸の範囲設定
ylim([-110 -20]);
xlim([0 1000]);

legend( {'0dBm', '10dBm','20dBm'}, 'FontSize', 20, 'FontName', 'Times New Roman', ...
    'Location', 'northeast', 'Box', 'off'); % 凡例の設定

hold off;
