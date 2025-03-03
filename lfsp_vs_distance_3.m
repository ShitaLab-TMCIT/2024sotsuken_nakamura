% パラメータ設定
Tp = 10; % 送信電力 [dBm]
f = 2.4 * 10^9; % 周波数 [Hz]
c = 3 * 10^8; % 光速 [m/s]
d = 1:1000; % 1m から 1000m までの距離 [m]
G = 2.14; % 利得 [dBi]

% 波長の計算
lambda = c / f;

% 自由空間伝搬損失 (FSPL) の計算 [dB]
Lfs1 = 20 * log10(4 * pi * d / lambda);
Lfs2 = 26 * log10(4 * pi * d / lambda);

% 受信電力の計算
pt1 = Tp - Lfs1 + 2*G;
pt2 = Tp - Lfs2 + 2*G;

% プロット設定
figure('Position', [100, 100, 800, 500]); % ウィンドウサイズ指定
hold on;

% 受信電力のプロット (x軸を対数スケール)
semilogx(d, pt1, 'b', 'LineWidth', 2); % 青色の線 (20log10)
semilogx(d, pt2, 'r', 'LineWidth', 2); % 赤色の線 (26log10)

% x 軸を対数スケールに変更
set(gca, 'XScale', 'log');

% -90 dBm の水平線
yline(-90, '-k', 'LineWidth', 2, 'LabelHorizontalAlignment', 'left');

% 軸ラベル
xlabel('Distance [m] ', 'FontSize', 20, 'FontName', 'Times New Roman');
ylabel('Received Power [dBm]', 'FontSize', 20, 'FontName', 'Times New Roman');

% 軸の設定
set(gca, 'FontSize', 20, 'FontName', 'Times New Roman');
grid off; % グリッドを表示
box on; % 枠線を表示

% 軸目盛設定
yticks(-100:10:-30); % y軸の目盛を10刻み
xticks([1, 10, 100, 1000]); % x軸の目盛を対数スケールに合わせる
xticklabels({'1', '10', '100', '1000'}); % 通常の数値で表示

% y軸の範囲設定
ylim([-100 -30]);

% 凡例  
legend( {'Free space propagation loss', 'Propagation multiplier = 2.6'}, 'FontSize', 20, 'FontName', 'Times New Roman', ...
    'Location', 'northeast', 'Box', 'off'); % 凡例の設定

hold off;
