% 各伝送レートにおける最大伝送距離の計算
Rmin = [-82 -81 -79 -77 -74 -70 -66 -65]; % 最小受信感度[dBm]
Tp = 10; % 送信電力[dBm]
f = 2.4 * 10^9;
c = 3 * 10^8;
d = zeros(size(Rmin));
Lfs = zeros(size(Rmin));
TR = [6 9 12 18 24 36 48 54];

% 変調方式と符号化率の対応表
mod_coding = {
    'BPSK 1/2', 'BPSK 3/4', ...
    'QPSK 1/2', 'QPSK 3/4', ...
    '16QAM 1/2', '16QAM 3/4', ...
    '64QAM 2/3', '64QAM 3/4'
};

for i = 1:length(Rmin)
    Lfs(i) = Tp - Rmin(i);
    d(i) = ((10^(Lfs(i)/20)) * c) / (4 * pi * f);
end

% プロット
figure('Position', [100, 100, 800, 500]);
hold on;

for i = 1:length(d)-1
    x = linspace(d(i), d(i+1), 2);
    y = ones(size(x)) * TR(i);
    plot(x, y, 'b', 'LineWidth', 2);
    

    text(mean(x)+3, TR(i) + 3, mod_coding{i}, 'FontSize', 13, 'FontName', 'Times New Roman', ...
        'Color', 'k', 'HorizontalAlignment', 'center');
end

% 最後の区間もプロット
x = linspace(d(end), 0, 100);
y = ones(size(x)) * TR(end);
plot(x, y, 'b', 'LineWidth', 2);


text(mean(x), TR(end) + 3, mod_coding{end}, 'FontSize', 13, 'FontName', 'Times New Roman', ...
    'Color', 'k', 'HorizontalAlignment', 'center');

xlabel('Distance [m]', 'FontSize', 20, 'FontName', 'Times New Roman');
ylabel('Transmission rate[Mbps]', 'FontSize', 20, 'FontName', 'Times New Roman');
set(gca, 'FontSize', 20, 'FontName', 'Times New Roman');
grid off;
box on;
hold off;

% 軸のメモリ設定
yticks(0:10:60);
xticks(0:100:400);
ylim([0 60]);
