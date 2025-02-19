% 各伝送レートにおける最大伝送距離の計算
Rmin = [-82 -81 -79 -77 -74 -70 -66 -65]; % 最小受信感度[dBm]
Tp_values = [0, 10, 20]; % 送信電力[dBm]の異なる値
f = 2.4 * 10^9;
c = 3 * 10^8;
TR = [6 9 12 18 24 36 48 54];

% 色指定
colors = {'r', 'b', 'g'};

figure('Position', [100, 100, 800, 500]);
hold on;

legend_handles = []; % 凡例用のハンドル

for j = 1:length(Tp_values)
    Tp = Tp_values(j);
    d = zeros(size(Rmin));
    
    for i = 1:length(Rmin)
        Lfs = Tp - Rmin(i);
        d(i) = ((10^(Lfs/20)) * c) / (4 * pi * f);
    end
    
    % プロット
    for i = 1:length(d)-1
        x = linspace(d(i), d(i+1), 2);
        y = ones(size(x)) * TR(i);
        h = plot(x, y, 'Color', colors{j}, 'LineWidth', 2);
        
        if i == 1  % 凡例用に最初の線のハンドルを保存
            legend_handles(j) = h;
        end
    end
    
    % 最後の区間もプロット
    x = linspace(d(end), 0, 100);
    y = ones(size(x)) * TR(end);
    plot(x, y, 'Color', colors{j}, 'LineWidth', 2);
end

xlabel('Distance [m]', 'FontSize', 20, 'FontName', 'Times New Roman');
ylabel('Transmission rate [Mbps]', 'FontSize', 20, 'FontName', 'Times New Roman');
set(gca, 'FontSize', 16, 'FontName', 'Times New Roman');
grid off;
box on;

% 軸のメモリ設定
yticks(0:10:60);
xticks(0:100:1000);
ylim([0 60]);
xlim([0 1000]);

% 凡例（プロットの色と一致）
legend(legend_handles, {'0dBm', '10dBm', '20dBm'}, 'FontSize', 20, 'FontName', 'Times New Roman', ...
    'Location', 'northeast', 'Box', 'off');

hold off;
