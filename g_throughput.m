clear;

TR = [6, 9, 12, 18, 24, 36, 48, 54]; % 伝送レート [Mbps]
databit = [24, 36, 48, 72, 96, 144, 192, 216]; % OFDMシンボルごとのデータビット[bit]

PLCP_pre = 16; % PLCPプリアンブル[μs]
PLCPhead_sig = 1; % PLCPヘッダ（シグナル）[μs]
PLCPhead_ser = 16; % PLCPヘッダ（サービス）[μs]
ACK = 80; % 802.11ACKフレーム[bit]
MAC = 192; % 802.11MACヘッダ[bit]
LLC = 64; % LLCヘッダ[bit]
packet = 12000; % IPパケット長[bit]
FCS = 32; % FCS[bit]
tail = 6; % テイルビット[bit]
SIFS = 10; % [μs]
DIFS = 34; % [μs]
backoff = 101.5; % 平均バックオフ制御時間 [μs]

overhead = zeros(size(TR));
payload = zeros(size(TR));
throughput = zeros(size(TR));

for i = 1:length(TR)
    databit_current = databit(i);
    
    % フレームの計算
    ACK_t = PLCP_pre + (PLCPhead_sig + ceil((PLCPhead_ser + ACK + FCS + tail) / databit_current)) * 4;

    data_t = PLCP_pre + (PLCPhead_sig + ceil((PLCPhead_ser + MAC + LLC + packet + FCS + tail) / databit_current)) * 4;

    
    total_tt = (ACK_t + data_t + SIFS +6+ backoff); % トータル時間計算

    throughput(i) = packet / total_tt; % スループット [Mbps]
end

% プロット

figure('Position', [100, 100, 800, 500]); % ウィンドウサイズ指定
hold on;
plot(TR, throughput, '-o', 'Color', 'b', 'LineWidth', 2, 'MarkerSize', 4, 'MarkerFaceColor', 'b');
xlabel('Transmission rate [Mbps]','FontSize', 20, 'FontName', 'Times New Roman');
ylabel('Throughput [Mbps]','FontSize', 20, 'FontName', 'Times New Roman');

% 目盛りのフォントサイズ設定
set(gca, 'FontSize', 18, 'FontName', 'Times New Roman'); % 目盛り数字を大きく設定し、フォントを指定

% グラフの枠を表示
box on;
grid off;

ylim([0, 35]); % y軸の範囲を指定
xlim([0, 60]); % x軸の範囲を指定