Rmin = [-82, -81, -79, -77, -74, -70, -66, -65]; % 最小受信感度 [dBm]
TR = [6, 9, 12, 18, 24, 36, 48, 54]; % 伝送レート [Mbps]
databit = [24, 36, 48, 72, 96, 144, 192, 216]; % OFDMシンボルごとのデータビット [bit]

Tp = 10; % 送信電力 [dBm]
f = 2.4 * 10^9; % 周波数 [Hz]
c = 3 * 10^8; % 光速 [m/s]
d = 50; % 距離のステップ [m]
PLCP_pre = 16; % PLCPプリアンブル [μs]
PLCPhead_sig = 1; % PLCPヘッダ（シグナル） [μs]
PLCPhead_ser = 16; % PLCPヘッダ（サービス） [μs]
ACK = 80; % 802.11ACKフレーム [bit]
MAC = 192; % 802.11MACヘッダ [bit]
LLC = 64; % LLCヘッダ [bit]
packet = 12000; % IPパケット [bit]
FCS = 32; % FCS [bit]
tail = 6; % テイルビット [bit]
SIFS = 10; % [μs]
DIFS = 34; % [μs]
backoff = 101.5; % 平均バックオフ制御時間 [μs]
ACK_t = (PLCP_pre + PLCPhead_sig + PLCPhead_ser + (ACK / databit(1))) * 1e-6; % ACK時間 [s]
data_t = (PLCP_pre + PLCPhead_sig + PLCPhead_ser + (packet + MAC + LLC + FCS + tail) / databit(1)) * 1e-6; % データフレーム時間 [s]

% フィギュアの設定
figure;
hold on;

max_distance = 1000; % 最大距離 [m]

for i = 1:length(Rmin)
    Rmin_current = Rmin(i);
    TR_current = TR(i);
    databit_current = databit(i);

    % 最大伝送距離の計算
    Lfs = Tp - Rmin_current; % 距離減衰 [dB]
    d_max = ((10^(Lfs / 20)) * c) / (4 * pi * f); % 最大距離 [m]

    % 距離の配列を作成
    distances = 0:d:max_distance; 
    throughput2 = zeros(size(distances)); % スループット配列の初期化

    for j = 1:length(distances)
        D = distances(j);
        if D > d_max
            throughput2(j) = 0; % 伝送距離がd_maxを超えた場合はスループットを0に
        else
            total_tt = (ACK_t + data_t + SIFS * 1e-6 + backoff * 1e-6 + DIFS * 1e-6); % 総送信時間
            throughput2(j) = packet / total_tt * 1e-6; % スループットの計算 [Mbps]
        end
    end

    % プロット
    plot(distances, throughput2, '-o', 'LineWidth', 1, 'MarkerSize', 4, 'DisplayName', sprintf('伝送レート: %d Mbps', TR_current));
end

% グラフのラベルとタイトル設定
xlabel('距離 [m]');
ylabel('スループット [Mbps]');
title('距離とスループットの関係');
legend('show');
grid on;
hold off;
