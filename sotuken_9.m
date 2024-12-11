Rmin = [-82, -81, -79, -77, -74, -70, -66, -65]; % 最小受信感度 [dBm]
TR = [6, 9, 12, 18, 24, 36, 48, 54]; % 伝送レート [Mbps]
databit = [24, 36, 48, 72, 96, 144, 192, 216]; % OFDMシンボルごとのデータビット[bit]

Tp = 10; % 送信電力 [dBm]
f = 2.4 * 10^9; % 周波数 [Hz]
c = 3 * 10^8; % 光速 [m/s]

PLCP_pre = 16; % PLCPプリアンブル[μs]
PLCPhead_sig = 1; % PLCPヘッダ（シグナル）[μs]
PLCPhead_ser = 16; % PLCPヘッダ（サービス）[μs]
ACK = 80; % 802.11ACKフレーム[bit]
MAC = 192; % 802.11MACヘッダ[bit]
LLC = 64; % LLCヘッダ[bit]
packet = 12000; % IPパケット[bit]
FCS = 32; % FCS[bit]
tail = 6; % テイルビット[bit]
SIFS = 10; % [μs]
DIFS = 34; % [μs]
backoff = 101.5; % 平均バックオフ制御時間 [μs]

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
    d1 = ((10^(Lfs / 20)) * c) / (4 * pi * f); % 最大伝送距離 [m]
    
    % フレームの計算
    ACK_t = PLCP_pre + (PLCPhead_sig + ceil((PLCPhead_ser + ACK + FCS + tail) / databit_current)) * 4;
    data_t = PLCP_pre + (PLCPhead_sig + ceil((PLCPhead_ser + MAC + LLC + packet + FCS + tail) / databit_current)) * 4;
    
    % データフレームの中のオーバーヘッド
    data_o_t = PLCP_pre + (PLCPhead_sig + ceil(ceil((PLCPhead_ser + MAC + LLC + packet + FCS + tail) / databit_current) - packet/databit_current)) * 4;

    % 距離の計算
    distances = 0:d1:max_distance; % 距離配列の作成 (d1単位で)
    throughput = zeros(size(distances)); % スループット配列の初期化
    overhead_ratio = zeros(size(distances)); % オーバーヘッド配列の初期化
    
    % スループットとオーバーヘッドの計算
    for j = 1:length(distances)
        D = distances(j);
        % 総伝送時間
        total_tt = (ACK_t + data_t + SIFS + backoff + DIFS + 6) * (D / d1);
        % オーバーヘッドの総時間
        total_o = (ACK_t + data_o_t + SIFS + backoff + DIFS) * (D / d1);
        payload_t = packet / databit_current * 4* (D / d1); % ペイロードの送信時間計算
        throughput(j) = packet / total_tt; % スループット計算 (bit/秒)
        overhead_ratio(j) = total_o / total_tt; % オーバーヘッド率の計算
          % オーバーヘッドの総時間
    total_o_all(i) = total_o;

    % ペイロー
    payload_t_all(i) = payload_t;
    end



        


    % 積み上げ棒グラフの作成
bar(TR, [total_o_all, payload_t_all], 'stacked');
end

% グラフのラベルとタイトル設定
xlabel('伝送レート [Mbps]');
ylabel('時間 [μs]');
title('伝送レートに対するオーバーヘッドとペイロード時間の積み上げ棒グラフ');
legend('オーバーヘッド時間', 'ペイロード時間', 'Location', 'northeastoutside');
grid on;
hold off;
