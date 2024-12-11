clear
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
packet = 12000; % IPパケット長[bit]
FCS = 32; % FCS[bit]
tail = 6; % テイルビット[bit]
SIFS = 10; % [μs]
DIFS = 34; % [μs]
backoff = 101.5; % 平均バックオフ制御時間 [μs]

max_distance = 1000; % 最大距離 [m]

% 初期化
overhead = zeros(size(TR)); % オーバーヘッドの配列
payload = zeros(size(TR)); % ペイロードの配列
N=zeros(size(TR));%送信回数
% フィギュア1: スループットのプロット
figure;
hold on;

for i = 1:length(Rmin)
    Rmin_current = Rmin(i);
    TR_current = TR(i);
    databit_current = databit(i);
    
    % 最大伝送距離の計算
    Lfs = Tp - Rmin_current; % 距離減衰 [dB]
    d_max = ((10^(Lfs / 20)) * c) / (4 * pi * f); % 最大伝送距離 [m]
    
    % フレームの計算
    ACK_t = PLCP_pre + (PLCPhead_sig + ceil((PLCPhead_ser + ACK + FCS + tail) / databit_current)) * 4;
    data_t = PLCP_pre + (PLCPhead_sig + ceil((PLCPhead_ser + MAC + LLC + packet + FCS + tail) / databit_current)) * 4;
    data_o_t = PLCP_pre + (PLCPhead_sig + ceil(ceil((PLCPhead_ser + MAC + LLC + packet + FCS + tail) / databit_current) - packet/databit_current)) * 4;
    
    % スループットを計算して繰り返しプロット
    distances = [0:d_max:max_distance max_distance]; % 1000mも含む
    throughput2 = zeros(size(distances)); % スループット配列の初期化
    payload_t = packet / databit_current * 4;
    N(i)=length(distances);
    for j = 1:length(distances)
        D = distances(j);
        if D == max_distance
            total_tt = (ACK_t + data_t + SIFS + DIFS + backoff) * ((distances(j-1) / d_max) + 1); % トータル時間計算
            overhead(i) = (ACK_t + data_o_t + SIFS + backoff + DIFS + 6) * ((distances(j-1) / d_max) + 1); % オーバーヘッドの総時間
            payload(i) = payload_t * ((distances(j-1) / d_max) + 1); % ペイロードの送信時間
        else
            total_tt = (ACK_t + data_t + SIFS + DIFS + backoff) * (D / d_max); % トータル時間計算
            overhead(i) = (ACK_t + data_o_t + SIFS + backoff + DIFS + 6) * (D / d_max); % オーバーヘッドの総時間
            payload(i) = payload_t * (D / d_max); % ペイロードの送信時間
        end
        throughput2(j) = packet / total_tt; % スループット [Mbps]
    end
    
    % 距離を50m単位で丸める
    distances_adjusted = floor(distances / 50) * 50;
    
    % プロット
    plot(distances_adjusted, throughput2, '-o', 'LineWidth', 1, 'MarkerSize', 4, 'DisplayName', sprintf('伝送レート:%d Mbps', TR_current));
end

% グラフのラベルとタイトル設定
xlabel('距離 [m]');
ylabel('スループット [Mbps]');
title('最大伝送距離毎のスループット計算（到達距離1000mまで）');
legend('show');
grid on;
hold off;

% フィギュア2: オーバーヘッドとペイロード時間の積み上げ棒グラフ
figure;
bar(TR, [overhead', payload'], 'stacked');

% グラフのラベルとタイトル設定
xlabel('伝送レート [Mbps]');
ylabel('時間 [μs]');
title('伝送レートに対するオーバーヘッドとペイロードの時間');
