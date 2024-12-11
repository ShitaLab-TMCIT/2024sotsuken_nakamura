% パラメータの設定
Rmin = [-82, -81, -79, -77, -74, -70, -66, -65]; % 最小受信感度 [dBm]
Tp =10; % 送信電力 [dBm]
f = 2.4* 10^9; % 周波数 [Hz]
c = 3 * 10^8; % 光速 [m/s]
d = [25, 50, 75, 100]; % 端末間の距離 [m]
N = 100; % 端末数
TR = [6, 9, 12, 18, 24, 36, 48, 54]; % 伝送レート [Mbps]

databit = [24, 36, 48, 72, 96, 144, 192, 216]; % OFDMシンボルごとのデータビット[bit]
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

% グラフの初期化
figure;
hold on;

% マーカーの種類
markers = {'o', 'x', '*', 's'}; % 円, x, アスタリスク, 四角

% 各距離に対してスループットを計算
for j = 1:length(d)
    % 変数の初期化
    n = zeros(size(TR)); % 一度に通信できる端末数
    nt = zeros(size(TR)); % 総送信回数
    d1 = zeros(size(TR)); % 各変調方式における伝送距離[m]
    Lfs = zeros(size(TR)); % 距離減衰 [dB]
    ACK_t= zeros(size(TR)); % ACKフレーム長[μs]
    data_t= zeros(size(TR)); % データ長[μs]
    throughput1 = zeros(size(TR)); % 端末間のスループット [Mbps]
    throughput2 = zeros(size(TR)); % 全体のスループット [Mbps]
    total_tt = zeros(size(TR)); % 1パケットの全体(100台)の送信時間[μs]
    
    for i = 1:length(TR)
        Lfs(i) = Tp - Rmin(i); % 距離減衰 [dB]
        d1(i) = ((10^(Lfs(i) / 20)) * c) / (4 * pi * f); % 伝送距離 [m]
        n(i) = floor(d1(i) / d(j)); % 一度に通信できる端末数

        % 総送信回数の計算
        if mod(N - 1, n(i)) == 0
            nt(i) = (N - 1) / n(i);
        else
            nt(i) = floor((N - 1) / n(i)) + 1;
        end

        % フレームの計算
        ACK_t(i) = PLCP_pre + (PLCPhead_sig + ceil((PLCPhead_ser + ACK + FCS + tail) / databit(i))) * 4;
        data_t(i) = PLCP_pre + (PLCPhead_sig + ceil((PLCPhead_ser + MAC + LLC + packet + FCS + tail) / databit(i))) * 4;

        % スループットの計算
        throughput1(i) = packet / (ACK_t(i) + data_t(i) + SIFS + backoff);
        total_tt(i) = (ACK_t(i) + data_t(i) + SIFS + backoff+6) * nt(i);
        throughput2(i) = packet / total_tt(i);
    end

    % グラフにプロット
    plot(TR, throughput2, 'LineWidth', 1.5, 'Marker', markers{j}, 'DisplayName', ['距離 ' num2str(d(j)) ' m']);
end

xlabel('伝送レート [Mbps]');
ylabel('スループット [Mbps]');
title('各伝送レートに対するスループット');
legend('show');
grid on;
hold off;
