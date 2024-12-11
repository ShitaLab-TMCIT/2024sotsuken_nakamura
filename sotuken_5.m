% パラメータの設定
Rmin = [-82, -81, -79, -77, -74, -70, -66, -65]; % 最小受信感度 [dBm]
Tp = 10; % 送信電力 [dBm]
f = 2.4 * 10^9; % 周波数 [Hz]
c = 3 * 10^8; % 光速 [m/s]
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

% 変数の初期化
nt = zeros(size(TR)); % 総送信回数
d1 = zeros(size(TR)); % 各変調方式における伝送距離[m]
Lfs = zeros(size(TR)); % 距離減衰 [dB]
ACK_t = zeros(size(TR)); % ACKフレーム長[μs]
data_t = zeros(size(TR)); % データ長[μs]
throughput1 = zeros(size(TR)); % 端末間のスループット [Mbps]
throughput2 = zeros(size(TR)); % 全体のスループット [Mbps]
total_tt = zeros(size(TR)); % 1パケットの全体(100台)の送信時間[μs]
n = zeros(size(TR)); % 一度に通信できる端末数

distances = 100:100:100*N; % 距離の範囲
throughput_distance = zeros(length(distances), length(TR)); % 距離に対するスループット

for i = 1:length(TR)
    Lfs(i) = Tp - Rmin(i); % 距離減衰 [dB]
    d1(i) = ((10^(Lfs(i) / 20)) * c) / (4 * pi * f); % 伝送距離 [m]
    for j = 1:length(distances)
        d = distances(j);
        if d1(i) >= d
            % 総送信回数の計算
            if mod(N - 1, d) == 0
                nt(i) = (N - 1) / d;
            else
                nt(i) = floor((N - 1) / d) + 1;
            end

            % フレームの計算
            ACK_t(i) = PLCP_pre + (PLCPhead_sig + ceil((PLCPhead_ser + ACK + FCS + tail) / databit(i))) * 4;
            data_t(i) = PLCP_pre + (PLCPhead_sig + ceil((PLCPhead_ser + MAC + LLC + packet + FCS + tail) / databit(i))) * 4;

            % スループットの計算
            total_tt(i) = (ACK_t(i) + data_t(i) + SIFS + backoff + DIFS) * nt(i);
            throughput_distance(j, i) = packet / total_tt(i);
        else
            throughput_distance(j, i) = 0;
        end
    end
end

% 結果のプロット
figure;
hold on;
for i = 1:length(TR)
    plot(distances, throughput_distance(:, i), 'DisplayName', sprintf('%d Mbps', TR(i)));
end
xlabel('距離 [m]');
ylabel('スループット [Mbps]');
title('距離に対するスループット');
legend show;
grid on;
hold off;
