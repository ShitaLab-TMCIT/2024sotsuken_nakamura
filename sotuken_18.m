clear;
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
slottime = 9; % ショートスロットタイム[μs]

max_distance = 1000; % 最大距離 [m]
N_max = 20; % 最大端末数

% 選択された伝送レート (TR = 18 Mbps) に対応するインデックスを取得
index = find(TR == 18);
if isempty(index)
    error('指定された伝送レートが見つかりません');
end

% 必要な値を抽出
Rmin_current = Rmin(index);
TR_current = TR(index);
databit_current = databit(index);

% 最大伝送距離の計算
Lfs = Tp - Rmin_current; % 距離減衰 [dB]
d_max = floor(((10^(Lfs / 20)) * c) / (4 * pi * f) / 50) * 50; % 最大伝送距離 [m]
N_skip = d_max / 50;

% フレームの計算
ACK_t = PLCP_pre + (PLCPhead_sig + ceil((PLCPhead_ser + ACK + FCS + tail) / databit_current)) * 4;
data_t = PLCP_pre + (PLCPhead_sig + ceil((PLCPhead_ser + MAC + LLC + packet + FCS + tail) / databit_current)) * 4;

% 条件ごとのスループット計算
conditions = [1, 2, 3]; % 各条件での変更量
colors = ['r', 'g', 'b']; % プロットの色

figure;
hold on;
for cond_idx = 1:length(conditions)
    N = 0;
    throughput = [];
    N_succsess = [];
    total_tt = 0;
    j = 1;

    while N <= N_max
        N = N + N_skip;

        total_tt = total_tt + (ACK_t + data_t + SIFS + backoff); % トータル時間
        N = N - conditions(cond_idx); % 条件ごとの端末減少
        total_tt = total_tt + conditions(cond_idx) * slottime + ACK_t; % 通信成功

        if N >= N_max
            throughput(j) = packet / total_tt; % スループット [Mbps]
            N_succsess(j) = N_max * 50; % 距離[m]を記録
            break;
        end

        throughput(j) = packet / total_tt; % スループット [Mbps]
        N_succsess(j) = N * 50; % 距離[m]を記録
        j = j + 1;
    end

    % プロット
    plot(N_succsess, throughput, '-o', 'Color', colors(cond_idx), 'LineWidth', 1, 'MarkerSize', 4, 'DisplayName', sprintf('条件 %d', cond_idx));
end

xlabel('距離 [m]');
ylabel('スループット [Mbps]');
title('TR = 18 Mbps におけるスループット計算');
legend('show');
grid on;
hold off;

