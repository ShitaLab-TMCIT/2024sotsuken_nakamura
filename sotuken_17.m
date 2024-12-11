% Git test
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

% 条件1: スループット計算 (条件ごとのスループット)
conditions = [1, 2, 3]; % 各条件での変更量
colors = ['r', 'g', 'b'];

figure;
hold on;
for cond_idx = 1:length(conditions)
    N = 0;
    throughput_cond = [];
    N_succsess_cond = [];
    total_tt = 0;
    j = 1;

    while N <= N_max
        N = N + N_skip;
        total_tt = total_tt + (ACK_t + data_t + SIFS + backoff); % トータル時間
        N = N - conditions(cond_idx); % 条件ごとの端末減少
        total_tt = total_tt + conditions(cond_idx) * slottime + ACK_t; % 通信成功

        if N >= N_max
            throughput_cond(j) = packet / total_tt; % スループット [Mbps]
            N_succsess_cond(j) = N_max * 50; % 距離[m]を記録
            break;
        end

        throughput_cond(j) = packet / total_tt; % スループット [Mbps]
        N_succsess_cond(j) = N * 50; % 距離[m]を記録
        j = j + 1;
    end

    % プロット
    plot(N_succsess_cond, throughput_cond, '-o', 'Color', colors(cond_idx), 'LineWidth', 1, 'MarkerSize', 4, 'DisplayName', sprintf('%dつ手前の端末で中継', cond_idx));
end

% 条件2: スループット計算 (固定距離リスト)
if mod(max_distance, d_max) == 0
    distances = d_max:d_max:max_distance; 
else
    distances = [d_max:d_max:max_distance max_distance];
end

throughput_fixed = zeros(size(distances));
for j = 1:length(distances)
    D = distances(j);
    total_tt = (ACK_t + data_t + SIFS + backoff) * (D / d_max) * 2; % トータル時間
    throughput_fixed(j) = packet / total_tt; % スループット [Mbps]
end

% プロット (固定距離リスト)
plot(distances, throughput_fixed, '-o', 'Color', 'k', 'LineWidth', 1, 'MarkerSize', 4, 'DisplayName', '従来のプロトコルで再送');

xlabel('距離 [m]');
ylabel('スループット [Mbps]');
title('TR = 18 Mbps におけるスループット計算');
legend('show');
grid on;
hold off;
