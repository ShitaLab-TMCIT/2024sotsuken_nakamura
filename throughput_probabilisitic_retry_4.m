clear;
Rmin = [-82, -81, -79, -77, -74, -70, -66, -65]; % 最小受信感度 [dBm]
TR = [6, 9, 12, 18, 24, 36, 48, 54]; % 伝送レート [Mbps]
databit = [24, 36, 48, 72, 96, 144, 192, 216]; % OFDMシンボルごとのデータビット[bit]

Tp = 10; % 送信電力 [dBm]
f = 2.4 * 10^9; % 周波数 [Hz]
c = 3 * 10^8; % 光速 [m/s]
rng(0);
PLCP_pre = 16; % PLCPプリアンブル[μs]
PLCPhead_sig = 1; % PLCPヘッダ（シグナル）[μs]
PLCPhead_ser = 16; % PLCPヘッダ（サービス）[μs]
ACK = 80; % 802.11ACKフレーム[bit]
MAC = 192; % 802.11MACヘッダ[bit]
LLC = 64; % LLCヘッダ[bit]
packet = 12000; % IPパケット長[bit]
FCS = 32; % FCS[bit]
tail = 6; % テイルビット[bit]
SIFS = 16; % [μs]
DIFS = 34; % [μs]
backoff = 101.5; % 平均バックオフ制御時間 [μs]
max_distance = 1000; % 最大距離 [m]
N_max = max_distance / 50; % 最大端末数

num_trials = 100000; % 試行回数

p = zeros(size(num_trials)); % 乱数
ACK_t = zeros(size(Rmin)); % ACKフレーム[μs]
data_t = zeros(size(Rmin)); % データフレーム[μs]
d_max = zeros(size(Rmin)); % 各伝送レートでの最大送信距離(50mごと)[m]
N_through = zeros(size(Rmin)); % 各伝送レートでのスルー出来る最大の端末数

num_times_sent_CTR = 0; % CTR方式で送信された回数
num_times_sent_CONV = 0; % 従来方式で送信された回数

throughput_CTR_all = zeros(101, 1); % CTR方式でのスループット
throughput_CONV_all = zeros(101, 1); % 従来方式でのスループット

% 誤り率k1を設定
k1 = 0.05; % 最初の誤りの確率（例：5%）

for k2 = 0:0.01:1  % k2を0から1まで0.01刻みで変化させる
    throughput_1 = zeros(num_trials, N_max);  % 試行回数と距離数に対応するスループット配列を事前確保
    throughput_2 = zeros(1, N_max);           % 平均スループット格納用
    N_succsess = zeros(1, N_max);             % 距離配列の事前確保

    % 最大伝送距離の計算
    for i = 1:length(Rmin)
        Rmin_current = Rmin(i);
        TR_current = TR(i);
        databit_current = databit(i);

        % 最大伝送距離の計算
        Lfs = Tp - Rmin_current; % 距離減衰 [dB]
        d_max(i) = floor(((10^(Lfs / 20)) * c) / (4 * pi * f) / 50) * 50; % 最大伝送距離 [m]
        N_through(i) = d_max(i) / 50;

        % フレームの計算
        ACK_t(i) = PLCP_pre + (PLCPhead_sig + ceil((PLCPhead_ser + ACK + FCS + tail) / databit_current)) * 4;
        data_t(i) = PLCP_pre + (PLCPhead_sig + ceil((PLCPhead_ser + MAC + LLC + packet + FCS + tail) / databit_current)) * 4;
    end

    for i = 1:num_trials
        N = 0;
        total_tt = 0;

        while N < N_max
            p = rand; % 通常の確率p
            num_times_sent_CTR = num_times_sent_CTR + 1; % CTR方式で送信された回数をカウント
            N = N + N_through(4); % 端末数をカウント
            total_tt = total_tt + (ACK_t(4) + data_t(4) + SIFS + backoff);

            % 最初の誤りが発生する場合 (k1を使用)
            if p < k1
                N = max(0, N - 1); % 端末数が1減少
                total_tt = total_tt + ACK_t(4); % ACKの送信時間
            end

            % 2回目の誤りが発生する場合 (k2を使用)
            p2 = rand; % 次の誤り確率
            if p2 < k2
                N = max(0, N - 1); % 端末数が再度1減少
                total_tt = total_tt + ACK_t(4); % ACKの送信時間
            end

            if N >= N_max
                throughput_1(i, N_max) = packet / total_tt;
                N_succsess(N_max) = N_max * 50;
                break;
            else
                throughput_1(i, N) = packet / total_tt;
                N_succsess(N) = N * 50;
            end
        end
    end

    % k2の変更ごとに必要な処理を行う（例えば、スループットを格納するなど）
    throughput_2 = mean(throughput_1, 1); % 各k2に対する平均スループット
    throughput_CTR_all(round(k2*100) + 1) = throughput_2(N_max); % k2に対応するスループットを格納
end

% スループットのプロット
figure('Position', [100, 100, 800, 500]);

plot(0:0.01:1, throughput_CTR_all, 'r-', 'LineWidth', 2); % CTR方式のスループット

xlabel('Error Rate k2 [%]');
ylabel('Throughput [Mbps]', 'FontSize', 20, 'FontName', 'Times New Roman');
set(gca, 'FontSize', 16, 'FontName', 'Times New Roman');
grid off; % グリッド表示
box on; % 枠表示

legend({'CTR scheme'}, 'FontSize', 20, 'FontName', 'Times New Roman', 'Location', 'northeast', 'Box', 'off');
ylim([0 3]);

hold off;
