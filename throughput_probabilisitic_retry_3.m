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
SIFS = 10; % [μs]
DIFS = 34; % [μs]
backoff = 101.5; % 平均バックオフ制御時間 [μs]
backoff_2= 173.5;% 衝突時平均バックオフ制御時間 [μs]
max_distance = 1000; % 最大距離 [m]
N_max = max_distance / 50; % 最大端末数

num_trials = 100000; % 試行回数

p = zeros(size(num_trials)); % 乱数
ACK_t = zeros(size(Rmin)); % ACKフレーム[μs]
data_t = zeros(size(Rmin)); % データフレーム[μs]
d_max = zeros(size(Rmin)); % 各伝送レートでの最大送信距離(50mごと)[m]
N_through = zeros(size(Rmin)); % 各伝送レートでのスルー出来る最大の端末数

num_times_sent_CTR=0;%CTR方式で送信された回数
num_times_sent_CONV=0;%従来方式で送信された回数

num_times_sent_CTR_all=zeros(101, 1);%CTR方式で送信された回数(0%~100%)
num_times_sent_CONV_all=zeros(101, 1);%従来方式で送信された回数(0%~100%)

throughput_CONV_1000m_all = zeros(101, 1);
throughput_CTR_1000m_all = zeros(101, 1);

for k=0:100%誤り率
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

    throughput_1 = zeros(num_trials, N_max);  % 試行回数と距離数に対応するスループット配列を事前確保
    throughput_2 = zeros(1, N_max);           % 平均スループット格納用
    N_succsess = zeros(1, N_max);             % 距離配列の事前確保

    % パラメータ設定
    N = 0;
 
    for i = 1:num_trials
        N = 0;
        total_tt = 0;

        while N < N_max
            p = rand;
            num_times_sent_CTR=num_times_sent_CTR+1;%CTR方式で送信された回数をカウント
            % 端末数をカウント
            N = N + N_through(4);
            total_tt = total_tt + (ACK_t(4) + data_t(4) + SIFS +6+ backoff);

            if p < (k/100)
                N = max(0, N - 1);
                total_tt = total_tt + ACK_t(4);
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
    num_times_sent_CTR=num_times_sent_CTR/num_trials;%CTR方式で送信された回数の平均

    for j = 1:N_max
        non_zero_values = throughput_1(throughput_1(:, j) > 0, j);
        if ~isempty(non_zero_values)
            throughput_2(j) = mean(non_zero_values);
        else
            throughput_2(j) = 0;
        end
    end

    valid_indices = throughput_2 > 0;

    throughput_CTR_1000m = throughput_2(N_max); % CTR方式のスループット

    % 条件2: スループット計算 (固定距離リスト)_
    if mod(max_distance, d_max(4)) == 0
        distances = d_max(4):d_max(4):max_distance;
    else
        distances = [d_max(4):d_max(4):max_distance max_distance];
    end

    throughput_3 = zeros(num_trials, length(distances));
    throughput_4 = zeros(1, length(distances));

    for i = 1:num_trials
        
        for j = 1:length(distances)
            num_times_sent_CONV=num_times_sent_CONV+1;%従来方式で送信された回数をカウント
            p = rand;
            D = distances(j);
            if p < (k/100)
                total_tt = total_tt + (ACK_t(4) + data_t(4) + SIFS +6+ backoff) + (ACK_t(3) + data_t(3) + SIFS+6 + backoff_2);
                num_times_sent_CONV=num_times_sent_CONV+1;%再送した回数をカウント
            else
                total_tt = total_tt + (ACK_t(4) + data_t(4) + SIFS +6+ backoff);
            end
            throughput_3(i, j) = packet / total_tt;
        end
        total_tt = 0;
    end
    num_times_sent_CONV=num_times_sent_CONV/num_trials;%従来方式で送信された回数の平均
    for j = 1:length(distances)
        throughput_4(j) = mean(throughput_3(:, j));
    end

    throughput_CONV_1000m = throughput_4((N_max/N_through(4))); % 従来方式のスループット

    % スループットデータを保存
    num_times_sent_CTR_all(k+1)=num_times_sent_CTR;%誤り率k%の時のCTR方式で送信された平均回数を保存
    num_times_sent_CONV_all(k+1)=num_times_sent_CONV;%誤り率k%の時の従来方式で送信された平均回数を保存
    throughput_CONV_1000m_all(k+1) = throughput_CONV_1000m;%誤り率k%の時のCTR方式で送信された平均スループットを保存
    throughput_CTR_1000m_all(k+1) = throughput_CTR_1000m;%誤り率k%の時の従来方式で送信された平均スループットを保存
end

figure('Position', [100, 100, 800, 500]);

% Conventional scheme と CTR scheme のスループットをプロット
plot(0:100, throughput_CONV_1000m_all, 'k-', 'LineWidth', 2); % Conventional scheme (黒)
hold on;
plot(0:100, throughput_CTR_1000m_all, 'r-', 'LineWidth', 2); % CTR scheme (赤)

% 軸ラベルやタイトルを追加
xlabel('Error Rate [%]');
ylabel('Throughput [Mbps]', 'FontSize', 20, 'FontName', 'Times New Roman');

% 軸のフォントサイズ
set(gca, 'FontSize', 20, 'FontName', 'Times New Roman');
grid off; % グリッド表示
box on; % 枠表示

% 凡例を設定
legend( {'Conventional scheme', 'CTR scheme'}, 'FontSize', 20, 'FontName', 'Times New Roman', ...
    'Location', 'northeast', 'Box', 'off'); % 凡例の設定

    % y軸の範囲を0から最大値に設定
ylim([0 3]);

hold off;
figure('Position', [100, 100, 800, 500]);

% Conventional scheme と CTR scheme のスループットをプロット
plot(0:100, num_times_sent_CONV_all, 'k-', 'LineWidth', 2); % Conventional scheme (黒)
hold on;
plot(0:100, num_times_sent_CTR_all, 'r-', 'LineWidth', 2); % CTR scheme (赤)

% 軸ラベルやタイトルを追加
xlabel('Error Rate [%]');
ylabel('Transmission Count', 'FontSize', 20, 'FontName', 'Times New Roman');

% 軸のフォントサイズ
set(gca, 'FontSize', 20, 'FontName', 'Times New Roman');
grid off; % グリッド表示
box on; % 枠表示

% 凡例を設定
legend( {'Conventional scheme', 'CTR scheme'}, 'FontSize', 20, 'FontName', 'Times New Roman', ...
    'Location', 'northeast', 'Box', 'off'); % 凡例の設定

hold off;

