clear;
Rmin = [-82, -81, -79, -77, -74, -70, -66, -65]; % 最小受信感度 [dBm]
TR = [6, 9, 12, 18, 24, 36, 48, 54]; % 伝送レート [Mbps]
databit = [24, 36, 48, 72, 96, 144, 192, 216]; % OFDMシンボルごとのデータビット[bit]
error_rate = [0.03, 0.1, 0.2, 0.5]; % 通信が失敗する確率
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
max_distance = 1000; % 最大距離 [m]
N_max = max_distance / 50; % 最大端末数

num_trials = 10000; % 試行回数

p = zeros(size(num_trials)); % 乱数
ACK_t = zeros(size(Rmin)); % ACKフレーム[μs]
data_t = zeros(size(Rmin)); % データフレーム[μs]
d_max = zeros(size(Rmin)); % 各伝送レートでの最大送信距離(50mごと)[m]
N_through = zeros(size(Rmin)); % 各伝送レートでのスルー出来る最大の端末数

throughput_CONV_1000m_all = zeros(length(error_rate), 1);
throughput_CTR_1000m_all = zeros(length(error_rate), 1);

for k=1:length(error_rate)
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

            % 端末数をカウント
            N = N + N_through(4);
            total_tt = total_tt + (ACK_t(4) + data_t(4) + SIFS + backoff);

            if p < error_rate(k)
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

    for j = 1:N_max
        non_zero_values = throughput_1(throughput_1(:, j) > 0, j);
        if ~isempty(non_zero_values)
            throughput_2(j) = mean(non_zero_values);
        else
            throughput_2(j) = 0;
        end
    end

    valid_indices = throughput_2 > 0;

    throughput_CTR_1000m = throughput_2(N_max); % CTRスキームのスループット

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
            p = rand;
            D = distances(j);
            if p < error_rate(k)
                total_tt = total_tt + (ACK_t(4) + data_t(4) + SIFS + backoff) + (ACK_t(3) + data_t(3) + SIFS + backoff);
            else
                total_tt = total_tt + (ACK_t(4) + data_t(4) + SIFS + backoff);
            end
            throughput_3(i, j) = packet / total_tt;
        end
        total_tt = 0;
    end

    for j = 1:length(distances)
        throughput_4(j) = mean(throughput_3(:, j));
    end

    throughput_CONV_1000m = throughput_4((N_max/N_through(4))); % 従来スキームのスループット

    % スループットデータを保存
    throughput_CONV_1000m_all(k) = throughput_CONV_1000m;
    throughput_CTR_1000m_all(k) = throughput_CTR_1000m;
end
figure('Position', [100, 100, 700, 500]);

% データとラベルを準備
bar_data = [throughput_CONV_1000m_all, throughput_CTR_1000m_all]; % データ
bar_labels = categorical({'3', '10', '20', '50'}); % エラーレートラベル

% バーを描画
h = bar(bar_data, 0.9); 
set(gca, 'XTickLabel', bar_labels, 'XTick', 1:length(bar_labels)); % X軸のラベルを設定

% バーの色を設定
h(1).FaceColor = 'k'; % 従来方式 (黒)
h(2).FaceColor = [0.8, 0.2, 0.2]; % CTR方式 (赤)

% 軸ラベルやタイトルを追加
xlabel('Error Rate [%]');
ylabel('Throughput [Mbps]', 'FontSize', 20, 'FontName', 'Times New Roman');

% 軸のフォントサイズ
set(gca, 'FontSize', 16, 'FontName', 'Times New Roman');
grid on; % グリッド表示
box on; % 枠表示

% 凡例を設定
legend(h, {'Conventional scheme', 'CTR scheme'}, 'FontSize', 20, 'FontName', 'Times New Roman', ...
    'Location', 'northeast', 'Box', 'off'); % 凡例の設定

hold off;
