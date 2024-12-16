%従来のプロトコル(18Mbps→12Mbps)で再送したときと中継し伝送したときのスループットの比較
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
error_rate=0.03;%再送率[%]
num_trials = 100;%試行回数

p= zeros(size(num_trials));%乱数
ACK_t=zeros(size(Rmin));%ACKフレーム[μs]
data_t=zeros(size(Rmin));%データフレーム[μs]
d_max=zeros(size(Rmin));%各伝送レートでの最大送信距離(50mごと)[m]
N_skip=zeros(size(Rmin));%各伝送レートでのスルー出来る最大の端末数

for i = 1:length(Rmin)
    Rmin_current = Rmin(i);
    TR_current = TR(i);
    databit_current = databit(i);
    
    % 最大伝送距離の計算
    Lfs = Tp - Rmin_current; % 距離減衰 [dB]
    d_max(i) = floor(((10^(Lfs / 20)) * c) / (4 * pi * f) / 50) * 50; % 最大伝送距離 [m]
    N_skip(i) = d_max(i) / 50;
    % フレームの計算
    ACK_t(i) = PLCP_pre + (PLCPhead_sig + ceil((PLCPhead_ser + ACK + FCS + tail) / databit_current)) * 4;
    data_t (i)= PLCP_pre + (PLCPhead_sig + ceil((PLCPhead_ser + MAC + LLC + packet + FCS + tail) / databit_current)) * 4;
end


figure;
hold on;


    N = 0;
    
    total_tt = 0;
    j = 1;

    while N <= N_max
        N = N + N_skip(4);
        total_tt = total_tt + (ACK_t(4) + data_t (4)+ SIFS + backoff); % トータル時間
        N = N - 1; % 条件ごとの端末減少
        total_tt = total_tt +  slottime +  ACK_t(4); % 通信成功

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
    plot(N_succsess_cond, throughput_cond, '-o', 'Color', 'r', 'LineWidth', 1, 'MarkerSize', 4, 'DisplayName', sprintf('%dつ手前の端末で中継', 1));
    

    % 条件2: スループット計算 (固定距離リスト)
    if mod(max_distance, d_max(4)) == 0
        distances = d_max(4):d_max(4):max_distance; 
    else
        distances = [d_max(4):d_max(4):max_distance max_distance];
    end
  
    
    throughput_2 = zeros(num_trials, length(distances)); % スループット_2 の事前割り当て
    throughput_3 = zeros(1, length(distances)); % スループット_3 の事前割り当て
    
    for i = 1:num_trials
        p = rand; % ランダムな確率を生成
        for j = 1:length(distances)
            D = distances(j);
            if p < error_rate
                % 再送が発生する場合 (最初は18 Mbps、再送は12 Mbps)
                total_tt = (ACK_t(4) + data_t(4) + SIFS + backoff) * (D / d_max(4)) + ...
                          (ACK_t(3) + data_t(3) + SIFS + backoff) * (D / d_max(3));
            else
                 % 再送が発生しない場合 (18 Mbps)
                 total_tt = (ACK_t(4) + data_t(4) + SIFS + backoff) * (D / d_max(4));
               
            end
            throughput_2(i, j) = packet / total_tt; % スループットを計算 [Mbps]
        end
    end
    
    disp(throughput_2);
    disp(size(throughput_2));
    
    % 各距離に対する平均スループットを計算
    for j = 1:length(distances)
        throughput_3(j) = mean(throughput_2(:, j));
    end
    
    % 結果をプロット
    figure;
    plot(distances, throughput_3, '-o', 'Color', 'k', 'LineWidth', 1, 'MarkerSize', 4, 'DisplayName', '従来のプロトコルで再送');
    xlabel('距離 [m]');
    ylabel('スループット [Mbps]');
    title('距離とスループットの関係');
    grid on;
    legend show;
    