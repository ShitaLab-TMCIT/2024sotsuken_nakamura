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
error_rate=0.03;%通信が失敗する確率
num_trials = 100;%試行回数

p= zeros(size(num_trials));%乱数
ACK_t=zeros(size(Rmin));%ACKフレーム[μs]
data_t=zeros(size(Rmin));%データフレーム[μs]
d_max=zeros(size(Rmin));%各伝送レートでの最大送信距離(50mごと)[m]
N_through=zeros(size(Rmin));%各伝送レートでのスルー出来る最大の端末数


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
    data_t (i)= PLCP_pre + (PLCPhead_sig + ceil((PLCPhead_ser + MAC + LLC + packet + FCS + tail) / databit_current)) * 4;
end


throughput_1 = zeros(num_trials, N_max);  % 試行回数と距離数に対応するスループット配列を事前確保
throughput_2 = zeros(1, N_max);           % 平均スループット格納用
N_succsess = zeros(1, N_max);             % 距離配列の事前確保



% パラメータ設定
N = 0;                                    % 現在の端末数
                   

for i = 1:num_trials  % 試行回数分ループ
    N = 0;                               % 各試行の開始時にNをリセット
    total_tt = 0;                        % 総伝送時間もリセット
    
    while N < N_max
        p = rand;  % 確率を生成
        
        % 端末数をカウント
        N = N + N_through(4);  
        total_tt = total_tt + (ACK_t(4) + data_t(4) + SIFS + backoff); % 総伝送時間の計算

        if p < error_rate  % パケットが失敗した場合
            N = max(0, N - 1);  % 1つ手前に戻る
            total_tt = total_tt + slottime + ACK_t(4); % 追加時間を加算
        end
        
        % スループット記録および距離の記録
        if N >= N_max
            throughput_1(i, N_max) = packet / total_tt; % 最大距離におけるスループット
            N_succsess(N_max) = N_max * 50;  % 最大距離を格納
            break;
        else
            throughput_1(i, N) = packet / total_tt; % 現在のNにおけるスループット記録
            N_succsess(N) = N * 50;  % 距離の記録
        end
    end
end
disp(throughput_1);
disp(size(throughput_1));

% 各距離での平均スループット計算
for j = 1:N_max
    % 0以外の要素のみを選んで平均を取る
    non_zero_values = throughput_1(throughput_1(:, j) > 0, j); % 0以外の要素を抽出
    if ~isempty(non_zero_values)  % 抽出後にデータが存在する場合のみ平均計算
        throughput_2(j) = mean(non_zero_values);
    else
        throughput_2(j) = 0;  % 0以外の要素がない場合は0
    end
end

% スループットが0のデータポイントを除外
valid_indices = throughput_2 > 0;  % 0より大きいスループットのみを抽出





      total_tt = 0;
    % 条件2: スループット計算 (固定距離リスト)
    if mod(max_distance, d_max(4)) == 0
        distances = d_max(4):d_max(4):max_distance; 
    else
        distances = [d_max(4):d_max(4):max_distance max_distance];
    end
  
    
    throughput_3= zeros(num_trials, length(distances)); % スループット_2 の事前割り当て
    throughput_4= zeros(1, length(distances)); % スループット_3 の事前割り当て
    
    for i = 1:num_trials
        
        for j = 1:length(distances)
            p = rand; % ランダムな確率を生成
            D = distances(j);
            if p < error_rate
                % 再送が発生する場合 (最初は18 Mbps、再送は12 Mbps)
                total_tt =total_tt+ (ACK_t(4) + data_t(4) + SIFS + backoff)+(ACK_t(3) + data_t(3) + SIFS + backoff) ;
            else
                 % 再送が発生しない場合 (18 Mbps)
                 total_tt =total_tt+ (ACK_t(4) + data_t(4) + SIFS + backoff) ;
               
            end
            throughput_3(i, j) = packet / total_tt; % スループットを計算 [Mbps]
        end
        total_tt = 0;
    end
    
    disp(throughput_3);
    disp(size(throughput_3));
    
    % 各距離に対する平均スループットを計算
    for j = 1:length(distances)
        throughput_4(j) = mean(throughput_3(:, j));
    end
    
  % 結果をプロット
figure;
hold on;

% グラフ1: 初めのスループットデータ
plot(N_succsess(valid_indices), throughput_2(valid_indices), '-o', ...
    'Color', 'r', 'LineWidth', 1, 'MarkerSize', 4, ...
    'DisplayName', sprintf('%dつ手前の端末で中継', 1));

% グラフ2: 距離ごとのスループットデータ
plot(distances, throughput_4, '-o', ...
    'Color', 'k', 'LineWidth', 1, 'MarkerSize', 4, ...
    'DisplayName', '従来のプロトコルで再送');

% ラベルとタイトル設定
xlabel('距離 [m]');
ylabel('スループット [Mbps]');
title('距離とスループットの関係');
grid on;
legend show;
hold off;