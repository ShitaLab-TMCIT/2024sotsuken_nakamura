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
N=0;%今の端末
N_succsess=zeros(size(N_max));%通信成功した端末の配列
N_max=20;%最大端末数
max_distance = 1000; % 最大距離 [m]
P=1;
slottime=9;%ショートスロットタイム[μs]



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
N_skip=d_max/50;


% フレームの計算
ACK_t = PLCP_pre + (PLCPhead_sig + ceil((PLCPhead_ser + ACK + FCS + tail) / databit_current)) * 4;
data_t = PLCP_pre + (PLCPhead_sig + ceil((PLCPhead_ser + MAC + LLC + packet + FCS + tail) / databit_current)) * 4;



% スループット計算
throughput = zeros(size(N_max));
j=0;
while N==N_max
  
      N=N+N_skip;
      
        if P==1 %通信が成功した場合
          
            total_tt = total_tt +(ACK_t + data_t + SIFS + backoff); % トータル時間
           
            throughput(j) = packet / total_tt; % スループット [Mbps]
            
        else                %通信が失敗した場合
            total_tt = total_tt +(ACK_t + data_t + SIFS + backoff)+slottime+ACK_t; % トータル時間

              
         
        end
        N_succsess(j)=N;
        j=j+1;

end

% スループットのプロット
figure;
plot(N_succsess, throughput, '-o', 'LineWidth', 1, 'MarkerSize', 4, 'DisplayName', sprintf('伝送レート: %d Mbps', TR_current));
xlabel('距離 [m]');
ylabel('スループット [Mbps]');
title('TR = 18 Mbps におけるスループット計算');
legend('show');
grid on;
