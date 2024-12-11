clear
% パラメータの設定
Rmin = [-82, -81, -79, -77, -74, -70, -66, -65]; % 最小受信感度 [dBm]
Tp = 0; % 送信電力 [dBm]
f = 2.4 * 10^9; % 周波数 [Hz]
c = 3 * 10^8; % 光速 [m/s]



d = input('距離[m]: '); % 距離 [m]
N = input('端末数: '); % 端末数

% 配列の初期化
SNRmin = zeros(size(Rmin));
R = zeros(N-1, 1);
SNR = zeros(N-2, 1);

% Throughputの設定
Throughput = [6.5, 13, 19.5, 26, 39, 52, 58.5, 65]; % Mbps

% 受信電力の計算
for i = 1:N-1
    R(i) = Tp - 20 * log10(4 * pi * i * d / (c / f)); % [dBm]
end

% SNRの計算
for i = 1:N-2
    SNR(i) = R(i) - R(N-1); % SNR = 所望信号-雑音信号
end

% SNRminの計算
for i = 1:length(Rmin)
    SNRmin(i) = Rmin(i) + 85; 
end


results = zeros(N-2, 1);

% SNRとSNRminの比較とThroughputの決定
for i = 1:N-2
    for j = 1:length(SNRmin)
        if SNR(i) > SNRmin(j)
            if j == 1
                results(i) = Throughput(1); % 最初のしきい値より大きければ最初のスループット
            end
        else
            if j == 1
                results(i) = 0; % すべてのスループットが条件を満たさない場合
            else
                results(i) = Throughput(j-1); 
            end
            break; 
        end
    end
end

% 結果の表示
disp('受信電力[dBm]:');
disp(R);
disp('SNR[dB]:');
disp(SNR);
disp('スループット[Mbps]:');
disp(results);

