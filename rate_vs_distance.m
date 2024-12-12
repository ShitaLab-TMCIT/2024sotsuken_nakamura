%各伝送レートにおける最大伝送距離の計算
Rmin=[-82 -81 -79 -77 -74 -70 -66 -65]; % 最小受信感度[dBm]
Tp=10; % 送信電力[dBm]
f = 2.4 * 10^9;
c = 3 * 10^8;
d = zeros(size(Rmin));
Lfs=zeros(size(Rmin));
TR=[6 9 12 18 24 36 48 54];

for i=1:length(Rmin)
    Lfs(i)=Tp-Rmin(i);
    d(i)=((10^(Lfs(i)/20))*c)/(4*pi*f);
end

% プロット
figure;
hold on;
for i = 1:length(d)-1
    x = linspace(d(i), d(i+1), 100); % d(i)からd(i+1)までの範囲を100点で分割
    y = ones(size(x)) * TR(i); % 同じThroughputの値を取る
    plot(x, y, 'b'); % 青色でプロット
end

% 最後の区間もプロット
x = linspace(d(end), 0, 100); % d(end)から0までの範囲を100点で分割
y = ones(size(x)) * TR(end); % 54 Mbpsの値を取る
plot(x, y, 'b');

xlabel('d(m)');
ylabel('TR(Mbps)');
hold off;
