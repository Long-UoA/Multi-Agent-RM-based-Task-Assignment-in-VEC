function R=DataRate(B,Rx,Ry,Vx,Vy)
% assume the data rate is constant
power=20; % dBm
% B=40*10^6; % Hz
dist=sqrt((Rx-Vx)^2+(Ry-Vy)^2);
pl=128.1+37.6*log10(dist/1000);
power=10^(power/10)/1000; % Watt
% power=200/1000; % Watt
pl=10^(pl/10); % 1/Watt
Nr=10^(-96/10)/1000; % Watt
R=B*log2(1+(power/pl)/(Nr^2)); % bps