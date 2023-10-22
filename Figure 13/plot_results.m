%-----------------Bach Long Nguyen-----------------------------------------
%-------Grant-Funded Researcher at The University of Adelaide, Australia---
%------Multi-Agent Regret-Matching-based Task Assignment--------------------
%--------------in Vehicular Edge Computing (VEC)---------------------------
clc
clear all

% varying the inter-RSU distance
load sumPayoffD3000.mat sumPayoff
sumDelayCostD3000=min(sumPayoff(:,1));

load sumPayoffD6000.mat sumPayoff
sumDelayCostD6000=min(sumPayoff(:,1));

load sumPayoffD9000.mat sumPayoff
sumDelayCostD9000=min(sumPayoff(:,1));

mtrxSum=[round(sumDelayCostD3000,1) sumDelayCostD6000 sumDelayCostD9000];

figure

D=[3 6 9];

bar(D,mtrxSum)

% legend('\lambda=0.5','\lambda=0.99','\lambda=0.9999');
xlabel('Inter-RSU Distance (D_R) (km)')
ylabel('Sum of delay and cost')
set(gca,'FontSize',20)


