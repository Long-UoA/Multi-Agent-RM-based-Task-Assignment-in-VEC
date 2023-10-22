%-----------------Bach Long Nguyen-----------------------------------------
%-------Grant-Funded Researcher at The University of Adelaide, Australia---
%------Multi-Agent Regret-Matching-based Task Assignment--------------------
%--------------in Vehicular Edge Computing (VEC)---------------------------
clc
clear all

% varying the speeds of requesting vehicles
load sumPayoffSpeeds_11_14_17.mat sumPayoff
sumDelayCostSpeed_11_14_17=sumPayoff';

load sumPayoffSpeeds_25_30_33.mat sumPayoff
sumDelayCostSpeed_25_30_33=sumPayoff';

figure
plot(sumDelayCostSpeed_11_14_17(1,1:2000),'m-','LineWidth',2)
hold on
plot(sumDelayCostSpeed_25_30_33(1,1:2000),'b-','LineWidth',2)

l=legend('Speeds in lanes 1, 2 and 3 are 40, 50 and 60 km/h, respectively',...
    'Speeds in lanes 1, 2 and 3 are 90, 100 and 120 km/h, respectively');
l.FontSize=16;
xlabel('Time step (s)')
ylabel('Sum of delay and cost')
set(gca,'FontSize',20)
