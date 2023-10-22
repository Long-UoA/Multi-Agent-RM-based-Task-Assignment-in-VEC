%-----------------Bach Long Nguyen-----------------------------------------
%-------Grant-Funded Researcher at The University of Adelaide, Australia---
%------Multi-Agent Regret-Matching-based Task Assignment--------------------
%--------------in Vehicular Edge Computing (VEC)---------------------------
clc
clear all

% varying the lambda
load sumPayoffLambda_05.mat sumPayoff
sumDelayCostLambda_05=sumPayoff';

load sumPayoffLambda_099.mat sumPayoff
sumDelayCostLambda_099=sumPayoff';

load sumPayoffLambda_09999.mat sumPayoff
sumDelayCostLambda_09999=sumPayoff';

figure
plot(sumDelayCostLambda_05(1,1:2000),'m-','LineWidth',2)
hold on
plot(sumDelayCostLambda_099(1,1:2000),'b-','LineWidth',2)
plot(sumDelayCostLambda_09999(1,1:2000),'k-','LineWidth',2)

legend('\lambda=0.5','\lambda=0.99','\lambda=0.9999');
xlabel('Time step (s)')
ylabel('Sum of delay and cost')
set(gca,'FontSize',20)

% varying the lambda
load sumPayoffDelayLambda_05.mat sumPayoffDelay
sumDelayLambda_05=sumPayoffDelay';

load sumPayoffDelayLambda_099.mat sumPayoffDelay
sumDelayLambda_099=sumPayoffDelay';

load sumPayoffDelayLambda_09999.mat sumPayoffDelay
sumDelayLambda_09999=sumPayoffDelay';

figure
plot(sumDelayLambda_05(1,1:2000),'m-','LineWidth',2)
hold on
plot(sumDelayLambda_099(1,1:2000),'b-','LineWidth',2)
plot(sumDelayLambda_09999(1,1:2000),'k-','LineWidth',2)

legend('\lambda=0.5','\lambda=0.99','\lambda=0.9999');
xlabel('Time step (s)')
ylabel('Sum of delay')
set(gca,'FontSize',20)

% varying the lambda
load sumPayoffCostLambda_05.mat sumPayoffCost
sumCostLambda_05=sumPayoffCost';

load sumPayoffCostLambda_099.mat sumPayoffCost
sumCostLambda_099=sumPayoffCost';

load sumPayoffCostLambda_09999.mat sumPayoffCost
sumCostLambda_09999=sumPayoffCost';

figure
plot(sumCostLambda_05(1,1:2000),'m-','LineWidth',2)
hold on
plot(sumCostLambda_099(1,1:2000),'b-','LineWidth',2)
plot(sumCostLambda_09999(1,1:2000),'k-','LineWidth',2)

legend('\lambda=0.5','\lambda=0.99','\lambda=0.9999');
xlabel('Time step (s)')
ylabel('Sum of cost')
set(gca,'FontSize',20)


