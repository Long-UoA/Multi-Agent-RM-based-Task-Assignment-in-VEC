%-----------------Bach Long Nguyen-----------------------------------------
%-------Grant-Funded Researcher at The University of Adelaide, Australia---
%------Multi-Agent Regret-Matching-based Task Assignment--------------------
%--------------in Vehicular Edge Computing (VEC)---------------------------
clc
clear all

% varying the number of requesting vehicles
load sumPayoff25vehicles.mat sumPayoff
sumDelayCost25vehicles=sumPayoff';

load sumPayoff50vehicles.mat sumPayoff
sumDelayCost50vehicles=sumPayoff';

load sumPayoff75vehicles.mat sumPayoff
sumDelayCost75vehicles=sumPayoff';

load sumPayoff100vehicles.mat sumPayoff
sumDelayCost100vehicles=sumPayoff';

figure
plot(sumDelayCost25vehicles(1,1:2000),'r-','LineWidth',2)
hold on
plot(sumDelayCost50vehicles(1,1:2000),'b-','LineWidth',2)
plot(sumDelayCost75vehicles(1,1:2000),'k-','LineWidth',2)
plot(sumDelayCost100vehicles(1,1:2000),'m-','LineWidth',2)

legend('25 requesting vehicles','50 requesting vehicles','75 requesting vehicles','100 requesting vehicles');
xlabel('Time step (s)');% with \lambda=0.5,',' and speeds of 90, 100 and 120 km/h in lanes \{1,4\}, \{2,5\} and \{3,6\}, respectively'})
ylabel('Sum of delay and cost')
set(gca,'FontSize',20)


load Fairness25vehicles.mat Fairness25vehicles

load Fairness50vehicles.mat Fairness50vehicles

load Fairness75vehicles.mat Fairness75vehicles

load Fairness100vehicles.mat Fairness100vehicles

mtrxSum=[Fairness25vehicles Fairness50vehicles Fairness75vehicles Fairness100vehicles];

figure

D=[25 50 75 100];

bar(D,mtrxSum)

% legend('\lambda=0.5','\lambda=0.99','\lambda=0.9999');
xlabel('Number of autonomous vehicles')
ylabel('J_F')
set(gca,'FontSize',20)

