%-----------------Bach Long Nguyen-----------------------------------------
%-------Grant-Funded Researcher at The University of Adelaide, Australia---
%------Multi-Agent Regret-Matching-based Task Assignment--------------------
%--------------in Vehicular Edge Computing (VEC)---------------------------

clc
clear all
% N = 400;          % Number of interactions                        t = 1..N
%=================Simulation Deployment====================================
numIter=1;
sumRewardIter=cell(1,numIter);

for iIter=1:numIter

    rng(0);

    simTime=60;
    D=3000;
    meanR=600;
    R=poissrnd(meanR);

    veL14=25;
    veL25=30;
    veL36=33;

    nL1=1;
    nL2=2;
    nL3=4;
    nL4=6;
    nL5=8;

    tLmt=75;

    numReq=10;

    nIteration=1; % number of iterations
    %-----------------------RSU location---------------------------------------
    nodesX(1)=1000;
    nodesY(1)=1000;
    nodesX(2)=nodesX(1)+D;
    nodesY(2)=1000;
    %---------------------BS location--------------------------------------
    nodesX0=nodesX(1)+D/2;
    nodesY0=1000;
    %----------------------------------------------------------------------
%     plot(nodesX0,nodesY0,'k^','MarkerSize',20,'MarkerFaceColor','k')
%     hold on
%     grid on
%     line([nodesX(1)-R-(D-2*R),nodesX(2)+R+D-2*R],[1040,1040])
%     line([nodesX(1)-R-(D-2*R),nodesX(2)+R+D-2*R],[1030,1030])
%     line([nodesX(1)-R-(D-2*R),nodesX(2)+R+D-2*R],[1020,1020])
%     line([nodesX(1)-R-(D-2*R),nodesX(2)+R+D-2*R],[1010,1010])
%     line([nodesX(1)-R-(D-2*R),nodesX(2)+R+D-2*R],[990,990])
%     line([nodesX(1)-R-(D-2*R),nodesX(2)+R+D-2*R],[980,980])
%     line([nodesX(1)-R-(D-2*R),nodesX(2)+R+D-2*R],[970,970])
%     line([nodesX(1)-R-(D-2*R),nodesX(2)+R+D-2*R],[960,960])   

    posRSU=cell(1,length(nodesX));
    for i=1:length(posRSU)
       posRSU{1,i}(1,1)=nodesX(1,i);
       posRSU{1,i}(1,2)=nodesY(1,i);
    end

%     plot(nodesX,nodesY,'b^','MarkerSize',20,'MarkerFaceColor','b') 
%     viscircles([nodesX(1),nodesY(1)],R,'LineStyle','--','Color','b')    
%     viscircles([nodesX(2),nodesY(2)],R,'LineStyle','--','Color','b')
% 
%     RBS=nodesX0-nodesX(1)+R+D-2*R;
% 
%     viscircles([nodesX0,nodesY0],RBS,'LineStyle','--','Color','k')
    %----------------------------------------------------------------------
    interDist=[500 1000];
    posX_target=zeros(1,numReq);
    posY_target=zeros(1,numReq);
    vTarget=zeros(1,numReq);
    initY=nodesY(1);
    for i=1:numReq
        %------------------------------------------------------------------
        % lane 1
        if i>0&&i<=nL1
            if i==0+1
                posX_target(1,i)=nodesX(1)-(D-2*R);
            else
                posX_target(1,i)=posX_target(1,i-1)-randi(interDist);
            end
            posY_target(1,i)=initY+35;
            vTarget(1,i)=veL14;
        end
        %----------------------------------------------------------------------
        % lane 2
        if i>nL1&&i<=nL2
            if i==nL1+1
                posX_target(1,i)=nodesX(1)-(D-2*R);
            else
                posX_target(1,i)=posX_target(1,i-1)-randi(interDist);
            end
            posY_target(1,i)=initY+25;
            vTarget(1,i)=veL25;
        end
        %----------------------------------------------------------------------
        % lane 3
        if i>nL2&&i<=nL3
            if i==nL2+1
                posX_target(1,i)=nodesX(1)-(D-2*R);
            else
                posX_target(1,i)=posX_target(1,i-1)-randi(interDist);
            end
            posY_target(1,i)=initY+15;
            vTarget(1,i)=veL36;
        end
        %----------------------------------------------------------------------
        % lane 4
        if i>nL3&&i<=nL4
            if i==nL3+1
                posX_target(1,i)=nodesX(2)+(D-2*R);
            else
                posX_target(1,i)=posX_target(1,i-1)+randi(interDist);
            end
            posY_target(1,i)=initY-35;
            vTarget(1,i)=veL14;
        end    
        %----------------------------------------------------------------------
        % lane 5
        if i>nL4&&i<=nL5
            if i==nL4+1
                posX_target(1,i)=nodesX(2)+(D-2*R);
            else
                posX_target(1,i)=posX_target(1,i-1)+randi(interDist);
            end
            posY_target(1,i)=initY-25;
            vTarget(1,i)=veL25;
        end     
        %----------------------------------------------------------------------
        % lane 6
        if i>nL5&&i<=numReq
            if i==nL5+1
                posX_target(1,i)=nodesX(2)+(D-2*R);
            else
                posX_target(1,i)=posX_target(1,i-1)+randi(interDist);
            end
            posY_target(1,i)=initY-15;
            vTarget(1,i)=veL36;
        end      
    end
    %----------------------------------------------------------------------
    posRqs=cell(1,numReq);
    for i=1:length(posRqs)
        posRqs{1,i}(1,1)=posX_target(1,i);
        posRqs{1,i}(1,2)=posY_target(1,i);
    end
    %-------------------------------------------------------------------------- 

    N=nIteration; 

    nRSU=length(posRSU); % a number of RSUs;
    nVehicle=length(posRqs); % number of vehicles = number of tasks
    nameAVe=cell(1,nVehicle); % name of each action of each requesting vehicle
    probActVehicle=cell(1,nVehicle); % probabiblity that an RSU choose an action
    proActVehicelIter=cell(1,N);

    %---------------------Vehicular Edge Computing Settings--------------------
    bwR=100*10^6; % Hz (100 MHz)
    alph=0.02; % second per hop
    alph0=0.02; % second per hop
    bwV2I=1*10^6; % Hz (1 MHz)
    bw0=0.25*10^6; % Hz (0.25 MHz)
    cCom=2*10^-6; % communication cost of an RSU (2 units/MHz)
    cCom0=20*10^-6; % communication cost of a BS (20 units/MHz)
    cMig=0.002/(8*10^6); % RSU (0.002 units/MB)
    cMig0=0.002/(8*10^6); % BS (0.02 units/MB)
    cComp=10*1/10^9; % cost unit per Hz at an RSU (10 units/GHz)
    cComp0=100*1/10^9; % cost unit per Hz at a BS (100 units/GHz)

    M=nVehicle; % number of requesting vehicles  

    S=nRSU+1; % number of RSUs and a BS
    avgP = zeros(N,M);             % Average payoff of user i at t                 avgP(t,i)
    switching = zeros(N,M);        % Number of switching of user i at t            switching(t,i)
    sumPayoff = zeros(N,1);
    fairness = zeros(N,1);
    xisquare = zeros(N,M);
    noise = 0;
    mu = zeros(M,1); 
    hr=cell(1,numReq);
    unoise = zeros(M,S);
    VeActIter=cell(1,N);
    TotalCount=cell(1,N);
    sumRegret = zeros(M,S,S);
    realP = zeros(N,M);            % Real payoff of user i at t   
    epsilon = 0.01;

    delayPnty=350;
    costPnty=350;
    %--------------------------------------------------------------------------
    u=cell(1,N); % utility of each player during N iterations 
    action = zeros(1,M);
    %--------------------------------------------------------------------------
    Tsize=ones(1,nVehicle); % size (bits) of each task requested by each vehicle
    tempTsize=2*10^8*8;
    for i=1:length(Tsize)
       Tsize(1,i)=tempTsize*Tsize(1,i); % size of bits (from 200 to 500 MB) 
    end
    %--------------------------------------------------------------------------
    ImageSize=5*10^8*8; % bits (500 MB)
    %--------------------------------------------------------------------------
    Tcpu=ones(1,nVehicle); % CPU cycles required to complete each task of each vehicle
    tempTcpu=5*10^8; 
    for i=1:length(Tcpu)
       Tcpu(1,i)=randi([5 12])*10^8*Tcpu(1,i); % CPU cycles (from 0.5 to 1.2 Gcycles)
    end
    %--------------------------------------------------------------------------
    RcpuMax=ones(1,nRSU+1); % maximum CPU cycles in each RSU or a BS
    RcpuMax(1,1:nRSU)=20*10^9; % maximum (Hz) in each RSU - 20 GHz
    RcpuMax(1,nRSU+1)=30*10^9; % maximum (Hz) in a BS - 30 GHz
    %--------------------------------------------------------------------------
    RcpuT=ones(1,nRSU+1); % (cycles/s-Hz) assigned to each task
    tempRcpuT=1*10^9;
    for i=1:length(RcpuT)
       if i<length(RcpuT)
           RcpuT(1,i)=randi([1 3])*10^9*RcpuT(1,i); % cycles/s (Hz) at an RSU (from 1 to 2 GHz)
       else
           RcpuT(1,i)=randi([1 3])*10^9*RcpuT(1,i); % cycles/s (Hz) at a BS (from 1 to 2 GHz)
       end
    end
    %--------------------------------------------------------------------------
    for i=1:nVehicle
       for ii=1:nRSU+1
           nameAVe{1,i}{1,ii}=[i,ii];
       end
       nameAVe{1,i}{1,nRSU+1}=[i,0];
    end
    %-------------------------Initial Location---------------------------------
    t=1; % seconds
    %----------------------------------------------------------------------
    for ii=1:numReq
       if posRqs{1,ii}(1,2)==initY+35||posRqs{1,ii}(1,2)==initY+25||posRqs{1,ii}(1,2)==initY+15
           posRqs{1,ii}(1,1)=posRqs{1,ii}(1,1)+vTarget(1,ii)*t*tLmt;
           posRqs{1,ii}(1,2)=posRqs{1,ii}(1,2);
       elseif posRqs{1,ii}(1,2)==initY-35||posRqs{1,ii}(1,2)==initY-25||posRqs{1,ii}(1,2)==initY-15
           posRqs{1,ii}(1,1)=posRqs{1,ii}(1,1)-vTarget(1,ii)*t*tLmt;
           posRqs{1,ii}(1,2)=posRqs{1,ii}(1,2);
       end
    end 
    %----------------------------------------------------------------------
    Rqs=ones(1,length(posRqs));
    RqsCurrRSU=cell(1,length(posRqs)); % the vehicle requests which RSU
    for i=1:length(posRqs)  
       if Rqs(1,i)==1
           for ii=1:length(posRSU)
               dist=sqrt((posRqs{1,i}(1,1)-posRSU{1,ii}(1,1))^2+(posRqs{1,i}(1,2)-posRSU{1,ii}(1,2))^2);
               if dist<=R
                   RqsCurrRSU{1,i}=[i,ii];
               end
           end
           if isempty(RqsCurrRSU{1,i})
               RqsCurrRSU{1,i}=[i,0];
           end       
       end
    end    
    %----------------------------------------------------------------------
    for it=1:N   % run iterations
       %-------------------------------------------------------------------
%        disp('Running! Please wait!')

       maxSumPayoff=(delayPnty+costPnty)*nVehicle;

       tic
       %-------------------------------------------------------------------
       for iNum=1:(2^30-1)

            disp('Running! Please wait!')

            temp=de2bi(iNum);
            if length(temp)<30
                tempMRMA=[temp zeros(1,30-length(temp))];
            end
            
            MRMA1=tempMRMA(1,1:10);
            MRMA2=tempMRMA(1,11:20);
            MRMAbs=tempMRMA(1,21:30);     
            
            cPnlt=0;
            for i=1:M
                if MRMA1(1,i)==0&&MRMA2(1,i)==0&&MRMAbs(1,i)==0
                    cPnlt=cPnlt+1;
                    break;                    
                elseif MRMA1(1,i)==1&&MRMA2(1,i)==1&&MRMAbs(1,i)==1
                    cPnlt=cPnlt+1;
                    break;
                elseif MRMA1(1,i)==1&&MRMA2(1,i)==1&&MRMAbs(1,i)==0
                    cPnlt=cPnlt+1;
                    break;
                elseif MRMA1(1,i)==1&&MRMA2(1,i)==0&&MRMAbs(1,i)==1
                    cPnlt=cPnlt+1;
                    break;
                elseif MRMA1(1,i)==0&&MRMA2(1,i)==1&&MRMAbs(1,i)==1
                    cPnlt=cPnlt+1;
                    break;
                elseif (MRMA1(1,i)==1&&MRMA2(1,i)==0&&MRMAbs(1,i)==0)
                    for ii=1:M
                        if ii~=i
                            if MRMA1(1,ii)==0&&MRMA2(1,ii)==0&&MRMAbs(1,ii)==0
                                cPnlt=cPnlt+1;
                            elseif MRMA1(1,ii)==1&&MRMA2(1,ii)==1&&MRMAbs(1,ii)==1
                                cPnlt=cPnlt+1;
                            elseif MRMA1(1,ii)==1&&MRMA2(1,ii)==1&&MRMAbs(1,ii)==0
                                cPnlt=cPnlt+1;
                            elseif MRMA1(1,ii)==1&&MRMA2(1,ii)==0&&MRMAbs(1,ii)==1
                                cPnlt=cPnlt+1;
                            elseif MRMA1(1,ii)==0&&MRMA2(1,ii)==1&&MRMAbs(1,ii)==1
                                cPnlt=cPnlt+1;
                            end
                        end
                    end

                    if cPnlt>0
                        break;
                    else
                        tempRSU=1;
                        tempVe=i;
                        if RqsCurrRSU{1,i}(1,2)==tempRSU
                           Rx=posRSU{1,tempRSU}(1,1);
                           Ry=posRSU{1,tempRSU}(1,2);
                           Vx=posRqs{1,tempVe}(1,1);
                           Vy=posRqs{1,tempVe}(1,2);
                           tempB=bwV2I;                
                           tempRate=DataRate(tempB,Rx,Ry,Vx,Vy);
                           tempSumCpu=RcpuT(1,tempRSU);
    
                           for ii=1:M
                              if ii~=i
                                 if MRMA1(1,ii)==1&&MRMA2(1,ii)==0&&MRMAbs(1,ii)==0
                                     tempSumCpu=tempSumCpu+...
                                         RcpuT(1,tempRSU);
                                 end
                              end
                           end
    
                           % Satisfy the constraints of time
                           if Vy>1000
                               distLeave=(posRSU{1,RqsCurrRSU{1,tempVe}(1,2)}(1,1)+R)-Vx;
                           elseif Vy<1000
                               distLeave=Vx-(posRSU{1,RqsCurrRSU{1,tempVe}(1,2)}(1,1)-R);
                           end
                           delay=Tsize(1,tempVe)/tempRate...
                                     +Tcpu(1,tempVe)/RcpuT(1,tempRSU);
                           if (delay>distLeave/(vTarget(1,i)))
                               delay=delayPnty;
                               cost=costPnty;
                           else
                               % Satisfy the constraints of maximum CPU cycles
                               if (tempSumCpu>RcpuMax(1,tempRSU))
                                   cost=costPnty;
                               else
                                   cost=tempB*cCom+RcpuT(1,tempRSU)*cComp;
                               end
                           end      
                        elseif RqsCurrRSU{1,i}(1,2)==2
                           Rx=posRSU{1,RqsCurrRSU{1,tempVe}(1,2)}(1,1);
                           Ry=posRSU{1,RqsCurrRSU{1,tempVe}(1,2)}(1,2);
                           Vx=posRqs{1,tempVe}(1,1);
                           Vy=posRqs{1,tempVe}(1,2);
                           tempB=bwV2I;
                           tempRate=DataRate(tempB,Rx,Ry,Vx,Vy);
                           nHop=1;
                           tempSumCpu=RcpuT(1,tempRSU);
    
                           for ii=1:M
                                if ii~=i
                                    if MRMA1(1,ii)==1&&MRMA2(1,ii)==0&&MRMAbs(1,ii)==0
                                        tempSumCpu=tempSumCpu+...
                                        RcpuT(1,tempRSU);
                                    end
                                end
                           end
    
                           % Satisfy the constraints of time
                           if Vy>1000
                               distLeave=(posRSU{1,RqsCurrRSU{1,tempVe}(1,2)}(1,1)+R)-Vx;
                           elseif Vy<1000
                               distLeave=Vx-(posRSU{1,RqsCurrRSU{1,tempVe}(1,2)}(1,1)-R);
                           end
                           delay=Tsize(1,tempVe)/tempRate+(Tsize(1,tempVe)/bwR+2*alph*nHop)...
                                    +Tcpu(1,tempVe)/RcpuT(1,tempRSU);
                           if (delay>(distLeave+2*R+D*nHop)/vTarget(1,i))...
                              ||(Vy>1000&&posRSU{1,tempRSU}(1,1)<posRSU{1,RqsCurrRSU{1,tempVe}(1,2)}(1,1))...
                              ||(Vy<1000&&posRSU{1,tempRSU}(1,1)>posRSU{1,RqsCurrRSU{1,tempVe}(1,2)}(1,1))
                               delay=delayPnty;
                               cost=costPnty;
                           else
                               % Satisfy the constraints of maximum CPU cycles
                               if (tempSumCpu>RcpuMax(1,tempRSU))
                                    cost=costPnty;
                               else
                                    cost=tempB*cCom+ImageSize*cMig...
                                        +RcpuT(1,tempRSU)*cComp;
                               end 
                           end
                        elseif RqsCurrRSU{1,i}(1,2)==0
                               Rx=nodesX0;
                               Ry=nodesY0;
                               Vx=posRqs{1,tempVe}(1,1);
                               Vy=posRqs{1,tempVe}(1,2);
                               tempB=bw0;
                               tempRate=DataRate(tempB,Rx,Ry,Vx,Vy);
                               nHop=1;
                               tempSumCpu=RcpuT(1,tempRSU);
                               for ii=1:M
                                    if ii~=i
                                        if MRMA1(1,ii)==1&&MRMA2(1,ii)==0&&MRMAbs(1,ii)==0
                                            tempSumCpu=tempSumCpu+...
                                            RcpuT(1,tempRSU);
                                        end
                                    end
                               end
                               % Satisfy the constraints of time
                               tempDist=abs(Vx-posRSU{1,1}(1,1));
                               indexRSU=1;
                               for iii=2:nRSU                       
                                  if tempDist>abs(Vx-posRSU{1,iii}(1,1))
                                       tempDist=abs(Vx-posRSU{1,iii}(1,1));
                                       indexRSU=iii;
                                  end
                               end
                               if Vy>1000
                                  if Vx>posRSU{1,indexRSU}(1,1)
                                       distLeave=(posRSU{1,indexRSU}(1,1)+D-R)-Vx;
                                  else
                                       distLeave=(posRSU{1,indexRSU}(1,1)-R)-Vx;
                                  end
                               elseif Vy<1000
                                  if Vx>posRSU{1,indexRSU}(1,1)
                                       distLeave=Vx-(posRSU{1,indexRSU}(1,1)+R);
                                  else
                                       distLeave=Vx-(posRSU{1,indexRSU}(1,1)-D+R);
                                  end
                               end 
                               delay=Tsize(1,tempVe)/tempRate...
                                        +(Tsize(1,tempVe)/bwR+2*alph0*nHop)...
                                        +Tcpu(1,tempVe)/RcpuT(1,tempRSU);
                               if tempRSU==indexRSU
                                  tempHop=0;
                               else
                                  tempHop=floor(abs(posRSU{1,indexRSU}(1,1)...
                                        -posRSU{1,tempRSU}(1,1))/D);                             
                               end
                               if (delay>(distLeave+2*R+D*tempHop)/vTarget(1,i))...
                                     ||(Vy>1000&&posRSU{1,tempRSU}(1,1)<Vx)...
                                     ||(Vy<1000&&posRSU{1,tempRSU}(1,1)>Vx)
                                      delay=delayPnty;
                                      cost=costPnty;
                               else
                                      % Satisfy the constraints of maximum CPU cycles
                                      if (tempSumCpu>RcpuMax(1,tempRSU))
                                          cost=costPnty;
                                      else
                                          cost=tempB*cCom0+ImageSize*cMig0...
                                                +RcpuT(1,tempRSU)*cComp;
                                      end
                               end                      
                        end
                        % real utility
                        u{1,it}{1,i}(1,tempRSU)=-(delay+cost);
                    end
                elseif MRMA1(1,i)==0&&MRMA2(1,i)==1&&MRMAbs(1,i)==0
                    for ii=1:M
                        if ii~=i
                            if MRMA1(1,ii)==0&&MRMA2(1,ii)==0&&MRMAbs(1,ii)==0
                                cPnlt=cPnlt+1;
                            elseif MRMA1(1,ii)==1&&MRMA2(1,ii)==1&&MRMAbs(1,ii)==1
                                cPnlt=cPnlt+1;
                            elseif MRMA1(1,ii)==1&&MRMA2(1,ii)==1&&MRMAbs(1,ii)==0
                                cPnlt=cPnlt+1;
                            elseif MRMA1(1,ii)==1&&MRMA2(1,ii)==0&&MRMAbs(1,ii)==1
                                cPnlt=cPnlt+1;
                            elseif MRMA1(1,ii)==0&&MRMA2(1,ii)==1&&MRMAbs(1,ii)==1
                                cPnlt=cPnlt+1;
                            end
                        end
                    end
                    
                    if cPnlt>0
                        break;
                    else
                        tempRSU=2;
                        tempVe=i;
                        if RqsCurrRSU{1,i}(1,2)==tempRSU
                           Rx=posRSU{1,tempRSU}(1,1);
                           Ry=posRSU{1,tempRSU}(1,2);
                           Vx=posRqs{1,tempVe}(1,1);
                           Vy=posRqs{1,tempVe}(1,2);
                           tempB=bwV2I;                
                           tempRate=DataRate(tempB,Rx,Ry,Vx,Vy);
                           tempSumCpu=RcpuT(1,tempRSU);
    
                           for ii=1:M
                              if ii~=i
                                 if MRMA1(1,ii)==0&&MRMA2(1,ii)==1&&MRMAbs(1,ii)==0
                                     tempSumCpu=tempSumCpu+...
                                         RcpuT(1,tempRSU);
                                 end
                              end
                           end
    
                           % Satisfy the constraints of time
                           if Vy>1000
                               distLeave=(posRSU{1,RqsCurrRSU{1,tempVe}(1,2)}(1,1)+R)-Vx;
                           elseif Vy<1000
                               distLeave=Vx-(posRSU{1,RqsCurrRSU{1,tempVe}(1,2)}(1,1)-R);
                           end
                           delay=Tsize(1,tempVe)/tempRate...
                                     +Tcpu(1,tempVe)/RcpuT(1,tempRSU);
                           if (delay>distLeave/(vTarget(1,i)))
                               delay=delayPnty;
                               cost=costPnty;
                           else
                               % Satisfy the constraints of maximum CPU cycles
                               if (tempSumCpu>RcpuMax(1,tempRSU))
                                   cost=costPnty;
                               else
                                   cost=tempB*cCom+RcpuT(1,tempRSU)*cComp;
                               end
                           end      
                        elseif RqsCurrRSU{1,i}(1,2)==1
                           Rx=posRSU{1,RqsCurrRSU{1,tempVe}(1,2)}(1,1);
                           Ry=posRSU{1,RqsCurrRSU{1,tempVe}(1,2)}(1,2);
                           Vx=posRqs{1,tempVe}(1,1);
                           Vy=posRqs{1,tempVe}(1,2);
                           tempB=bwV2I;
                           tempRate=DataRate(tempB,Rx,Ry,Vx,Vy);
                           nHop=1;
                           tempSumCpu=RcpuT(1,tempRSU);
    
                           for ii=1:M
                                if ii~=i
                                    if MRMA1(1,ii)==0&&MRMA2(1,ii)==1&&MRMAbs(1,ii)==0
                                        tempSumCpu=tempSumCpu+...
                                        RcpuT(1,tempRSU);
                                    end
                                end
                           end
    
                           % Satisfy the constraints of time
                           if Vy>1000
                               distLeave=(posRSU{1,RqsCurrRSU{1,tempVe}(1,2)}(1,1)+R)-Vx;
                           elseif Vy<1000
                               distLeave=Vx-(posRSU{1,RqsCurrRSU{1,tempVe}(1,2)}(1,1)-R);
                           end
                           delay=Tsize(1,tempVe)/tempRate+(Tsize(1,tempVe)/bwR+2*alph*nHop)...
                                    +Tcpu(1,tempVe)/RcpuT(1,tempRSU);
                           if (delay>(distLeave+2*R+D*nHop)/vTarget(1,i))...
                              ||(Vy>1000&&posRSU{1,tempRSU}(1,1)<posRSU{1,RqsCurrRSU{1,tempVe}(1,2)}(1,1))...
                              ||(Vy<1000&&posRSU{1,tempRSU}(1,1)>posRSU{1,RqsCurrRSU{1,tempVe}(1,2)}(1,1))
                               delay=delayPnty;
                               cost=costPnty;
                           else
                               % Satisfy the constraints of maximum CPU cycles
                               if (tempSumCpu>RcpuMax(1,tempRSU))
                                    cost=costPnty;
                               else
                                    cost=tempB*cCom+ImageSize*cMig...
                                        +RcpuT(1,tempRSU)*cComp;
                               end 
                           end
                        elseif RqsCurrRSU{1,i}(1,2)==0
                               Rx=nodesX0;
                               Ry=nodesY0;
                               Vx=posRqs{1,tempVe}(1,1);
                               Vy=posRqs{1,tempVe}(1,2);
                               tempB=bw0;
                               tempRate=DataRate(tempB,Rx,Ry,Vx,Vy);
                               nHop=1;
                               tempSumCpu=RcpuT(1,tempRSU);
                               for ii=1:M
                                    if ii~=i
                                        if MRMA1(1,ii)==0&&MRMA2(1,ii)==1&&MRMAbs(1,ii)==0
                                            tempSumCpu=tempSumCpu+...
                                            RcpuT(1,tempRSU);
                                        end
                                    end
                               end
                               % Satisfy the constraints of time
                               tempDist=abs(Vx-posRSU{1,1}(1,1));
                               indexRSU=1;
                               for iii=2:nRSU                       
                                  if tempDist>abs(Vx-posRSU{1,iii}(1,1))
                                       tempDist=abs(Vx-posRSU{1,iii}(1,1));
                                       indexRSU=iii;
                                  end
                               end
                               if Vy>1000
                                  if Vx>posRSU{1,indexRSU}(1,1)
                                       distLeave=(posRSU{1,indexRSU}(1,1)+D-R)-Vx;
                                  else
                                       distLeave=(posRSU{1,indexRSU}(1,1)-R)-Vx;
                                  end
                               elseif Vy<1000
                                  if Vx>posRSU{1,indexRSU}(1,1)
                                       distLeave=Vx-(posRSU{1,indexRSU}(1,1)+R);
                                  else
                                       distLeave=Vx-(posRSU{1,indexRSU}(1,1)-D+R);
                                  end
                               end 
                               delay=Tsize(1,tempVe)/tempRate...
                                        +(Tsize(1,tempVe)/bwR+2*alph0*nHop)...
                                        +Tcpu(1,tempVe)/RcpuT(1,tempRSU);
                               if tempRSU==indexRSU
                                  tempHop=0;
                               else
                                  tempHop=floor(abs(posRSU{1,indexRSU}(1,1)...
                                        -posRSU{1,tempRSU}(1,1))/D);                             
                               end
                               if (delay>(distLeave+2*R+D*tempHop)/vTarget(1,i))...
                                     ||(Vy>1000&&posRSU{1,tempRSU}(1,1)<Vx)...
                                     ||(Vy<1000&&posRSU{1,tempRSU}(1,1)>Vx)
                                      delay=delayPnty;
                                      cost=costPnty;
                               else
                                      % Satisfy the constraints of maximum CPU cycles
                                      if (tempSumCpu>RcpuMax(1,tempRSU))
                                          cost=costPnty;
                                      else
                                          cost=tempB*cCom0+ImageSize*cMig0...
                                                +RcpuT(1,tempRSU)*cComp;
                                      end
                               end                      
                        end
                        % real utility
                        u{1,it}{1,i}(1,tempRSU)=-(delay+cost);
                    end
                elseif MRMA1(1,i)==0&&MRMA2(1,i)==0&&MRMAbs(1,i)==1
                    for ii=1:M
                        if ii~=i
                            if MRMA1(1,ii)==0&&MRMA2(1,ii)==0&&MRMAbs(1,ii)==0
                                cPnlt=cPnlt+1;
                            elseif MRMA1(1,ii)==1&&MRMA2(1,ii)==1&&MRMAbs(1,ii)==1
                                cPnlt=cPnlt+1;
                            elseif MRMA1(1,ii)==1&&MRMA2(1,ii)==1&&MRMAbs(1,ii)==0
                                cPnlt=cPnlt+1;
                            elseif MRMA1(1,ii)==1&&MRMA2(1,ii)==0&&MRMAbs(1,ii)==1
                                cPnlt=cPnlt+1;
                            elseif MRMA1(1,ii)==0&&MRMA2(1,ii)==1&&MRMAbs(1,ii)==1
                                cPnlt=cPnlt+1;
                            end
                        end
                    end

                    if cPnlt>0
                        break;
                    else
                        tempRSU=0;
                        tempVe=i;
                        if RqsCurrRSU{1,i}(1,2)==tempRSU
                           Rx=nodesX0;
                           Ry=nodesY0;
                           Vx=posRqs{1,tempVe}(1,1);
                           Vy=posRqs{1,tempVe}(1,2);
                           tempB=bw0;                
                           tempRate=DataRate(tempB,Rx,Ry,Vx,Vy);
                           tempSumCpu=RcpuT(1,S);
                           for ii=1:M
                              if ii~=i
                                 if MRMA1(1,i)==0&&MRMA2(1,i)==0&&MRMAbs(1,i)==1
                                     tempSumCpu=tempSumCpu+...
                                         RcpuT(1,S);
                                 end
                              end
                           end
                           % Satisfy the constraints of time
                           delay=Tsize(1,tempVe)/tempRate...
                                     +Tcpu(1,tempVe)/RcpuT(1,S);
                           % Satisfy the constraints of maximum CPU cycles
                           if (tempSumCpu>RcpuMax(1,S))
                               cost=costPnty;
                           else
                               cost=tempB*cCom0+RcpuT(1,S)*cComp0;
                           end  
                        elseif RqsCurrRSU{1,i}(1,2)==1
                           Rx=posRSU{1,RqsCurrRSU{1,tempVe}(1,2)}(1,1);
                           Ry=posRSU{1,RqsCurrRSU{1,tempVe}(1,2)}(1,2);
                           Vx=posRqs{1,tempVe}(1,1);
                           Vy=posRqs{1,tempVe}(1,2);
                           tempB=bwV2I;
                           tempRate=DataRate(tempB,Rx,Ry,Vx,Vy);
                           nHop=1;
                           tempSumCpu=RcpuT(1,S);
                           for ii=1:M
                                if ii~=i
                                    if MRMA1(1,i)==0&&MRMA2(1,i)==0&&MRMAbs(1,i)==1
                                        tempSumCpu=tempSumCpu+...
                                        RcpuT(1,S);
                                    end
                                end
                           end
                           % Satisfy the constraints of time
                           if Vy>1000
                               distLeave=(posRSU{1,RqsCurrRSU{1,tempVe}(1,2)}(1,1)+R)-Vx;
                           elseif Vy<1000
                               distLeave=Vx-(posRSU{1,RqsCurrRSU{1,tempVe}(1,2)}(1,1)-R);
                           end
                           delay=Tsize(1,tempVe)/tempRate+(Tsize(1,tempVe)/bwR+2*alph*nHop)...
                                    +Tcpu(1,tempVe)/RcpuT(1,S);
                           if (delay>(distLeave+D*nHop)/vTarget(1,i))...
                              ||(Vy>1000&&nodesX0<posRSU{1,RqsCurrRSU{1,tempVe}(1,2)}(1,1))...
                              ||(Vy<1000&&nodesX0>posRSU{1,RqsCurrRSU{1,tempVe}(1,2)}(1,1))
                               delay=delayPnty;
                               cost=costPnty;
                           else
                               % Satisfy the constraints of maximum CPU cycles
                               if (tempSumCpu>RcpuMax(1,S))
                                    cost=costPnty;
                               else
                                    cost=tempB*cCom+ImageSize*cMig...
                                        +RcpuT(1,S)*cComp;
                               end 
                           end
                        elseif RqsCurrRSU{1,i}(1,2)==2
                           Rx=posRSU{1,RqsCurrRSU{1,tempVe}(1,2)}(1,1);
                           Ry=posRSU{1,RqsCurrRSU{1,tempVe}(1,2)}(1,2);
                           Vx=posRqs{1,tempVe}(1,1);
                           Vy=posRqs{1,tempVe}(1,2);
                           tempB=bwV2I;
                           tempRate=DataRate(tempB,Rx,Ry,Vx,Vy);
                           nHop=1;
                           tempSumCpu=RcpuT(1,S);
                           for ii=1:M
                                if ii~=i
                                    if MRMA1(1,i)==0&&MRMA2(1,i)==0&&MRMAbs(1,i)==1
                                        tempSumCpu=tempSumCpu+...
                                        RcpuT(1,S);
                                    end
                                end
                           end
                           % Satisfy the constraints of time
                           if Vy>1000
                               distLeave=(posRSU{1,RqsCurrRSU{1,tempVe}(1,2)}(1,1)+R)-Vx;
                           elseif Vy<1000
                               distLeave=Vx-(posRSU{1,RqsCurrRSU{1,tempVe}(1,2)}(1,1)-R);
                           end
                           delay=Tsize(1,tempVe)/tempRate+(Tsize(1,tempVe)/bwR+2*alph*nHop)...
                                    +Tcpu(1,tempVe)/RcpuT(1,S);
                           if (delay>(distLeave+D*nHop)/vTarget(1,i))...
                              ||(Vy>1000&&nodesX0<posRSU{1,RqsCurrRSU{1,tempVe}(1,2)}(1,1))...
                              ||(Vy<1000&&nodesX0>posRSU{1,RqsCurrRSU{1,tempVe}(1,2)}(1,1))
                               delay=delayPnty;
                               cost=costPnty;
                           else
                               % Satisfy the constraints of maximum CPU cycles
                               if (tempSumCpu>RcpuMax(1,S))
                                    cost=costPnty;
                               else
                                    cost=tempB*cCom+ImageSize*cMig...
                                        +RcpuT(1,S)*cComp;
                               end 
                           end                                  
                        end
                        % real utility
                        u{1,it}{1,i}(1,S)=-(delay+cost);
                    end
                end

                if tempRSU==0
                   realP(it,i) = -u{1,it}{1,i}(1,S);
                else
                   realP(it,i) = -u{1,it}{1,i}(1,tempRSU);       % Real payoff of user i at iteration it
                end
            end

            if cPnlt>0
            else
                tempSumPayoff = sum(realP(it,1:M),2);
                   
                if tempSumPayoff<maxSumPayoff
                    maxSumPayoff=tempSumPayoff;
    
                    maxMRMA1=MRMA1;
    
                    maxMRMA2=MRMA2;
    
                    maxMRMAbs=MRMAbs;
    
                    maxU=u{1,it};
                end
            end
       end

       compTime=toc;
       %%%%%%%%%%%%%%%%%%%%%%%%%% Fairness -- PoA %%%%%%%%%%%%%%%%%%%%%%%%%%   
       sumPayoff(it,1) = maxSumPayoff;
       %------------------------Vehicular Mobility-----------------------------
%        if it<=tLmt
%            for ii=1:numReq
%                if posRqs{1,ii}(1,2)==initY+35||posRqs{1,ii}(1,2)==initY+25||posRqs{1,ii}(1,2)==initY+15
%                    posRqs{1,ii}(1,1)=posRqs{1,ii}(1,1)+vTarget(1,ii)*t;
%                    posRqs{1,ii}(1,2)=posRqs{1,ii}(1,2);
%                elseif posRqs{1,ii}(1,2)==initY-35||posRqs{1,ii}(1,2)==initY-25||posRqs{1,ii}(1,2)==initY-15
%                    posRqs{1,ii}(1,1)=posRqs{1,ii}(1,1)-vTarget(1,ii)*t;
%                    posRqs{1,ii}(1,2)=posRqs{1,ii}(1,2);
%                end
%            end 
%        end
       %----------------------------------------------------------------------
%        for iii=1:numReq
%             delete(hr{1,iii})
%             hr{1,iii}=plot(posRqs{1,iii}(1,1),posRqs{1,iii}(1,2),'ro','MarkerSize',10,'MarkerFaceColor','r'); 
%             drawnow;
%        end
       %----------------------------------------------------------------------
       Rqs=ones(1,length(posRqs));
       RqsCurrRSU=cell(1,length(posRqs)); % the vehicle requests which RSU
       for i=1:length(posRqs)  
           if Rqs(1,i)==1
               for ii=1:length(posRSU)
                   dist=sqrt((posRqs{1,i}(1,1)-posRSU{1,ii}(1,1))^2+(posRqs{1,i}(1,2)-posRSU{1,ii}(1,2))^2);
                   if dist<=R
                       RqsCurrRSU{1,i}=[i,ii];
                   end
               end
               if isempty(RqsCurrRSU{1,i})
                   RqsCurrRSU{1,i}=[i,0];
               end       
           end
       end 

%        pause(1)
    end
end

minDelayCost=maxSumPayoff

computation_time=compTime
