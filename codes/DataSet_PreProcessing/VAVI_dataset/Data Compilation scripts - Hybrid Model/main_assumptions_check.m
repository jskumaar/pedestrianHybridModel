%% hybrid model assumptions check

%1) constant velocity in moving states - approach crosswalk, cross, walk
%away
%2) constant position in waiting state

for kk=1:30
    for jj=1:6
    
    %Read Data
    [EventsData,~] = xlsread('EventsDatawithApproach.xlsx');  
    [UserPos,~] = xlsread('PedestrianDataRaw.xlsx',6*(kk-1)+jj);
        
    
    %Event times
    times.ApproachStart = EventsData(:,3);
    times.ApproachEnd = EventsData(:,4);
    times.ApproachInWaitArea = EventsData(:,5);
    times.ApproachEndInWaitArea = EventsData(:,6);
    times.WaitStart = EventsData(:,7);
    times.WaitEnd = EventsData(:,8);
    times.CrossStart = EventsData(:,9);
    times.CrossEnd = EventsData(:,10);
    times.RetreatInWaitArea = EventsData(:,11);
    times.RetreatEndInWaitArea = EventsData(:,12);   
    times.RetreatStart = EventsData(:,13);
    times.RetreatEnd = EventsData(:,14);
    
    
    
    
    
    % Pedestrian position filter
    [xZoh,TSteps] = holdFilter(UserPos(:,4),UserPos(:,3),Ts,tStep,1);
    [yZoh,~] = holdFilter(UserPos(:,5),UserPos(:,3),Ts,tStep,1);
    %% 2)WMA (Weighted Moving Average) - equal weights
    [xPosFiltWMA,~] = MwaFilter(xZoh,TSteps,5);
    [yPosFiltWMA,~] = MwaFilter(yZoh,TSteps,5);
    
    %% Pedstrian position filter
    %% 3) Diff WMA   
    [xVelWMA] = DifferenceOperation(xPosFiltWMA,TSteps,order);    
    [yVelWMA] = DifferenceOperation(yPosFiltWMA,TSteps,order);   
    
    %% 3) MWA for Diff WMA
    [xVelFiltWMA,~] = MwaFilter(xVelWMA,TSteps,5);
    [yVelFiltWMA,~] = MwaFilter(yVelWMA,TSteps,5);    
    
        
%     %% Recalculate velocity and position
%     xPosFiltWMA

    %% Velocities
    
        
        
        
        
        
        
        
        
    end
end