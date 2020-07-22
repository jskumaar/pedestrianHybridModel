%Created: 03/30/2019
%Updated: 04/02/2019


%% This script identifies the discrete states in the crossing activities

% 1) Approach, 2) Waiting, 3) Crossing and 4) Retreat (walkway from
% crosswalk)

% 1) Approach - user walking on sidewalk; has a minimum velocity; Approach
% starts when they have taken the ball and reached the -10m point in
% x-direction. Approach ends if wait activity starts or crossing activity starts.

% 2) Wait - user waits before crossing, velocity below a threshold; can cross anywhere from sidewalk
% (jaywalking allowed). There can be multiple wait durations before the
% user decides to cross. As long as the user has not reached the crossing
% threshold of y-distance, they are considered to be waiting from the start
% of the first wait duration till the end of the last wait duration.

% 3) Crosses the road - once wait ends or when users reach the crossing threshold of y-position,
% whichever is earlier; crossing ends when user reaches the other end of
% crosswalk

% 4) Retreat (walkaway) - the user walks away from crosswalk to take the
% ball; retreat starts after crossing ends and retreat ends when user
% reaches the -10m x-position
%
% The two commons transitions in the current dataset are
% a) 1-2-3-4
% b) 1-3-4


% The earlier script split into 4 states was done based on position and velocity (diff of position) but from the noisy Zoh value. 
% Since for all practical purposes the filtered data is used, this script
% uses the smoothened and filtered position and velocity data for
% segregating the discrete states






clear all
close all

%% Filter parameters        
Ts = 0.1;       % sampling time
tStep = 0.1;    % zoh filter resampling time
snip = 0;       % no.of points to remove from start after filtering
order = 2;      % order of difference operator
VelThreshold = 0.10; %threhold to be considered as walking
xPosThreshold = 1.5; %threshold to be considered in the wait area (does not consider jaywalking)
yPosThreshold = 3.2; %threshold to be considered in the wait area (does not consider jaywalking)
FilterWindow = 11;   %dont increase this too much; would affect Wait Start indx calculation
roundDen = 100;    % in case position needs to be rounded; the significant digits of position is higher than the noise.


% variables needed to save to excel
xVelMean = [];
yVelMean = [];
rVelMean = [];
xVelStd = [];
yVelStd = [];
rVelStd = [];
velExcelData = [];
EventIndices = [];



%% Read Data
%Read old events data; this will be used as a set within which the
%transitions are searched
[EventsData,~] = xlsread('EventsDatawithApproach.xlsx');  

%% old event times
EventsData = EventsData - snip*Ts;   %corrections based on snipped data
indices.ApproachStart = int32(EventsData(:,3)*10 + 1);
indices.ApproachStartInWaitArea = int32(EventsData(:,5)*10 + 1);
indices.ApproachEndInWaitArea = int32(EventsData(:,6)*10 +1);
indices.WaitStart = int32(EventsData(:,7)*10 +1);
indices.WaitEnd = int32(EventsData(:,8)*10 +1);
indices.CrossStart = int32(EventsData(:,9)*10 +1);
indices.CrossEnd = int32(EventsData(:,10)*10 +1);
indices.RetreatInWaitArea = int32(EventsData(:,11)*10 +1);
indices.RetreatEnd = int32(EventsData(:,14)*10 +1);

% indices to search for discrete states
indices.start = [indices.ApproachStartInWaitArea,indices.WaitStart,indices.CrossStart,indices.RetreatInWaitArea];
indices.end = [indices.ApproachEndInWaitArea,indices.WaitEnd,indices.CrossEnd,indices.RetreatEnd];

% reorder the old indices for each action
indices.start = reshape(indices.start',[2160,1]);
indices.end = reshape(indices.end',[2160,1]);



%% Data compile loop 
for kk=1:30          % subject ID
    for jj=1:3         % scenario ID
        
        %Read raw position data from the simulation
        [UserPos,~] = xlsread('PedestrianDataRaw.xlsx',6*(kk-1)+jj);
        UserPos = ceil(UserPos*roundDen)/roundDen;
        
        %Define the variable for discrete state
        discreteState = zeros(size(UserPos,1),1);
        
        % Pedestrian position filter
        [xZoh,TSteps] = holdFilter(UserPos(:,4),UserPos(:,3),Ts,tStep,1);
        [yZoh,~] = holdFilter(UserPos(:,5),UserPos(:,3),Ts,tStep,1);
        %% 2)WMA (Weighted Moving Average) - equal weights
        [xPosFiltWMA,~] = MwaFilter(xZoh,TSteps,FilterWindow);
        [yPosFiltWMA,~] = MwaFilter(yZoh,TSteps,FilterWindow);

        %% Pedstrian velocity filter
        %% 3) Diff WMA   
        [xVelWMA] = DifferenceOperation(xPosFiltWMA,TSteps,order);    
        [yVelWMA] = DifferenceOperation(yPosFiltWMA,TSteps,order);   

        %% 3) MWA for Diff WMA
        [xVelFiltWMA,~] = MwaFilter(xVelWMA,TSteps,5);
        [yVelFiltWMA,~] = MwaFilter(yVelWMA,TSteps,5);  
        rVelFiltWMA = sqrt(xVelFiltWMA.^2 + yVelFiltWMA.^2);
        
      for ii=1:6  
        %% calculate new approach, cross, wait and retreat indices
        
        % indices to search for wait end
        indStart = indices.start(72*(kk-1)+24*(jj-1)+4*(ii-1)+1);  %approach start in wait area
        indEnd = indices.start(72*(kk-1)+24*(jj-1)+4*(ii-1)+3)+2*FilterWindow;    %start of cross + buffer for filtering changes
        
        %% wait indices
        WaitEnd = find((rVelFiltWMA(indStart:indEnd)<VelThreshold & abs(yPosFiltWMA(indStart:indEnd))>yPosThreshold),1,'last');
       
        if ~isempty(WaitEnd)
           WaitEnd = WaitEnd + (indStart-1);
           if ~(abs(yPosFiltWMA(WaitEnd+20)<3.5))  %i.e. not crossed the road within 2 s from end of wait; sometimes a low velocity somewhere close to pickign the ball is considered as waiting
                WaitEnd = NaN;
           end
        else
           WaitEnd = NaN;
        end
        
        if ~isnan(WaitEnd)
            WaitStart1 = find((rVelFiltWMA(indStart:indEnd)<VelThreshold & abs(yPosFiltWMA(indStart:indEnd))>yPosThreshold),1);      
            if ~isempty(WaitStart1)
                WaitStart = max(WaitStart1)+(indStart-1);
            else
                WaitStart = NaN;
            end
        else
            WaitStart = NaN;
        end
        
        
        % cross indices
        if ~isnan(WaitEnd)
            CrossStart = WaitEnd+1;
        else
            CrossRoad = find((rVelFiltWMA(indStart:indEnd)>VelThreshold & abs(yPosFiltWMA(indStart:indEnd))<yPosThreshold),1,'first');
            CrossRoad = CrossRoad + (indStart-1);
            CrossStart = CrossRoad;
        end
        diffYPos = [0;diff(abs(yPosFiltWMA))];
        
        % indices to search for  cross end
        indStart = indices.start(72*(kk-1)+24*(jj-1)+4*(ii-1)+3);  %start of cross
        indEnd = indices.end(72*(kk-1)+24*(jj-1)+4*(ii-1)+4);      %end of retreat
                       
        CrossEnd = find((abs(yPosFiltWMA(indStart:indEnd))>3.5 & diffYPos(indStart:indEnd)>0 ));
        CrossEnd = CrossEnd+double(indStart-1);
        CrossEndInd = find(CrossEnd>CrossStart,1,'first');       
        CrossEnd = CrossEnd(CrossEndInd);
        
        % retreat
        RetreatStart = CrossEnd+1;
        RetreatEnd = indices.end(72*(kk-1)+24*(jj-1)+4*(ii-1)+4);
        % approach
        
        ApproachStart = indices.ApproachStart(18*(kk-1)+6*(jj-1)+ii);
        if ~isnan(WaitEnd)
           ApproachEnd = WaitStart-1;
        else
           ApproachEnd = CrossStart-1; 
        end

%%
%%%%%% Alternate option to calculate indices (deprecated)   %%%%%%%%%%

%         % action state; approach-1, wait-2, cross-3, retreat-4
%         for ii=1:6  %for each crossing
%             for mm=1:4 % for each state
%                 ind = 72*(kk-1)+24*(jj-1)+4*(ii-1)+mm;
%                 %identify discrete state based on velocity              
%                 if mm==2
%                     rVel = sqrt(xVelFiltWMA(indices.start(ind):indices.end(ind)).^2 + ....
%                                 yVelFiltWMA(indices.start(ind):indices.end(ind)).^2);
%                             
%                     VelocityThresholdIndices = find(rVel<VelThreshold);
%                     
%                     if isempty(VelocityThresholdIndices)    %if no time during wait zone the velocity is less than 0.05,
%                         discreteState(indices.start(ind):indices.end(ind)) = mm-1;    %it means they are continuously walking and considered as part of the approach state                 
% %                     elseif (max(diff(VelocityThresholdIndices))==1)   % if there is exactly one block of time when they are waiting
%                     else                        
%                         discreteState(indices.start(ind):indices.start(ind)+VelocityThresholdIndices(end)-1) = mm;    %waiting
%                         discreteState(indices.start(ind)+VelocityThresholdIndices(end):indices.end(ind)) = mm+1; %crossing
% %                     else
% %                         closestVelThresholdIndex = find(diff(VelocityThresholdIndices)>1,1,'last');
% %                         discreteState(indices.start(ind):indices.start(ind)+closestVelThresholdIndex) = mm;    %waiting
% %                         discreteState(indices.start(ind)+closestVelThresholdIndex+1:indices.end(ind)) = mm+1; %crossing                                        
%                     end
%                 else
%                     discreteState(indices.start(ind):indices.end(ind)) = mm;
%                 end
%                velExcelData = [velExcelData;[kk,jj,ii,mm]];
%             end
            
          
            indAction{1} = [ApproachStart:ApproachEnd];
            indAction{2} = [WaitStart:WaitEnd];
            indAction{3} = [CrossStart:CrossEnd];
            indAction{4} = [RetreatStart:RetreatEnd];
            
%             indAction = cellfun(@(x) x+double(indStart-1), indAction,'un',0);
            
            
            for mm=1:4
                %mean velocities for each action
                if ~isnan(indAction{mm})
                    xVelMean = [xVelMean; mean(abs(xVelFiltWMA(indAction{mm})))];
                    yVelMean = [yVelMean; mean(abs(yVelFiltWMA(indAction{mm})))];
                    rVelMean = [rVelMean; mean(sqrt(xVelFiltWMA(indAction{mm}).^2 + yVelFiltWMA(indAction{mm}).^2))];

                    xVelStd = [xVelStd; std(abs(xVelFiltWMA(indAction{mm})))];
                    yVelStd = [yVelStd; std(abs(yVelFiltWMA(indAction{mm})))];
                    rVelStd = [rVelStd; std(sqrt(xVelFiltWMA(indAction{mm}).^2 + yVelFiltWMA(indAction{mm}).^2))];
                    discreteState(indAction{mm}) = mm;
                else
                    xVelMean = [xVelMean; NaN];
                    yVelMean = [yVelMean; NaN];
                    rVelMean = [rVelMean; NaN];

                    xVelStd = [xVelStd; NaN];
                    yVelStd = [yVelStd; NaN];
                    rVelStd = [rVelStd; NaN];
                end
                velExcelData = [velExcelData;[kk,jj,ii,mm]];
            end
            
            x=1;
            
            
            EventIndices = [EventIndices;[kk,jj,ii,ApproachStart,ApproachEnd,WaitStart,WaitEnd,CrossStart,CrossEnd,RetreatStart,RetreatEnd]];
            
            
       end
        
        % write discrete states and corresponding filtered position and
        % velocity values used to excel
        excelData = [UserPos(:,1:3),xPosFiltWMA,yPosFiltWMA,xVelFiltWMA,yVelFiltWMA,rVelFiltWMA,discreteState];
        
        excelHeader = {'SubjectID','ScenarioID','Time','x-position WMA','z-position WMA','x-velocity WMA',...
                    'Absolute velocity','z-velocity WMA','Discrete State'};

        
        xlswrite('PedestrianDiscreteDataW11_onlyforMLmodel.xlsx',excelData,6*(kk-1)+jj,'A2');
        xlswrite('PedestrianDiscreteDataW11_onlyforMLmodel.xlsx',excelHeader,6*(kk-1)+jj,'A1'); 
        
       
    end
end

% write mean velocities to excel
% excelData = [velExcelData,xVelMean,yVelMean,rVelMean,xVelStd,yVelStd,rVelStd];
% excelHeader = {'SubjectID','ScenarioID','Crossing ID','Discrete State','Mean x-velocity WMA',...
%     'Mean z-velocity WMA','Std x-velocity WMA','Std z-velocity WMA','Mean abs-velocity WMA','Std abs-velocity WMA'};
% xlswrite('PedestrianVelocityDifference.xlsx',excelData,1,'A2');
% xlswrite('PedestrianVelocityDifference.xlsx',excelHeader,1,'A1'); 

% write event new indices to excel
excelHeader = {'SubjectID','ScenarioID','Crossing ID','ApproachStart','ApproachEnd','WaitStart',...
                'WaitEnd','CrossStart','CrossEnd','RetreatStart','RetreatEnd'};
xlswrite('DiscreteStateEventIndicesW5.xlsx',EventIndices,1,'A2');
xlswrite('DiscreteStateEventIndicesW5.xlsx',excelHeader,1,'A1'); 
