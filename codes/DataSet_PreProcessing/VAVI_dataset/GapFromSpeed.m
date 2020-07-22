%% Gap from vehicle speed and distance to pedestrian;
% Calcuate gap probabilities
clear all
close all



%% Updated 05/12/2019:
%1. Updated the expected gap table to include both the current and next
%gaps, so that subsequent analysis dont have to do that identification each time
%2. DTCurb was updated; earlier found distance from a single fixed point
%irrrespective of direction of crossing
%3. DTCW also updated to give the x-distance between user and crosswalk
%instead of the absolute distance
%4. 

%% Updated 10/19/2019:
% added additional variables such as vehicle speed and distance; but not
% confident of this code!!



%add paths
addpath('G:\my drive\Research\Pedestrian Modelling Project\Modelling Scripts and Results\Mat Data\')
addpath('G:\My Drive\Research\Pedestrian Modelling Project\Study I Data for Modeling\Compiled Data\');


%% inputs

% waitThreshold=300;
% k=5;        %cross-validation fold

%% Read DataPredict{ii}
VehicleGapTimes = xlsread('VehicleGapTimesV6.xlsx');
GapData = xlsread('GapWiseCompiledDataV6_65535_removed.xlsx');

% EventIndices = xlsread('DiscreteStateEventIndicesW11_onlyforMLmodel_DONT_USE.xlsx');
% load('AllFeaturesCrossingWise_PW_11.mat')

EventIndices = xlsread('DiscreteStateEventIndicesW5.xlsx');
load('AllFeaturesCrossingWise_PW_5_SpeedHist_CorrectDTCurbDTCW.mat')


% WCExpectedGap = GapDataPredict{ii}(:,101);
% WCExpectedGapStartGap = GapDataPredict{ii}(:,101);
% WCExpectedGapAcc = zeros(length(GapDataPredict{ii}),1);
% WCExpectedGapStartGapAcc = zeros(length(GapDataPredict{ii}),1);
% 
CrossingStart = EventIndices(:,8);
ApproachStart = EventIndices(:,4);
% GapStart = max([VehicleGapTimes(:,7),VehicleGapTimes(:,5)]')';
GapStart = VehicleGapTimes(:,5);
GapEnd = VehicleGapTimes(:,8);
GapEndOnRoad = VehicleGapTimes(:,9);

%indices of vehicle gaps for each crossing
ind = find(diff(VehicleGapTimes(:,3))~=0);
indStart = [1;ind+1];
indEnd = [ind;length(VehicleGapTimes)];


%% Gap acceptance from speed
for ii=1:540
        % for every crossing
        syms x
        indtoFind = [indStart(ii):indEnd(ii)];
            
%         for ii=GapStart(indtoFind(1)):GapEnd(indtoFind(end))
%             if (abs(DataPredict{ii}.PedestrianPosition(ii,1))>=1.5 & abs(DataPredict{ii}.PedestrianPosition(ii+1,1))<1.5)
%                 PedCrossIndex(index,1) = ii;
%             end
%             if (abs(DataPredict{ii}.PedestrianPosition(ii,1))>=1.0 & abs(DataPredict{ii}.PedestrianPosition(ii+1,1))<1.0)
%                 PedCrossIndex(index,2) = ii;
%             end
%             if (abs(DataPredict{ii}.PedestrianPosition(ii,1))>=0.5 & abs(DataPredict{ii}.PedestrianPosition(ii+1,1))<0.5)
%                 PedCrossIndex(index,3) = ii;
%             end
%             if (abs(DataPredict{ii}.PedestrianPosition(ii,1))>=0.1 & abs(DataPredict{ii}.PedestrianPosition(ii+1,1))<0.1)
%                 PedCrossIndex(index,4) = ii;
%             end
%         end
        
    
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %% Expected Gaps - wait to cross
            
            % to calculate in all gaps
            for mm=1:length(indtoFind)
                    
                    GapStartIndex = GapStart(indtoFind(mm)) - ApproachStart(ii)+1;      %when vehicle gap starts before the pedestrian starts their approach, assume the vehicle gap starts as when they first start approaching
                    
                    if GapStartIndex<=0
                        GapStartIndex=1;
                    end
                    
                    if GapStartIndex>0
                        WCExpectedGapStartGap(indtoFind(mm),1) = DataPredict{ii}.VehicleTimeGaptoPedestrian(GapStartIndex);
                        WCExpectedNextVehicleGapStartGap(indtoFind(mm),1) = DataPredict{ii}.NextVehicleTimeGaptoPedestrian(GapStartIndex);
                    
                               
                       % acceleration based gap when crossed gap starts
                        soln = double(real(solve(-DataPredict{ii}.VehicleAcceleration(GapStartIndex)/2*x^2 - DataPredict{ii}.VehicleSpeed(GapStartIndex)*x...
                                                            - DataPredict{ii}.PedestrianVehicleDistance(GapStartIndex)==0,x,'IgnoreAnalyticConstraints', true))); 

                       [~,temp] = min(abs(soln-WCExpectedGapStartGap(indtoFind(mm))));
                       if ~isempty(temp)
                           WCExpectedGapStartGapAcc(indtoFind(mm),1) = soln(temp);
                       else
                           WCExpectedGapStartGapAcc(indtoFind(mm),1) = WCExpectedGapStartGap(indtoFind(mm),1);
                       end

                       % acceleration based gap when crossed gap starts for
                       % next vehicle
                        soln = double(real(solve(-DataPredict{ii}.NextVehicleAcceleration(GapStartIndex)/2*x^2 - DataPredict{ii}.NextVehicleSpeed(GapStartIndex)*x...
                                                            - DataPredict{ii}.PedestrianNextVehicleDistance(GapStartIndex)==0,x,'IgnoreAnalyticConstraints', true))); 

                       [~,temp] = min(abs(soln-WCExpectedNextVehicleGapStartGap(indtoFind(mm))));
                       if ~isempty(temp)
                           WCExpectedNextVehicleGapStartGapAcc(indtoFind(mm),1) = soln(temp);
                       else
                           WCExpectedNextVehicleGapStartGapAcc(indtoFind(mm),1) = WCExpectedNextVehicleGapStartGap(indtoFind(mm),1);
                       end


                       %% gaze ratios - wait to cross
                       GazeRatiosGapStart(indtoFind(mm),:) = DataPredict{ii}.GazeAtVehicleRatio(GapStartIndex,:);
                       PedestrianAbsoluteVelocityAverage(indtoFind(mm),:) = DataPredict{ii}.PedestrianAbsoluteVelocityAverage(GapStartIndex,:);
                       PedestrianHeading(indtoFind(mm),:) = DataPredict{ii}.PedestrianHeading(GapStartIndex,:);
                       PedestrianCumulativeWaitTime(indtoFind(mm),:) = VehicleGapTimes(indtoFind(mm),:);
                       GazeAngle(indtoFind(mm),:) = DataPredict{ii}.GazeAngle(GapStartIndex,:);
                       VehicleLaneID(indtoFind(mm),:) = DataPredict{ii}.VehicleLaneID(GapStartIndex,:);
                       
                       PedestrianDistancetoCurb(indtoFind(mm),:) = DataPredict{ii}.PedestrianDistancetoCurb_new(GapStartIndex,:);
                       PedestrianDistancetoCW(indtoFind(mm),:) = DataPredict{ii}.PedestrianDistancetoCW_new(GapStartIndex,:);
                       
                       % new variables - added 10/17/19
                       VehicleDistancetoPed(indtoFind(mm),:) = DataPredict{ii}.PedestrianVehicleDistance(GapStartIndex,:);
                       VehicleSpeed(indtoFind(mm),:) = abs(DataPredict{ii}.VehicleSpeed(GapStartIndex,:));
                       
                       
                       
                       
                       
                       
                    end
                    
            end
            
            
            CrossStartIndex = CrossingStart(ii) - ApproachStart(ii)+1;
            OnRoadIndex = GapEndOnRoad(indtoFind(mm)) - ApproachStart(ii)+1;
                       
            WCExpectedGapCrossStart(indtoFind(mm),1) = DataPredict{ii}.VehicleTimeGaptoPedestrian(CrossStartIndex);
            WCExpectedNextVehicleGapCrossStart(indtoFind(mm),1) = DataPredict{ii}.NextVehicleTimeGaptoPedestrian(CrossStartIndex);
            GazeRatiosBeforeCrossing(indtoFind(mm),:) = DataPredict{ii}.GazeAtVehicleRatio(CrossStartIndex,:);
                        
            WCExpectedGapOnRoad(indtoFind(mm),1) = DataPredict{ii}.VehicleTimeGaptoPedestrian(OnRoadIndex);
            WCExpectedNextVehicleGapOnRoad(indtoFind(mm),1) = DataPredict{ii}.NextVehicleTimeGaptoPedestrian(OnRoadIndex);
            GazeRatiosBeforeOnRoad(indtoFind(mm),:) = DataPredict{ii}.GazeAtVehicleRatio(OnRoadIndex,:);
            

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
           % acceleration based gap when crossed gap starts
            soln = double(real(solve(-DataPredict{ii}.VehicleAcceleration(CrossStartIndex)/2*x^2 - DataPredict{ii}.VehicleSpeed(CrossStartIndex)*x...
                                                - DataPredict{ii}.PedestrianVehicleDistance(CrossStartIndex)==0,x,'IgnoreAnalyticConstraints', true))); 

           [~,temp] = min(abs(soln-WCExpectedGapCrossStart(indtoFind(mm))));
           if ~isempty(temp)
               WCExpectedGapCrossStartAcc(indtoFind(mm),1) = soln(temp);
           else
               WCExpectedGapCrossStartAcc(indtoFind(mm),1) = WCExpectedGapCrossStart(indtoFind(mm),1);
           end
           
           
                       
           % acceleration based gap when crossed gap starts
            soln = double(real(solve(-DataPredict{ii}.NextVehicleAcceleration(CrossStartIndex)/2*x^2 - DataPredict{ii}.NextVehicleSpeed(CrossStartIndex)*x...
                                                - DataPredict{ii}.PedestrianNextVehicleDistance(CrossStartIndex)==0,x,'IgnoreAnalyticConstraints', true))); 

           [~,temp] = min(abs(soln-WCExpectedNextVehicleGapCrossStart(indtoFind(mm))));
           if ~isempty(temp)
               WCExpectedNextVehicleGapCrossStartAcc(indtoFind(mm),1) = soln(temp);
           else
               WCExpectedNextVehicleGapCrossStartAcc(indtoFind(mm),1) = WCExpectedNextVehicleGapCrossStart(indtoFind(mm),1);
           end

           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
           
           % acceleration based gap when crossed gap starts
            soln = double(real(solve(-DataPredict{ii}.VehicleAcceleration(OnRoadIndex)/2*x^2 - DataPredict{ii}.VehicleSpeed(OnRoadIndex)*x...
                                                - DataPredict{ii}.PedestrianVehicleDistance(OnRoadIndex)==0,x,'IgnoreAnalyticConstraints', true))); 

           [~,temp] = min(abs(soln-WCExpectedGapOnRoad(indtoFind(mm))));
           if ~isempty(temp)
               WCExpectedGapOnRoadAcc(indtoFind(mm),1) = soln(temp);
           else
               WCExpectedGapOnRoadAcc(indtoFind(mm),1) = WCExpectedGapOnRoad(indtoFind(mm),1);
           end
           
           
                       
           % acceleration based gap when crossed gap starts
            soln = double(real(solve(-DataPredict{ii}.NextVehicleAcceleration(OnRoadIndex)/2*x^2 - DataPredict{ii}.NextVehicleSpeed(OnRoadIndex)*x...
                                                - DataPredict{ii}.PedestrianNextVehicleDistance(OnRoadIndex)==0,x,'IgnoreAnalyticConstraints', true))); 

           [~,temp] = min(abs(soln-WCExpectedNextVehicleGapOnRoad(indtoFind(mm))));
           if ~isempty(temp)
               WCExpectedNextVehicleGapOnRoadAcc(indtoFind(mm),1) = soln(temp);
           else
               WCExpectedNextVehicleGapOnRoadAcc(indtoFind(mm),1) = WCExpectedNextVehicleGapOnRoad(indtoFind(mm),1);
           end
           
           
           
           
          % distance of vehicle

           [~,temp] = min(abs(soln-WCExpectedNextVehicleGapOnRoad(indtoFind(mm))));
           if ~isempty(temp)
               WCExpectedNextVehicleGapOnRoadAcc(indtoFind(mm),1) = soln(temp);
           else
               WCExpectedNextVehicleGapOnRoadAcc(indtoFind(mm),1) = WCExpectedNextVehicleGapOnRoad(indtoFind(mm),1);
           end
           
    
    
end



% Expected Gap - 1
ExpectedGap2 = WCExpectedGapStartGap;
WCExpectedNextVehicleGapCrossStart2 = WCExpectedNextVehicleGapCrossStart;

CrossIndices = find(VehicleGapTimes(:,4)==2 & WCExpectedGapStartGap<20 & WCExpectedGapCrossStart<20 & WCExpectedGapOnRoad<20);
NotSameGapIndices = find(VehicleGapTimes(:,4)==1);
NotSameGapIndicesCheck = NotSameGapIndices+1;
[CommonIndices,~,~] = intersect(CrossIndices,NotSameGapIndicesCheck);

ExpectedGap2(CommonIndices) = WCExpectedNextVehicleGapCrossStart2(CommonIndices);
ExpectedGap_bothCurrentNextVehicle = ExpectedGap2;


ExpectedGapData = table(VehicleGapTimes,ExpectedGap_bothCurrentNextVehicle,WCExpectedGapStartGap,WCExpectedGapCrossStart,WCExpectedGapOnRoad,WCExpectedNextVehicleGapStartGap,...
                        WCExpectedNextVehicleGapCrossStart,WCExpectedNextVehicleGapOnRoad,WCExpectedGapStartGapAcc,WCExpectedGapCrossStartAcc,...
                        WCExpectedGapOnRoadAcc,WCExpectedNextVehicleGapStartGapAcc,WCExpectedNextVehicleGapCrossStartAcc,WCExpectedNextVehicleGapOnRoadAcc,...
                        GazeRatiosGapStart,GazeRatiosBeforeCrossing,GazeRatiosBeforeOnRoad, PedestrianAbsoluteVelocityAverage,PedestrianDistancetoCW,...
                        PedestrianDistancetoCurb,PedestrianHeading,PedestrianCumulativeWaitTime,GazeAngle,VehicleLaneID,VehicleDistancetoPed,VehicleSpeed);
                   
                    
save('ExpectedGapData_10_17_2019.mat','ExpectedGapData');


%                % calculate only for crossing gaps; for other gaps consider
%                % start of gap as gap;
%                 WCExpectedGap(indEnd(index),1) = DataPredict{ii}.VehicleTimeGaptoPedestrian(crossingStart(index)); 
%                 
%                 % acceleration based gap when starting to cross
%                 soln = double(real(solve(-DataPredict{ii}.VehicleAcceleration(crossingStart(index))/2*x^2 - DataPredict{ii}.VehicleSpeed(crossingStart(index))*x...
%                                                     - DataPredict{ii}.PedestrianVehicleDistance(crossingStart(index))==0,x,'IgnoreAnalyticConstraints', true))); 
%                 
%                [~,temp] = min(abs(soln-WCExpectedGap(indEnd(index))));
%                 if ~isempty(temp)
%                     WCExpectedGapAcc(indEnd(index),1) = soln(temp);
%                 else
%                     WCExpectedGapAcc(indEnd(index),1) = WCExpectedGap(indEnd(index),1);
%                 end            
%             
%                 WCExpectedGap(indEnd(index),1) = NextVehicleTimeGap(crossingStart(index));
%                 WCExpectedGapStartGap(indEnd(index),1) = NextVehicleTimeGap(GapStart(indEnd(index)));  
%                 
%                 % acceleration based gap when starting to cross
%                 soln = double(real(solve(-DataPredict{ii}.NextVehicleAcceleration(crossingStart(index))/2*x^2 - DataPredict{ii}.NextVehicleSpeed(crossingStart(index))*x...
%                                                     - DataPredict{ii}.PedestrianNextVehicleDistance(crossingStart(index))==0,x,'IgnoreAnalyticConstraints', true)));  
%                 
%                 [~,temp] = min(abs(soln-WCExpectedGap(indEnd(index))));
%                 if ~isempty(temp)
%                     WCExpectedGapAcc(indEnd(index),1) = soln(temp);
%                 else
%                     WCExpectedGapAcc(indEnd(index),1) = WCExpectedGap(indEnd(index),1);
%                 end
%                 
%                 % acceleration based gap when gap starts
%                 soln = double(real(solve(-DataPredict{ii}.NextVehicleAcceleration(crossingStart(index))/2*x^2 - DataPredict{ii}.NextVehicleSpeed(crossingStart(index))*x...
%                                                     - DataPredict{ii}.PedestrianNextVehicleDistance(crossingStart(index))==0,x,'IgnoreAnalyticConstraints', true))); 
% 
%                [~,temp] = min(abs(soln-WCExpectedGap(indtoFind(mm))));
%                 if ~isempty(temp)
%                     WCExpectedGapStartGapAcc(indEnd(index),1) = soln(temp);
%                 else
%                     WCExpectedGapStartGapAcc(indEnd(index),1) = WCExpectedGapStartGap(indEnd(index),1);
%                 end
%             
%             
%             
%                 
%             
%             %when last gap of the crossing is the gap when crossing starts
%             if (crossingStart(index)>=GapStart(indEnd(index)) & crossingStart(index)<=GapEnd(indEnd(index)))
%                % calculate only for crossing gaps; for other gaps consider
%                % start of gap as gap;
%                 WCExpectedGap(indEnd(index),1) = DataPredict{ii}.VehicleTimeGaptoPedestrian(crossingStart(index)); 
%                 
%                 % acceleration based gap when starting to cross
%                 soln = double(real(solve(-DataPredict{ii}.VehicleAcceleration(crossingStart(index))/2*x^2 - DataPredict{ii}.VehicleSpeed(crossingStart(index))*x...
%                                                     - DataPredict{ii}.PedestrianVehicleDistance(crossingStart(index))==0,x,'IgnoreAnalyticConstraints', true))); 
%                 
%                [~,temp] = min(abs(soln-WCExpectedGap(indEnd(index))));
%                 if ~isempty(temp)
%                     WCExpectedGapAcc(indEnd(index),1) = soln(temp);
%                 else
%                     WCExpectedGapAcc(indEnd(index),1) = WCExpectedGap(indEnd(index),1);
%                 end                           
%             else
%                 WCExpectedGap(indEnd(index),1) = NextVehicleTimeGap(crossingStart(index));
%                 WCExpectedGapStartGap(indEnd(index),1) = NextVehicleTimeGap(GapStart(indEnd(index)));  
%                 
%                 % acceleration based gap when starting to cross
%                 soln = double(real(solve(-DataPredict{ii}.NextVehicleAcceleration(crossingStart(index))/2*x^2 - DataPredict{ii}.NextVehicleSpeed(crossingStart(index))*x...
%                                                     - DataPredict{ii}.PedestrianNextVehicleDistance(crossingStart(index))==0,x,'IgnoreAnalyticConstraints', true)));  
%                 
%                 [~,temp] = min(abs(soln-WCExpectedGap(indEnd(index))));
%                 if ~isempty(temp)
%                     WCExpectedGapAcc(indEnd(index),1) = soln(temp);
%                 else
%                     WCExpectedGapAcc(indEnd(index),1) = WCExpectedGap(indEnd(index),1);
%                 end
%                 
%                 % acceleration based gap when gap starts
%                 soln = double(real(solve(-DataPredict{ii}.NextVehicleAcceleration(crossingStart(index))/2*x^2 - DataPredict{ii}.NextVehicleSpeed(crossingStart(index))*x...
%                                                     - DataPredict{ii}.PedestrianNextVehicleDistance(crossingStart(index))==0,x,'IgnoreAnalyticConstraints', true))); 
% 
%                [~,temp] = min(abs(soln-WCExpectedGap(indtoFind(mm))));
%                 if ~isempty(temp)
%                     WCExpectedGapStartGapAcc(indEnd(index),1) = soln(temp);
%                 else
%                     WCExpectedGapStartGapAcc(indEnd(index),1) = WCExpectedGapStartGap(indEnd(index),1);
%                 end
%             end
%             
%                        
%             %% gaze ratio calculation - wait to cross
%             GazeRatiosBeforeCrossing(indEnd(index),:) = DataPredict{ii}.GazeAtVehicleRatio(crossingStart(index),:);
% 
%             
%             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%         





% 
% %indices of vehicle gaps for each crossing
% ind = find(diff(vehicleGapTimes(:,3))~=0);
% indStart = [1;ind+1];
% indEnd = [ind;length(vehicleGapTimes)];
% 
% %%
% PedCrossIndex = ones(540,4);
% 
% AWCExpectedGap = zeros(540,4);
% AWCExpectedGapAcc = zeros(540,4);
% AWCGazeRatio_10 = zeros(540,4);
% AWCGazeRatio_15 = zeros(540,4);
% AWCGazeRatio_20 = zeros(540,4);
% AWCGazeRatio_25 = zeros(540,4);
% AWCGazeRatio_30 = zeros(540,4);
% 
% 
% AWCPedSpeed = zeros(540,4);
% 
% 
% %loop for every subject and every crossing
% for kk=1:30
%     for jj=1:3
%         % read vehicle DataPredict{ii} and pedestrian DataPredict{ii} and save it in a table
%         DataPredict{ii} = HybridReadDataPredict{ii}(kk,jj);
%         % Next vehicle DataPredict{ii}, for when crossing gap and on road are
%         % different; the decisions are made for the next vehicle gap and
%         % not the current one
%         NextVehicleDTC = DataPredict{ii}.NextVehiclePosition - DataPredict{ii}.PedestrianPosition(:,1);
%         NextVehicleTimeGap = NextVehicleDTC./abs(DataPredict{ii}.NextVehicleSpeed);
%         
%         %% find pedestrian indices of approaching CW 1.5, 1.0, 0.5 and 0 m
% 
%             
%         % for every crossing
%         for ii=1:6
%             syms x
%             index = 18*(kk-1)+6*(jj-1)+ii;  % of this crossing
%             indtoFind = [indStart(index):indEnd(index)];
%             
%         for nn=GapStart(indtoFind(1)):GapEnd(indtoFind(end))
%             if (abs(DataPredict{ii}.PedestrianPosition(nn,1))>=1.5 & abs(DataPredict{ii}.PedestrianPosition(nn+1,1))<1.5)
%                 PedCrossIndex(index,1) = nn;
%             end
%             if (abs(DataPredict{ii}.PedestrianPosition(nn,1))>=1.0 & abs(DataPredict{ii}.PedestrianPosition(nn+1,1))<1.0)
%                 PedCrossIndex(index,2) = nn;
%             end
%             if (abs(DataPredict{ii}.PedestrianPosition(nn,1))>=0.5 & abs(DataPredict{ii}.PedestrianPosition(nn+1,1))<0.5)
%                 PedCrossIndex(index,3) = nn;
%             end
%             if (abs(DataPredict{ii}.PedestrianPosition(nn,1))>=0.1 & abs(DataPredict{ii}.PedestrianPosition(nn+1,1))<0.1)
%                 PedCrossIndex(index,4) = nn;
%             end
%         end
%                  
%  

%             %% Expected Gaps and gaze of Approach to wait/cross
% %             if (GapDataPredict{ii}(indEnd(index),11)==3 & (GapDataPredict{ii}(indEnd(index),10)==0))               
%                     AWCExpectedGap(index,:) = DataPredict{ii}.VehicleTimeGaptoPedestrian(PedCrossIndex(index,:));
%                     for mm=1:4 
%                         % acceleration based gap when starting to cross
%                         soln = double(real(solve(-DataPredict{ii}.VehicleAcceleration(PedCrossIndex(index,mm))/2*x^2 - DataPredict{ii}.VehicleSpeed(PedCrossIndex(index,mm))*x...
%                                                             - DataPredict{ii}.PedestrianVehicleDistance(PedCrossIndex(index,mm))==0,x,'IgnoreAnalyticConstraints', true)));  
% 
%                         [~,temp] = min(abs(soln-AWCExpectedGap(index)));
%                         if ~isempty(temp)
%                             AWCExpectedGapAcc(index,mm) = soln(temp);
%                         else
%                             AWCExpectedGapAcc(index,mm) = AWCExpectedGap(index,mm);
%                         end
%                     end
%                     
% %             else
% %                     AWCExpectedGap(index,:) = NextVehicleTimeGap(PedCrossIndex(index,:));
% %                     for mm=1:4 
% %                         % acceleration based gap when starting to cross
% %                         soln = double(real(solve(-DataPredict{ii}.NextVehicleAcceleration(PedCrossIndex(index,mm))/2*x^2 - DataPredict{ii}.NextVehicleSpeed(PedCrossIndex(index,mm))*x...
% %                                                             - DataPredict{ii}.PedestrianNextVehicleDistance(PedCrossIndex(index,mm))==0,x,'IgnoreAnalyticConstraints', true)));  
% % 
% %                         [~,temp] = min(abs(soln-AWCExpectedGap(index)));
% %                         AWCExpectedGapAcc(index,mm) = soln(temp);
% %                     end
% %             end    
% 
% 
%             for bb=1:4
%                 AWCPedSpeed(index,bb) = nanmean(DataPredict{ii}.PedestrianAbsoluteVelocity(PedCrossIndex(index,bb):PedCrossIndex(index,bb)+10,1));
%             end
%                 
%       
%             AWCGazeRatio_10(index,:) = DataPredict{ii}.GazeAtVehicleRatio(PedCrossIndex(index,:),1);
%             AWCGazeRatio_15(index,:) = DataPredict{ii}.GazeAtVehicleRatio(PedCrossIndex(index,:),2);
%             AWCGazeRatio_20(index,:) = DataPredict{ii}.GazeAtVehicleRatio(PedCrossIndex(index,:),3);
%             AWCGazeRatio_25(index,:) = DataPredict{ii}.GazeAtVehicleRatio(PedCrossIndex(index,:),4);
%             AWCGazeRatio_30(index,:) = DataPredict{ii}.GazeAtVehicleRatio(PedCrossIndex(index,:),5);
% 
% 
%             %% update all NA pedestrian indices DataPredict{ii}
%             
%             AWCExpectedGap(PedCrossIndex==1) = NaN;
%             AWCExpectedGapAcc(PedCrossIndex==1) = NaN;
%             AWCGazeRatio_10(PedCrossIndex==1) = NaN;
%             AWCGazeRatio_15(PedCrossIndex==1) = NaN;
%             AWCGazeRatio_20(PedCrossIndex==1) = NaN;
%             AWCGazeRatio_25(PedCrossIndex==1) = NaN;
%             AWCGazeRatio_30(PedCrossIndex==1) = NaN;
%             AWCPedSpeed(PedCrossIndex==1) = NaN;
% 
%             
%             
%         end
% 
% 
% 
%          x=1;
%         
%     end
% end
%  
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% 
% 
% % 
% 
% 
% for mm=1:540
%     
% %    
% %     
% %    AWCDataPredict{ii}(indEnd(mm),:) = [AWCExpectedGap(mm,:),AWCExpectedGapAcc(mm,:),AWCGazeRatio_10(mm,:),...
% %                             AWCGazeRatio_15(mm,:),AWCGazeRatio_20(mm,:),AWCGazeRatio_25(mm,:),AWCGazeRatio_30(mm,:)];
%                         
%           PedDataPredict{ii}(indEnd(mm),:)=AWCPedSpeed(mm,:);
%     
% end
% 
% 
% save('WC_AWC_DataPredict{ii}_w_ped_speed.mat')





% 
% 
% 
% 
% 
% 
% 
% 
% 
% %use gap acceptance DataPredict{ii} for only crossings with nominal waiting times;
% %remove the outliers DataPredict{ii}! These indices are crossing indices
% indices.WaitActivityOutliers = find((EventIndices(:,7)-EventIndices(:,6))>waitThreshold);
% 
% 
% % vehicle gaps indices corresponding to the waiting outliers
% indices.GapToRemoveWaitTimeOutliers =[];
% for ii=1:length(indices.WaitActivityOutliers)
%      indices.GapToRemoveWaitTimeOutliers = [indices.GapToRemoveWaitTimeOutliers;[indStart(indices.WaitActivityOutliers(ii)):indEnd(indices.WaitActivityOutliers(ii))]'];
% end
% 
% % crossing and approach indices; do not count for gap rejection
% indices.Approach = find(GapDataPredict{ii}(:,11)==1);
% indices.Cross = find(GapDataPredict{ii}(:,11)==3);
% 
% %unique vehicle gap indices for rejected gaps
% indices.uniqueRemoveRejectedGap = unique([indices.Approach;indices.GapToRemoveWaitTimeOutliers;indices.Cross]);
% 
% %% Rejected Gaps
% RejectedGaps = GapDataPredict{ii}(:,101);
% RejectedGaps(indices.uniqueRemoveRejectedGap) = [];
% indices.RejectedGapIndices = [1:length(GapDataPredict{ii})]';
% indices.RejectedGapIndices(indices.uniqueRemoveRejectedGap) = [];
% 
% %crossing indices of gaps accepted
% indices.WaitActivityNotOutliers = find((EventIndices(:,7)-EventIndices(:,6))<=waitThreshold);
% 
% %% Accepted Gaps
% indices.AcceptedGapIndices = indices.Cross(indices.WaitActivityNotOutliers);
% 
% %% Accepted gaps while waiting
% indices.AcceptedGapWhileWaitIndices = find(GapDataPredict{ii}(:,11)==3 & (GapDataPredict{ii}(:,10)~=0));
% indices.AcceptedGapsWhileApproachIndices = find(GapDataPredict{ii}(:,11)==3 & (GapDataPredict{ii}(:,10)==0));
% 
% [~,ind,~] = intersect(indices.AcceptedGapWhileWaitIndices,indices.AcceptedGapIndices);
% indices.AcceptedGapWhileWaitIndicesNoOutlier = indices.AcceptedGapWhileWaitIndices(ind);
% 
% [~,ind,~] = intersect(indices.AcceptedGapsWhileApproachIndices,indices.AcceptedGapIndices);
% indices.AccepetedGapsWhileApproachIndicesNoOutlier = indices.AccepetedGapsWhileApproachIndices(ind);
% 
% 
% %% Probabilities
% N = length(indices.AcceptedGapWhileWaitIndicesNoOutlier);
% M = length(indices.RejectedGapIndices);
% Prob_GapAcceptance = N/(N+M);
% 
% % AllWaitToCrossGaps = WCExpectedGap([indices.AcceptedGapWhileWaitIndicesNoOutlier;indices.RejectedGapIndices]);
% % AcceptedWaitToCrossGaps = WCExpectedGap([indices.AcceptedGapWhileWaitIndicesNoOutlier]);
% % RejectedWaitToCrossGaps = WCExpectedGap([indices.RejectedGapIndices]);
% 
% AllWaitToCrossGaps = WCExpectedGap([indices.AcceptedGapWhileWaitIndicesNoOutlier;indices.RejectedGapIndices]);
% AcceptedWaitToCrossGaps = WCExpectedGap([indices.AcceptedGapWhileWaitIndicesNoOutlier]);
% RejectedWaitToCrossGaps = WCExpectedGap([indices.RejectedGapIndices]);
% 
% 
% 
% h = histogram(AllWaitToCrossGaps,[0:0.5:10]);
% Prob_GapDistribution = h.BinCounts/sum(h.BinCounts);
% 
% 
% h = histogram(AcceptedWaitToCrossGaps,[0:0.5:10]);
% Prob_AcceptedGapDistribution = h.BinCounts/sum(h.BinCounts);
% 
% 
% h = histogram(RejectedWaitToCrossGaps,[0:0.5:10]);
% Prob_RejectedGapDistribution = h.BinCounts/sum(h.BinCounts);
% 
% 
% 
% 
% 
% 
% 
% 
% AllGapsDecision = zeros(length(GapDataPredict{ii}),1);
% AllGapsDecision(indices.AcceptedGapWhileWaitIndicesNoOutlier)=1;
% 
% WCAllGapsDecision = [ones(N,1);zeros(M,1)];
% 
% 
% indices.WaitToCross = [indices.AcceptedGapWhileWaitIndicesNoOutlier;indices.RejectedGapIndices];
% 
% 
% % cross-validation set
% 
% c = cvpartition(WCAllGapsDecision,'KFold',k);
% 
% %% cross-validation loop
% 
% for ii=1:k
%     % training
%     tempIndicesTrain.WCAllGaps = indices.WaitToCross(c.training(k));
%     
%     temp = find(diff(tempIndicesTrain.WCAllGaps)<0,1,'first');
%     
%     %find indices of accepted and rejected gaps (indices are ordered,
%     %accepted gaps then rejected gaps)
%     tempIndicesTrain.WCAcceptedGaps = tempIndicesTrain.WCAllGaps(1:temp);
%     tempIndicesTrain.WCRejectedGaps = tempIndicesTrain.WCAllGaps(temp+1:end);
%     
%     % probabilities from training DataPredict{ii}
%     [N_Train(k),M_Train(k),Prob_GapAcceptance_Train(:,k),Prob_GapDistribution_Train(:,k),Prob_AcceptedGapDistribution_Train(:,k),...
%     Prob_RejectedGapDistribution_Train(:,k)] = WCProbabilityGaps(WCExpectedGap,tempIndicesTrain);
%     
%     
%     % testing
%     tempIndicesTest.WCAllGaps = indices.WaitToCross(c.test(k));   
%     temp = find(diff(tempIndicesTest.WCAllGaps)<0,1,'first');  
%     %find indices of accepted and rejected gaps
%     tempIndicesTest.WCAcceptedGaps = tempIndicesTest.WCAllGaps(1:temp);
%     tempIndicesTest.WCRejectedGaps = tempIndicesTest.WCAllGaps(temp+1:end);
%     
%     AllGapsTest = WCExpectedGap(tempIndicesTest.WCAllGaps);
%     AcceptedGapsTest = WCExpectedGap(tempIndicesTest.WCAcceptedGaps);
%     RejectedGapsTest = WCExpectedGap(tempIndicesTest.WCRejectedGaps);
%     %convert to groups
%     AcceptedGapsTest = floor(AcceptedGapsTest/0.5)+1;
%     RejectedGapsTest = floor(RejectedGapsTest/0.5)+1;
%     AllGapsTest = floor(AllGapsTest/0.5)+1;
%     
%     Prob_WCAcceptTest{k} = Prob_GapAcceptance_Train(:,k)*Prob_AcceptedGapDistribution_Train(AllGapsTest,k)./Prob_GapDistribution_Train(AllGapsTest,k);
%     Prob_WCRejectTest{k} = (1-Prob_GapAcceptance_Train(:,k))*Prob_RejectedGapDistribution_Train(AllGapsTest,k)./Prob_GapDistribution_Train(AllGapsTest,k);
%     
%     WCDecisionPred = Prob_WCAcceptTest{k}>=Prob_WCRejectTest{k};
%     WCDecisionActual = AllGapsDecision(tempIndicesTest.WCAllGaps);
%     
%     %Performance
%     [CVPerformance{k}] = probabilisticPredictionPerformance(WCDecisionActual,WCDecisionPred);
% 
%     
% end
% 
% 
% %% Time to reach road
% 
% TimetoReachRoad = (GapDataPredict{ii}(indices.Cross,9)-EventIndices(:,8))/10;
% 
% figure()
% h=histogram(RejectedGaps,[0:0.5:6]);
% 
% figure()
% h=histogram(AllGaps,[0:0.5:10]);
% %h=histogram(AllGaps,[0:0.5:10],'Normalization','probability');   %to plot probability density
% 
% figure()
% h=histogram(WCExpectedGap(indices.WaitActivityNotOutliers),[0:0.5:10]);
% %h=histogram(CombinedCrossingDataPredict{ii}(:,27),15,'Normalization','probability');   %to plot probability density
% 
% xlabel('Accepted Gaps [s]')
% ylabel('Probability')
% title('Gap acceptance')
% set(gca,'fontsize', 18)
% %To plot the distribution curve
% hold on;
% plot(conv(h.BinEdges, [0.5 0.5], 'valid'), h.BinCounts/sum(h.BinCounts), 'LineWidth',3)
% 
% figure()
% plot(conv(h.BinEdges, [0.5 0.5], 'valid'), cumsum(h.BinCounts/sum(h.BinCounts)),'-og','MarkerSize',10,'LineWidth',3);hold on
% 
% figure()
% plot(conv(h.BinEdges, [0.5 0.5], 'valid'), 1-cumsum(h.BinCounts/sum(h.BinCounts)),'-og','MarkerSize',10,'LineWidth',3);hold on
% 
% 









