%% script to calculate the proces variance

clear all



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%1) calculate process variance for each action/discrete state
%2) assume filtered position and velocities as ground truth
%3) use the measured position and temporal difference as velocities and use
%them to identify the measurement noise

%use only the allowable crossings (523)

load('HybridModelTestTrainIndices.mat')
load('AllFeaturesCrossingWise_PW_5_SpeedHist_CorrectDTCurbDTCW.mat');
EventIndices = xlsread('DiscreteStateEventIndicesW5.xlsx');
ApproachStart = EventIndices(:,4);

indices = unique([TestIndices_woExtremeOutlier;TrainIndices_woExtremeOutlier]);


% dt=0.1;
% 
% F = [1 0 dt 0;
%      0 1 0 dt;
%      0 0 1 0;
%      0 0 0 1];  
% 
% ActualStates = [];
% DiscreteStates = [];
% PredictedStates = [];
% 
% for ii=1:length(indices)    
%     ind = indices(ii);
%     
%     PedState = [DataPredict{ind}.PedestrianPosition,DataPredict{ind}.PedestrianCartesianVelocity];
%     
%     ActualStates = [ActualStates;PedState];
%     DiscreteStates = [DiscreteStates;DataPredict{ind}.PedestrianDiscreteState];
%     
%     PredictedStates = [PredictedStates;PedState(1,:)];   
%     for jj=2:size(DataPredict{ind},1)
%         PredictedStates = [PredictedStates;[F*PedState(jj-1,:)']'];
%     end
%     
% end
% 
% 
% Error.Complete = ActualStates - PredictedStates;
% 
% ApproachIndex = find(DiscreteStates==1);
% WaitIndex = find(DiscreteStates==2);
% CrossIndex = find(DiscreteStates==3);
% RetreatIndex = find(DiscreteStates==4);
% 
% Error.ApproachError = abs(Error.Complete(ApproachIndex,:));
% Error.WaitError = abs(Error.Complete (WaitIndex,:));
% Error.CrossError = abs(Error.Complete (CrossIndex,:));
% Error.RetreatError = abs(Error.Complete (RetreatIndex,:));
% 
% ProcessNoise.Approach = (Error.ApproachError'*Error.ApproachError)/length(Error.ApproachError);
% ProcessNoise.Wait = (Error.WaitError'*Error.WaitError)/length(Error.WaitError);
% ProcessNoise.Cross = (Error.CrossError'*Error.CrossError)/length(Error.CrossError);
% ProcessNoise.Retreat = (Error.RetreatError'*Error.RetreatError)/length(Error.RetreatError);
% ProcessNoise.Overall = (Error.Complete'*Error.Complete)/length(Error.Complete);
% 
% 
% save('CVProcess_covariance_variables.mat','ActualStates','DiscreteStates','PredictedStates','Error','ProcessNoise')
% save('CVProcess_Noise_matrix.mat','ProcessNoise')




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%1) calculate the distribution of the target locations during wait and
%cross

% ApproachStart = EventIndices(:,4);
% WaitStart = EventIndices(:,6)-ApproachStart+1;
% CrossEnd = EventIndices(:,9)-ApproachStart+1;
% RetreatEnd = EventIndices(:,11)-ApproachStart+1;
% Waitcount = 1;
% Crosscount = 1;
% 
% for ii=1:length(indices)
%     Crossind = indices(ii);
%      
%     if WaitStart(Crossind)>0
%         WaitPosition(Waitcount,:) = DataPredict{Crossind}.PedestrianPosition(WaitStart(Crossind),:);
%         Waitcount = Waitcount+1;
%     end    
%     CrossPosition(ii,:) = DataPredict{Crossind}.PedestrianPosition(CrossEnd(Crossind),:);
% 
% end
% figure()
% scatter(WaitPosition(:,1),WaitPosition(:,2))
% 
% figure()
% scatter(CrossPosition(:,1),CrossPosition(:,2))
% 
% 
% mean(WaitPosition(WaitPosition(:,2)>0,:))
% mean(WaitPosition(WaitPosition(:,2)<0,:))
% 
% mean(CrossPosition(CrossPosition(:,2)>0,:))
% mean(CrossPosition(CrossPosition(:,2)<0,:))




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% %1) calculate the wait start from gap rejection time from the data
% % 2) continue with fitting the distribution later
% 
% load('SVMData_ExtremeOutlier_StartGapExpectedGap_CorrectDTCurbDTCW.mat')
% load('ExpectedGapData_W5_NewDTCurbDTCW_StartingTimeforCrossingGaps.mat')
% 
% GapInd = [];
% for ii=1:length(indices)
%     ind = indices(ii);
%     temp = find(ExpectedGapData.CrossingNumber==ind);
%     GapInd = [GapInd;temp];
% end
% 
% 
% temp = find(ExpectedGapData.VehicleGapTimes(GapInd,11)==2);
% 
% TimeToWaitStartInd = GapInd(temp);
% TimeToWaitStart = (ExpectedGapData.VehicleGapTimes(TimeToWaitStartInd,7) - ExpectedGapData.VehicleGapTimes(TimeToWaitStartInd,5))/10;
% 
% figure()
% hist(TimeToWaitStart,15)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%1) calculate the pedestrian position when appraoch to wait gap/approach to
%cross gap starts 

load('ExpectedGapData_W5_NewDTCurbDTCW_StartingTimeforCrossingGaps.mat')
count=1;

% GapIndicesNoOutliers = [];
% 
% for ii=1:length(indices)
%     ind = indices(ii);
%     temp(1,:) = find(ExpectedGapData.CrossingNumber==(ind));
%     GapIndicesNoOutliers = [GapIndicesNoOutliers;tremp];    
% end






for ii=1:length(indices)
    
    ind = indices(ii);    
    temp = find(ExpectedGapData.CrossingNumber==ind);
    
    temp2 = find(ExpectedGapData.DiscreteState(temp)==2,1,'first');
    if isempty(temp2)
        temp3 = find(ExpectedGapData.DiscreteState(temp)==3,1,'first');  
    else
        temp3 = [];
    end
    
    
    
    
    
    
%     correctGapInd = temp(temp2);


%     if ~isempty(temp3)
        correctGapInd = temp(temp3);
        pedIndex = ExpectedGapData.VehicleGapTimes(correctGapInd,5)-ApproachStart(ind);

        if pedIndex>0
            PedestrianPositionApproachGapEvaluate(count,:) = [ind,DataPredict{ind}.PedestrianPosition(pedIndex,:)];
            count=count+1;
        end
%     end

                
end


mean(abs(PedestrianPositionApproachGapEvaluate))
std(abs(PedestrianPositionApproachGapEvaluate))
max(abs(PedestrianPositionApproachGapEvaluate))
min(abs(PedestrianPositionApproachGapEvaluate))
prctile(abs(PedestrianPositionApproachGapEvaluate),95)



