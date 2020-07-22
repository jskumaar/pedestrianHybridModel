%% Expected gap (velocity and acceleration fo when the gap starts before they cross/decide to wait)


clear all


load('AllFeaturesCrossingWise.mat');


vehicleGapTimes = xlsread('VehicleGapTimesV6.xlsx');
GapDataAWC = xlsread('GapWiseCompiledDataV6.xlsx',2);
GapDataWC = xlsread('GapWiseCompiledDataV6.xlsx',1);
GapStart = GapDataAWC(:,5);
EventIndices = xlsread('DiscreteStateEventIndicesW5.xlsx');
ApproachStart = EventIndices(:,4);
RetreatEnd = EventIndices(:,11);


%% Time gap - Dependent variable
ExpectedGap = GapDataAWC(:,5);
ExpectedGapAcc = GapDataAWC(:,6);
PedSpeed = GapDataAWC(:,7);

%% Gaze Data - Dependent variable
GazeRatiosGapStart = GapDataAWC(:,8:12);

%% inputs
Nk=5;        %cross-validation fold
out=1;
waitThreshold = 300;

binsize = 0.5;
Gazebinsize = 0.2;
Speedbinsize = 0.5;
GapInd = 1;     %1-4 values
GazeInd = 1;    %1-5 values

GapType = ExpectedGap(:,GapInd);
% GapType = ExpectedGapAcc(:,GapInd);


GazeType = GazeRatiosGapStart(:,5*(GazeInd-1)+GapInd);
SpeedType = PedSpeed;



%indices of vehicle gaps for each crossing
ind = find(diff(vehicleGapTimes(:,3))~=0);
indStart = [1;ind+1];
indEnd = [ind;length(vehicleGapTimes)];


% % find crossing indices that wait or cross without waiting
% indCross = find(vehicleGapTimes(:,4)==2);
% cumWait = vehicleGapTimes(indCross,10);
% indCrossingApproachtoCross = find(cumWait==0);
% indCrossingApproachtoWait = find(cumWait~=0);


% 
% for kk=1:30
%     for jj=1:3
%         
%         for ii=1:6
%             ind = 18*(kk-1)+6*(jj-1)+ii;
%             check = intersect(ind,indCrossingApproachtoCross);
%             
%             if ~isempty(check) %crossing where pedestrian directly crosses without waiting for that crossing
%                 PedIndex(ind,1) = GapStart(indEnd(ind))-ApproachStart(ind)+1;
%             else
%                 indWait = find(GapData(indStart(ind):indEnd(ind),11)==2,1,'first');
%                 
%                 if isempty(indWait)
%                     indWait=indEnd(ind)-indStart(ind)+1;
%                 end
%       
%                 PedIndex(ind,1) = GapStart(indStart(ind)+indWait-1)-ApproachStart(ind)+1; 
%               
%             end
%             
%             if PedIndex(ind,1)<=0
%                 PedIndex(ind,1)=1;
%             end
%             
%             
%             ExpectedGap(ind,1) = DataPredict{ind}.VehicleTimeGaptoPedestrian(PedIndex(ind));
%             % acceleration based gap when starting to cross
%             syms x;
%             soln = double(real(solve(-DataPredict{ind}.VehicleAcceleration(PedIndex(ind))/2*x^2 - DataPredict{ind}.VehicleSpeed(PedIndex(ind))*x...
%                                                 -DataPredict{ind}.PedestrianVehicleDistance(PedIndex(ind))==0,x,'IgnoreAnalyticConstraints', true)));  
% 
%             [~,temp] = min(abs(soln-ExpectedGap(ind)));
%             if ~isempty(temp)
%                 ExpectedGapAcc(ind,1) = soln(temp);
%             else
%                 ExpectedGapAcc(ind,1) = ExpectedGap(ind,1);
%             end
% 
%             AWCGazeRatio(ind,:) = DataPredict{ind}.GazeAtVehicleRatio(PedIndex(ind,1),:);
%             PedSpeed(ind,1) = DataPredict{ind}.PedestrianAbsoluteVelocity(PedIndex(ind,1));
%             
%       
%         end
%     end
% end



%% Wait to cross indices
indices = WaitToCrossGapIndices(GapDataWC,EventIndices,waitThreshold,out);

%% Approach to cross indices
indices.AWCWaitGaps = find(GapDataAWC(:,4)==0);
indices.AWCCrossGaps = find(GapDataAWC(:,4)==1);
indices.AWCAllGaps = [indices.AWCWaitGaps;indices.AWCCrossGaps];
AWCAllGapsDecision = GapDataAWC(indices.AWCAllGaps,4);
% AllGapsDecision = GapData(:,7);


%% ApproachWC Probabilities from full data
[N,M,Prob_AWCGapWaitance,Prob_AWCFullGapDistribution,Prob_AWCFullWaitedGapDistribution,...
Prob_AWCFullCrossedGapDistribution] = ApproachWCProbability(GapType,indices,binsize,10);

%% Wait to cross Probabilities from full data
Prob_WC = WaitToCrossTrain(GapDataWC,EventIndices);

%% Prediction loop

% cross-validation set
c = cvpartition(AWCAllGapsDecision,'KFold',Nk);

%% cross-validation loop
Accuracy = [];
Precision = [];
Recall = [];
F1Score = [];
GazeAccuracy = [];
GazePrecision = [];
GazeRecall = [];
GazeF1Score = [];
GapAccuracy = [];
GapPrecision = [];
GapRecall = [];
GapF1Score = [];
SpeedAccuracy = [];
SpeedPrecision = [];
SpeedRecall = [];
SpeedF1Score = [];

for ii=1:Nk
    % training
    tempIndicesTrain.AWCAllGaps = indices.AWCAllGaps(c.training(ii));
    
    temp = find(diff(tempIndicesTrain.AWCAllGaps)<0,1,'first');
    
    %find indices of wait and cross gaps (indices are ordered,
    %Waited gaps then Crossed gaps)
    tempIndicesTrain.AWCWaitGaps = tempIndicesTrain.AWCAllGaps(1:temp);
    tempIndicesTrain.AWCCrossGaps = tempIndicesTrain.AWCAllGaps(temp+1:end);
    
%     % gap probabilities from training data
%     [N_Train(ii),M_Train(ii),Prob_GapCross_Train(:,ii),Prob_GapDistribution_Train(:,ii),Prob_WaitedGapDistribution_Train(:,ii),...
%     Prob_CrossedGapDistribution_Train(:,ii)] = ApproachWCProbability(GapType,tempIndicesTrain,binsize,10);
%     
% 
%     % gaze probabilities from training data
%     [N_GazeTrain(ii),M_GazeTrain(ii),Prob_GazeCross_Train(:,ii),Prob_GazeDistribution_Train(:,ii),Prob_WaitedGazeDistribution_Train(:,ii),...
%     Prob_CrossedGazeDistribution_Train(:,ii)] = ApproachWCProbability(GazeType,tempIndicesTrain,Gazebinsize,1);
% 
% 
%     % speed probabilities from training data
%     [N_SpeedTrain(ii),M_SpeedTrain(ii),Prob_SpeedCross_Train(:,ii),Prob_SpeedDistribution_Train(:,ii),Prob_WaitedSpeedDistribution_Train(:,ii),...
%     Prob_CrossedSpeedDistribution_Train(:,ii)] = ApproachWCProbability(SpeedType,tempIndicesTrain,Speedbinsize,3);
    






    
    % testing
    tempIndicesTest.AWCAllGaps = indices.AWCAllGaps(c.test(ii));   
    temp = find(diff(tempIndicesTest.AWCAllGaps)<0,1,'first');  
    %find indices of Waited and Crossed gaps
    tempIndicesTest.AWCCrossGaps = tempIndicesTest.AWCAllGaps(temp+1:end);
    
    AllGapsTest = GapType(tempIndicesTest.AWCAllGaps);   
    AllGazeTest = GazeType(tempIndicesTest.AWCAllGaps);    
    AllSpeedTest = SpeedType(tempIndicesTest.AWCAllGaps);

%   %convert to groups
    AllGapsTest = min(floor(AllGapsTest/binsize)+1,floor(10/binsize));
    AllGazeTest = min(floor(AllGazeTest/Gazebinsize)+1,floor(1/Gazebinsize));
    AllSpeedTest = min(floor(AllSpeedTest/Speedbinsize)+1,floor(3/Speedbinsize));


    Prob_WCCrossTest_Gap{ii} = Prob_GapCross_Train(:,ii)*Prob_CrossedGapDistribution_Train(AllGapsTest,ii)./Prob_GapDistribution_Train(AllGapsTest,ii);
    Prob_WCCrossTest_Gaze{ii} = Prob_GazeCross_Train(:,ii)*Prob_CrossedGazeDistribution_Train(AllGazeTest,ii)./Prob_GazeDistribution_Train(AllGazeTest,ii);
    Prob_WCCrossTest_Speed{ii} = Prob_SpeedCross_Train(:,ii)*Prob_CrossedSpeedDistribution_Train(AllSpeedTest,ii)./Prob_SpeedDistribution_Train(AllSpeedTest,ii);      

    Prob_WCCrossTest{ii} = Prob_WCCrossTest_Gap{ii}.*Prob_WCCrossTest_Gaze{ii}.*Prob_WCCrossTest_Speed{ii};


    WCDecisionPred = Prob_WCCrossTest{ii}>=0.5;
    WCDecisionGazePred = Prob_WCCrossTest_Gaze{ii}>=0.5;
    WCDecisionGapPred = Prob_WCCrossTest_Gap{ii}>=0.5;
    WCDecisionSpeedPred = Prob_WCCrossTest_Speed{ii}>=0.5;

    WCDecisionActual = AWCAllGapsDecision(tempIndicesTest.AWCAllGaps);

    %Performance
    [CVPerformance{ii}] = probabilisticPredictionPerformance(WCDecisionActual,WCDecisionPred);
    [GazeCVPerformance{ii}] = probabilisticPredictionPerformance(WCDecisionActual,WCDecisionGazePred);
    [GapCVPerformance{ii}] = probabilisticPredictionPerformance(WCDecisionActual,WCDecisionGapPred);
    [SpeedCVPerformance{ii}] = probabilisticPredictionPerformance(WCDecisionActual,WCDecisionSpeedPred);


    Accuracy = [Accuracy;CVPerformance{ii}.Accuracy];
    Precision = [Precision;CVPerformance{ii}.Precision];
    Recall = [Recall;CVPerformance{ii}.Recall];
    F1Score = [F1Score;CVPerformance{ii}.F1Score];


    GazeAccuracy = [ GazeAccuracy; GazeCVPerformance{ii}.Accuracy];
    GazePrecision = [ GazePrecision; GazeCVPerformance{ii}.Precision];
    GazeRecall = [ GazeRecall; GazeCVPerformance{ii}.Recall];
    GazeF1Score = [ GazeF1Score; GazeCVPerformance{ii}.F1Score];

    GapAccuracy = [GapAccuracy;GapCVPerformance{ii}.Accuracy];
    GapPrecision = [GapPrecision;GapCVPerformance{ii}.Precision];
    GapRecall = [GapRecall;GapCVPerformance{ii}.Recall];
    GapF1Score = [GapF1Score;GapCVPerformance{ii}.F1Score];        


    SpeedAccuracy = [ SpeedAccuracy; SpeedCVPerformance{ii}.Accuracy];
    SpeedPrecision = [ SpeedPrecision; SpeedCVPerformance{ii}.Precision];
    SpeedRecall = [ SpeedRecall; SpeedCVPerformance{ii}.Recall];
    SpeedF1Score = [ SpeedF1Score; SpeedCVPerformance{ii}.F1Score];


end


Accuracy = nanmean(Accuracy);
Precision = nanmean(Precision);
Recall = nanmean(Recall);
F1Score = nanmean(F1Score);
Performance = [Accuracy;Precision;Recall;F1Score];

GazeAccuracy = nanmean(GazeAccuracy);
GazePrecision = nanmean(GazePrecision);
GazeRecall = nanmean(GazeRecall);
GazeF1Score = nanmean(GazeF1Score);
GazePerformance = [GazeAccuracy;GazePrecision;GazeRecall;GazeF1Score];

GapAccuracy = nanmean(GapAccuracy);
GapPrecision = nanmean(GapPrecision);
GapRecall = nanmean(GapRecall);
GapF1Score = nanmean(GapF1Score);
GapPerformance = [GapAccuracy;GapPrecision;GapRecall;GapF1Score];

SpeedAccuracy = nanmean(SpeedAccuracy);
SpeedPrecision = nanmean(SpeedPrecision);
SpeedRecall = nanmean(SpeedRecall);
SpeedF1Score = nanmean(SpeedF1Score);
SpeedPerformance = [SpeedAccuracy;SpeedPrecision;SpeedRecall;SpeedF1Score];


OverallPerformance = [GapPerformance,GazePerformance,SpeedPerformance,Performance];