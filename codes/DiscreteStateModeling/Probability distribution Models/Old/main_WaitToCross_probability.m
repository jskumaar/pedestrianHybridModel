clear all
close all

%% Updated on 05-12-2019:
%1. Expected Gap mat file includes the current gap and next gap; 
%2. extreme wait time threshold; added extreme wait thresholds and mild
%wait thresholds
%3. Cross gap have starting gap times instead of crossing gap times; 
%4. Performance: F1-57%; combined and individual 
%5. Using corrected DTCurb and DTCW; DTCurb calculated distance from one
%fixed poitn irrespective of direction of crossing; DTCW used distance
%between points, now using only x-distance between the CW point and
%pedestrian

%  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% inputs to adjust

% waitThreshold =52.2;     %extreme threshold, (3rd quartile + 3*interquartle range; all waiting crossing (N = 403))
% waitThreshold = 33.1;     %mild threshold, (3rd quartile + 1.5*interquartle range; all waiting crossing (N = 403))
waitThreshold = 30;     %comparison threshold,


Nk=5;        %cross-validation fold
out=1;              % 1 - include approach to wait/cross data
gazeInd = 1;        %1-10, 2-15, 3-20. 4-25, 5-30, gaze windows


%  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Read data

vehicleGapTimes = xlsread('VehicleGapTimesV6.xlsx');
GapData = xlsread('GapWiseCompiledDataV6.xlsx');
EventIndices = xlsread('DiscreteStateEventIndicesW5.xlsx');

% load('ExpectedGapData_W5_SpeedHist.mat')
% load('ExpectedGapData_W5_NewDTCurbDTCW_CrossingTimeforCrossingGaps.mat')
% load('ExpectedGapData_W5_OldDTCurbDTCW_CrossingTimeforCrossingGaps.mat')
load('ExpectedGapData_W5_NewDTCurbDTCW_StartingTimeforCrossingGaps.mat')

% load train and test indices
load('HybridModelTestTrainIndices.mat')


DataLength = size(ExpectedGapData,1);

%  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % Expected Gap - 1
% ExpectedGap = ExpectedGapData.WCExpectedGapStartGap;
% WCExpectedNextVehicleGapCrossStart = ExpectedGapData.WCExpectedNextVehicleGapCrossStart;
% 
% CrossIndices = find(ExpectedGapData.VehicleGapTimes(:,4)==2 & ExpectedGapData.WCExpectedGapStartGap<20 & ExpectedGapData.WCExpectedGapCrossStart<20 & ExpectedGapData.WCExpectedGapOnRoad<20);
% NotSameGapIndices = find(ExpectedGapData.VehicleGapTimes(:,4)==1);
% NotSameGapIndicesCheck = NotSameGapIndices+1;
% [CommonIndices,~,~] = intersect(CrossIndices,NotSameGapIndicesCheck);
% 
% ExpectedGap(CommonIndices) = WCExpectedNextVehicleGapCrossStart(CommonIndices);

% or directly from new Expected Gap .mat file

% ExpectedGap = ExpectedGapData.ExpectedGap_bothCurrentNextVehicle;
ExpectedGap = ExpectedGapData.WCExpectedGapStartGap;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % Expected Gap - 2
% ExpectedGap = ExpectedGapData.WCExpectedGapStartGapAcc;
% WCExpectedNextVehicleGapCrossStartAcc = ExpectedGapData.WCExpectedNextVehicleGapCrossStartAcc;
% 
% CrossIndices = find(ExpectedGapData.VehicleGapTimes(:,4)==2 & ExpectedGapData.WCExpectedGapStartGapAcc<20 & ExpectedGapData.WCExpectedGapCrossStartAcc<20 & ExpectedGapData.WCExpectedGapOnRoadAcc<20);
% NotSameGapIndices = find(ExpectedGapData.VehicleGapTimes(:,4)==1);
% NotSameGapIndicesCheck = NotSameGapIndices+1;
% [CommonIndices,~,~] = intersect(CrossIndices,NotSameGapIndicesCheck);
% 
% ExpectedGap(CommonIndices) = WCExpectedNextVehicleGapCrossStartAcc(CommonIndices);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% %% Expected Gap - 3 
% ExpectedGap = GapData(:,107);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Data - Dependent variables
GapType = ExpectedGap;
GazeType = ExpectedGapData.GazeRatiosGapStart(:,gazeInd);
SpeedType = ExpectedGapData.PedestrianAbsoluteVelocityAverage(:,gazeInd);
PedestrianDistancetoCW = ExpectedGapData.PedestrianDistancetoCW;
PedestrianDistancetoCurb = ExpectedGapData.PedestrianDistancetoCurb;
PedestrianCumulativeWaitTime = ExpectedGapData.VehicleGapTimes(:,10);

% do not change the order of the variables
DistributionData = [GapType,GazeType,SpeedType,PedestrianDistancetoCurb,PedestrianDistancetoCW,PedestrianCumulativeWaitTime];
DataBinSizes = [0.5,0.1,0.2,0.2,0.2,1];


startlimit = [0,0,0,0,0,0];
endlimit = [10,1,3,2.5,5,50];

% startlimit = min(DistributionData);
% endlimit = max(DistributionData);

% select the combination of observations (all observations cannot be
% included; state explosion; either bin size should be increased or number
% of observation variables should be reduced
N_obs = [1,3];
DistributionData = DistributionData(:,N_obs);
DataBinSizes = DataBinSizes(:,N_obs);

% Limits = Limits(:,N_obs);
startlimit = startlimit(:,N_obs);
endlimit = endlimit(:,N_obs);

%% Wait to cross indices
indices = WaitToCrossGapIndices_v2(GapData,EventIndices,waitThreshold,out);

% startlimit = min(DistributionData(indices.WCAllGaps,:));
% endlimit = max(DistributionData(indices.WCAllGaps,:));

% DataBinSizes = (endlimit-startlimit)/50;
%% combine observations;
% for certain uninterested indices, the observation type can be negative as
[ObservationType,DataBinned,Num_levels] = CombineObservations(DistributionData,DataBinSizes,startlimit,endlimit);


%% WC Probabilities from full data
for ii=1:size(DistributionData,2)
    [N,M,Prob_WCGapAcceptance,Prob_WCFullGapDistribution{ii},Prob_WCFullAcceptedGapDistribution{ii},...
    Prob_WCFullRejectedGapDistribution{ii}] = WCProbability(DistributionData(:,ii),indices,DataBinSizes(ii),startlimit(ii),endlimit(ii));
end
% combined observation probabilities from training data
[N,M,Prob_WCGapAcceptance,Prob_WCFullCombinedDistribution,Prob_AcceptedCombinedDistribution,...
Prob_RejectedCombinedDistribution] = WCProbability(ObservationType-1,indices,1,1,Num_levels);


% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% 
% 
%% Prediction loop
AllGapsDecision = zeros(DataLength,1);
AllGapsDecision(indices.WCAcceptedGaps)=1;

% cross-validation set
% order, 1st accepted indice, next rejected indices; beacuse this is the
% order from 'WaitToCrossGapIndices' function
WCAllGapsForCVPartition = [ones(length(indices.WCAcceptedGaps),1);zeros(length(indices.WCRejectedGaps),1)];
c = cvpartition(WCAllGapsForCVPartition,'KFold',Nk);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Approch prediction
% indices.ApproachWCRejectedGaps = indices.FirstWait(indices.FirstWait~=0);
% indices.ApproachWCAcceptedGaps = indices.Gap_AcceptedGapsWhileApproachIndices;
% indices.ApproachWCAllGaps = [indices.ApproachWCAcceptedGaps;indices.ApproachWCRejectedGaps];
% 
% AllGapsDecision = zeros(DataLength,1);
% AllGapsDecision(indices.ApproachWCAcceptedGaps)=1;
% 
% ApproachWCAllGapsDecision = [ones(length(indices.ApproachWCAcceptedGaps),1);zeros(length(indices.ApproachWCRejectedGaps),1)];
% 
% % cross-validation set
% Nk=1;
% 
% tempIndicesTest.WCAllGaps = indices.ApproachWCAllGaps;  
% Prob_WC = WaitToCrossTrain(GapData(indices.WCAllGaps,:),DistributionData(indices.WCAllGaps,:),EventIndices,waitThreshold,out,binsize,Gazebinsize);
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%% cross-validation loop
CombinedAccuracy = [];
CombinedPrecision = [];
CombinedRecall = [];
CombinedF1Score = [];
IndividualAccuracy = cell(size(DistributionData,2),1);
IndividualPrecision = cell(size(DistributionData,2),1);
IndividualRecall = cell(size(DistributionData,2),1);
IndividualF1Score = cell(size(DistributionData,2),1);


for ii=1:Nk
    %% training
    tempIndicesTrain.WCAllGaps = sort(indices.WCAllGaps(c.training(ii)));   
    Prob_WC = WaitToCrossTrain(GapData(tempIndicesTrain.WCAllGaps,:),DistributionData(tempIndicesTrain.WCAllGaps,:),EventIndices,waitThreshold,out,DataBinSizes,startlimit,endlimit);

 
    %% testing
    tempIndicesTest.WCAllGaps = indices.WCAllGaps(c.test(ii));
    
  
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    temp = find(diff(tempIndicesTest.WCAllGaps)<0,1,'first'); 
    
    %find indices of accepted and rejected gaps
    tempIndicesTest.WCAcceptedGaps = tempIndicesTest.WCAllGaps(1:temp);
    
    % Actual decision
    WCDecisionActual = AllGapsDecision(tempIndicesTest.WCAllGaps);
    
    %% Individual observation Test data
    AllIndividualDataTest = DistributionData(tempIndicesTest.WCAllGaps,:);
    
    %% combine observations test data
    AllCombinedTest = ObservationType(tempIndicesTest.WCAllGaps);

    %% individual observations   
    for jj=1:size(DistributionData,2)
        AllGapsTest = AllIndividualDataTest(:,jj);
        
%         AllGapsTest = min(floor(AllGapsTest/DataBinSizes(jj))+1,floor(Limits(jj)/DataBinSizes(jj)));
       
        AllGapsTest = min(ceil((AllGapsTest-startlimit(jj))/DataBinSizes(jj))+1,ceil((endlimit(jj)-startlimit(jj))/DataBinSizes(jj)));
 
        Prob_WCAcceptTest_IndividualData{ii}{jj} = Prob_WC.AcceptedGap*Prob_WC.AcceptedGapDistribution_Train{jj}(AllGapsTest)./Prob_WC.GapDistribution_Train{jj}(AllGapsTest);      
        WCDecisionIndividualDataPred=[];
        WCDecisionIndividualDataPred(:,1) = Prob_WCAcceptTest_IndividualData{ii}{jj}>=0.5;
        
        %Performance
        [IndividualObservation_CVPerformance{ii}{jj}] = probabilisticPredictionPerformance(WCDecisionActual,WCDecisionIndividualDataPred);
        
        
        %% compile performances
        IndividualAccuracy{jj} = [ IndividualAccuracy{jj}; IndividualObservation_CVPerformance{ii}{jj}.Accuracy];
        IndividualPrecision{jj} = [ IndividualPrecision{jj}; IndividualObservation_CVPerformance{ii}{jj}.Precision];
        IndividualRecall{jj} = [ IndividualRecall{jj}; IndividualObservation_CVPerformance{ii}{jj}.Recall];
        IndividualF1Score{jj} = [ IndividualF1Score{jj}; IndividualObservation_CVPerformance{ii}{jj}.F1Score];
        
        IndividualAccuracy{jj} = nanmean(IndividualAccuracy{jj});
        IndividualPrecision{jj}= nanmean(IndividualPrecision{jj});
        IndividualRecall{jj} = nanmean(IndividualRecall{jj});
        IndividualF1Score{jj} = nanmean(IndividualF1Score{jj});
        IndividualPerformance{jj} = [IndividualAccuracy{jj};IndividualPrecision{jj};IndividualRecall{jj};IndividualF1Score{jj}];
              
    end  
    
    %% all combined observations       
        AllCombinedTest = max(AllCombinedTest,1);
        Prob_WCAcceptTest_Combined{ii} = Prob_WC.AcceptedGap*Prob_WC.AcceptedCombinedDistribution_Train(AllCombinedTest)./Prob_WC.CombinedDistribution_Train(AllCombinedTest);        
        WCDecisionCombinedPred = Prob_WCAcceptTest_Combined{ii}>=0.5;
        
                    
        %Performance
        [CombinedCVPerformance{ii}] = probabilisticPredictionPerformance(WCDecisionActual,WCDecisionCombinedPred);

    %% compile performances
        CombinedAccuracy = [CombinedAccuracy;CombinedCVPerformance{ii}.Accuracy];
        CombinedPrecision = [CombinedPrecision;CombinedCVPerformance{ii}.Precision];
        CombinedRecall = [CombinedRecall;CombinedCVPerformance{ii}.Recall];
        CombinedF1Score = [CombinedF1Score;CombinedCVPerformance{ii}.F1Score];
        
        
end


CombinedAccuracy = nanmean(CombinedAccuracy);
CombinedPrecision = nanmean(CombinedPrecision);
CombinedRecall = nanmean(CombinedRecall);
CombinedF1Score = nanmean(CombinedF1Score);
CombinedPerformance = [CombinedAccuracy;CombinedPrecision;CombinedRecall;CombinedF1Score];





