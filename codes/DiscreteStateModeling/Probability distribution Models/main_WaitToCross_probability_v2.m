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

%% Updated on 05-14-2019:
% 1. Simplified the indices calculation; made it external and fixed data
% sets
%  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% inputs to adjust
out=1;              % 1 - include approach to wait/cross data
%  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% %% Read data
load('SVMTrainData_ExtremeOutlier_StartGapExpectedGap_CorrectDTCurbDTCW.mat')
load('SVMTestData_ExtremeOutlier_StartGapExpectedGap_CorrectDTCurbDTCW.mat')

% load('SVMTrainData_ExtremeOutlier_WaitStartGapExpectedGap_OldDTCurbDTCW.mat')
% load('SVMTestData_ExtremeOutlier_WaitStartGapExpectedGap_OldDTCurbDTCW.mat')


%  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Expected Gap - 1

% ExpectedGap = SVMTrainData.ExpectedGap_bothCurrentNextVehicle;
ExpectedGap = SVMTrainData.WCExpectedGapStartGap;


%% Data - Dependent variables
GapType = ExpectedGap;
GazeType = SVMTrainData.GazeRatiosGapStart;
SpeedType = SVMTrainData.PedestrianAbsoluteVelocityAverage;
PedestrianDistancetoCW = SVMTrainData.PedestrianDistancetoCW;
PedestrianDistancetoCurb = SVMTrainData.PedestrianDistancetoCurb;
PedestrianCumulativeWaitTime = SVMTrainData.PedestrianCumulativeWaitTime;
DiscreteState = SVMTrainData.DiscreteState;

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
N_obs = [1,3,6];
DistributionData = DistributionData(:,N_obs);
DataBinSizes = DataBinSizes(:,N_obs);

% Limits = Limits(:,N_obs);
startlimit = startlimit(:,N_obs);
endlimit = endlimit(:,N_obs);

%% Wait to cross indices
indices.WCAllGaps = [1:size(SVMTrainData)];
indices.WCAcceptedGaps = find(SVMTrainData.WCAllGapsDecision_CrossDecisionOnRoadGap==1);
indices.WCRejectedGaps = find(SVMTrainData.WCAllGapsDecision_CrossDecisionOnRoadGap==0);


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
Prob_RejectedCombinedDistribution] = WCProbability(ObservationType,indices,1,1,Num_levels);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Evaluation loop
IndividualAccuracy = [];
IndividualPrecision = [];
IndividualRecall = [];
IndividualF1Score = [];
 
%% testing
GapTypeTest  = SVMTestData.WCExpectedGapStartGap;
GazeTypeTest  = SVMTestData.GazeRatiosGapStart;
SpeedTypeTest  = SVMTestData.PedestrianAbsoluteVelocityAverage;
PedestrianDistancetoCWTest  = SVMTestData.PedestrianDistancetoCW;
PedestrianDistancetoCurbTest  = SVMTestData.PedestrianDistancetoCurb;
PedestrianCumulativeWaitTimeTest  = SVMTestData.PedestrianCumulativeWaitTime;
DiscreteStateTest = SVMTestData.DiscreteState; 


% do not change the order of the variables
DistributionDataTest = [GapTypeTest,GazeTypeTest,SpeedTypeTest,PedestrianDistancetoCurbTest,PedestrianDistancetoCWTest,PedestrianCumulativeWaitTimeTest];
DistributionDataTest = DistributionDataTest(:,N_obs);

[ObservationTypeTest,DataBinnedTest,Num_levels] = CombineObservations(DistributionDataTest,DataBinSizes,startlimit,endlimit);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   

TestIndices.AllGaps = [1:size(SVMTestData)];
TestIndices.AcceptedGaps = find(SVMTestData.WCAllGapsDecision_CrossDecisionOnRoadGap==1);
TestIndices.RejectedGaps = find(SVMTestData.WCAllGapsDecision_CrossDecisionOnRoadGap==0);
    

    %% individual observations   
    for jj=1:size(DistributionData,2)
        
        AllGapsTest = DataBinnedTest(:,jj);
        WCDecisionActual = SVMTestData.WCAllGapsDecision_CrossDecisionOnRoadGap;
               
        AllGapsTest(AllGapsTest<1)=1;
        c = max(length(Prob_WCFullGapDistribution{jj}));
        AllGapsTest(AllGapsTest>c) = c ;
        
        Prob_WCAcceptTest_IndividualData{jj} = Prob_WCGapAcceptance*Prob_WCFullAcceptedGapDistribution{jj}(AllGapsTest)./Prob_WCFullGapDistribution{jj}(AllGapsTest);      
        WCDecisionIndividualDataPred=[];
        WCDecisionIndividualDataPred(:,1) = Prob_WCAcceptTest_IndividualData{jj}>=0.5;
        
        %Performance
        [IndividualObservation_CVPerformance{jj}] = probabilisticPredictionPerformance(WCDecisionActual,WCDecisionIndividualDataPred);
        
        
        %% compile performances
        IndividualAccuracy = [ IndividualAccuracy; IndividualObservation_CVPerformance{jj}.Accuracy];
        IndividualPrecision = [ IndividualPrecision; IndividualObservation_CVPerformance{jj}.Precision];
        IndividualRecall = [ IndividualRecall; IndividualObservation_CVPerformance{jj}.Recall];
        IndividualF1Score = [ IndividualF1Score; IndividualObservation_CVPerformance{jj}.F1Score];        
    end  
  
%% all combined observations       
AllCombinedTest = ObservationTypeTest;
Prob_WCAcceptTest_Combined = Prob_WCGapAcceptance*Prob_AcceptedCombinedDistribution(AllCombinedTest)./Prob_WCFullCombinedDistribution(AllCombinedTest);        
WCDecisionCombinedPred = Prob_WCAcceptTest_Combined>=0.5;


%Performance
[CombinedCVPerformance] = probabilisticPredictionPerformance(WCDecisionActual,WCDecisionCombinedPred);

%% compile performances
CombinedAccuracy = [CombinedCVPerformance.Accuracy];
CombinedPrecision = [CombinedCVPerformance.Precision];
CombinedRecall = [CombinedCVPerformance.Recall];
CombinedF1Score = [CombinedCVPerformance.F1Score];
        





