%% Expected gap (velocity and acceleration fo when the gap starts before they cross/decide to wait)


clear all


vehicleGapTimes = xlsread('VehicleGapTimesV6.xlsx');
% GapDataAWC = xlsread('GapWiseCompiledDataV6.xlsx',2);
% GapDataWC = xlsread('GapWiseCompiledDataV6.xlsx',1);
% GapStart = GapDataAWC(:,5);

EventIndices = xlsread('DiscreteStateEventIndicesW5.xlsx');
% EventIndices = xlsread('DiscreteStateEventIndicesW11_onlyforMLmodel_DONT_USE.xlsx');

ApproachStart = EventIndices(:,4);
RetreatEnd = EventIndices(:,11);

load('AllFeaturesCrossingWise_PW_11.mat')


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
Speedbinsize = 0.2;
GapInd = 1;     %1-4 values
GazeInd = 1;    %1-5 values

GapType = ExpectedGap(:,GapInd);
%GapType = ExpectedGapAcc(:,GapInd);

GazeType = GazeRatiosGapStart(:,5*(GazeInd-1)+GapInd);
SpeedType = PedSpeed;


%% Wait to cross indices
indices = WaitToCrossGapIndices(GapDataWC,EventIndices,waitThreshold,out);

%% Approach to cross indices
indices.AWCWaitGaps = find(GapDataAWC(:,4)==0);
indices.AWCCrossGaps = find(GapDataAWC(:,4)==1);
indices.AWCAllGaps = [indices.AWCWaitGaps;indices.AWCCrossGaps];
AWCAllGapsDecision = GapDataAWC(indices.AWCAllGaps,4);


%% ApproachWC Probabilities from full data
Prob_AWC = ApproachToWaitCrossTrain(GapDataAWC);

%% Wait to cross Probabilities from full data
Prob_WC = WaitToCrossTrain(GapDataWC,EventIndices);

%% Prediction loop

% % cross-validation set
% c = cvpartition(AWCAllGapsDecision,'KFold',Nk);

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

for ii=1:1
% for checking with wait to cross probabilities
    AllGapsTest = GapType(indices.AWCAllGaps);   
    AllGazeTest = GazeType(indices.AWCAllGaps);    
    AllSpeedTest = SpeedType(indices.AWCAllGaps);


%   %convert to groups
    AllGapsTest = min(floor(AllGapsTest/binsize)+1,floor(10/binsize));
    AllGazeTest = min(floor(AllGazeTest/Gazebinsize)+1,floor(1/Gazebinsize));
    AllSpeedTest = min(floor(AllSpeedTest/Speedbinsize)+1,floor(3/Speedbinsize));

    
    Prob_WCCrossTest_Gap{ii} = Prob_WC.AcceptedGap*Prob_WC.AcceptedGapDistribution_Train(AllGapsTest)./Prob_WC.GapDistribution_Train(AllGapsTest);
    

%     Prob_WCCrossTest_Gap{ii} = Prob_GapCross_Train(:,ii)*Prob_CrossedGapDistribution_Train(AllGapsTest,ii)./Prob_GapDistribution_Train(AllGapsTest,ii);
%     Prob_WCCrossTest_Gaze{ii} = Prob_GazeCross_Train(:,ii)*Prob_CrossedGazeDistribution_Train(AllGazeTest,ii)./Prob_GazeDistribution_Train(AllGazeTest,ii);
    %Prob_WCCrossTest_Speed{ii} = Prob_SpeedCross_Train(:,ii)*Prob_CrossedSpeedDistribution_Train(AllSpeedTest,ii)./Prob_SpeedDistribution_Train(AllSpeedTest,ii);      

    %Prob_WCCrossTest{ii} = Prob_WCCrossTest_Gap{ii}.*Prob_WCCrossTest_Gaze{ii}.*Prob_WCCrossTest_Speed{ii};

% 
%     WCDecisionPred = Prob_WCCrossTest{ii}>=0.5;
%     WCDecisionGazePred = Prob_WCCrossTest_Gaze{ii}>=0.5;
    WCDecisionGapPred = Prob_WCCrossTest_Gap{ii}>=0.5;
   % WCDecisionSpeedPred = Prob_WCCrossTest_Speed{ii}>=0.5;

    WCDecisionActual = AWCAllGapsDecision(indices.AWCAllGaps);

    %Performance
%     [CVPerformance{ii}] = probabilisticPredictionPerformance(WCDecisionActual,WCDecisionPred);
%     [GazeCVPerformance{ii}] = probabilisticPredictionPerformance(WCDecisionActual,WCDecisionGazePred);
    [GapCVPerformance{ii}] = probabilisticPredictionPerformance(WCDecisionActual,WCDecisionGapPred);
   % [SpeedCVPerformance{ii}] = probabilisticPredictionPerformance(WCDecisionActual,WCDecisionSpeedPred);

% 
%     Accuracy = [Accuracy;CVPerformance{ii}.Accuracy];
%     Precision = [Precision;CVPerformance{ii}.Precision];
%     Recall = [Recall;CVPerformance{ii}.Recall];
%     F1Score = [F1Score;CVPerformance{ii}.F1Score];


%     GazeAccuracy = [ GazeAccuracy; GazeCVPerformance{ii}.Accuracy];
%     GazePrecision = [ GazePrecision; GazeCVPerformance{ii}.Precision];
%     GazeRecall = [ GazeRecall; GazeCVPerformance{ii}.Recall];
%     GazeF1Score = [ GazeF1Score; GazeCVPerformance{ii}.F1Score];

    GapAccuracy = [GapAccuracy;GapCVPerformance{ii}.Accuracy];
    GapPrecision = [GapPrecision;GapCVPerformance{ii}.Precision];
    GapRecall = [GapRecall;GapCVPerformance{ii}.Recall];
    GapF1Score = [GapF1Score;GapCVPerformance{ii}.F1Score];        

% 
%     SpeedAccuracy = [ SpeedAccuracy; SpeedCVPerformance{ii}.Accuracy];
%     SpeedPrecision = [ SpeedPrecision; SpeedCVPerformance{ii}.Precision];
%     SpeedRecall = [ SpeedRecall; SpeedCVPerformance{ii}.Recall];
%     SpeedF1Score = [ SpeedF1Score; SpeedCVPerformance{ii}.F1Score];


end


% Accuracy = nanmean(Accuracy);
% Precision = nanmean(Precision);
% Recall = nanmean(Recall);
% F1Score = nanmean(F1Score);
% Performance = [Accuracy;Precision;Recall;F1Score];
% 
% GazeAccuracy = nanmean(GazeAccuracy);
% GazePrecision = nanmean(GazePrecision);
% GazeRecall = nanmean(GazeRecall);
% GazeF1Score = nanmean(GazeF1Score);
% GazePerformance = [GazeAccuracy;GazePrecision;GazeRecall;GazeF1Score];

GapAccuracy = nanmean(GapAccuracy);
GapPrecision = nanmean(GapPrecision);
GapRecall = nanmean(GapRecall);
GapF1Score = nanmean(GapF1Score);
GapPerformance = [GapAccuracy;GapPrecision;GapRecall;GapF1Score];

% SpeedAccuracy = nanmean(SpeedAccuracy);
% SpeedPrecision = nanmean(SpeedPrecision);
% SpeedRecall = nanmean(SpeedRecall);
% SpeedF1Score = nanmean(SpeedF1Score);
% SpeedPerformance = [SpeedAccuracy;SpeedPrecision;SpeedRecall;SpeedF1Score];

% 
% OverallPerformance = [GapPerformance,GazePerformance,SpeedPerformance,Performance];