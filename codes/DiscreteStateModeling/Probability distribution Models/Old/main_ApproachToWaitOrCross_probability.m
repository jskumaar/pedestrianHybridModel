clear all
close all

%%% Read data
vehicleGapTimes = xlsread('VehicleGapTimesV6.xlsx');
GapData = xlsread('GapWiseCompiledDataV6.xlsx');
EventIndices = xlsread('DiscreteStateEventIndicesW5.xlsx');

%% Time gap - Dependent variable
ExpectedGap = GapData(:,119:122);
ExpectedGapAcc = GapData(:,123:126);
PedSpeed = GapData(:,148:151);

%% Gaze Data - Dependent variable
GazeRatiosGapStart = GapData(:,127:146);

%% inputs
Nk=5;        %cross-validation fold

binsize = 0.5;
Gazebinsize = 0.1;
Speedbinsize = 0.5;


GapInd = 1;     %1-4 values
GazeInd = 1;    %1-5 values

GapType = ExpectedGap(:,GapInd);
% GapType = ExpectedGapAcc(:,GapInd);


GazeType = GazeRatiosGapStart(:,5*(GazeInd-1)+GapInd);
SpeedType = PedSpeed(:,GapInd);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Approach to cross indices
indices.AWCWaitGaps = find(GapData(:,147)==0);
indices.AWCCrossGaps = find(GapData(:,147)==1);
indices.AWCAllGaps = [indices.AWCWaitGaps;indices.AWCCrossGaps];
AWCAllGapsDecision = GapData(indices.AWCAllGaps,147);
AllGapsDecision = GapData(:,147);


%% WC Probabilities from full data
[N,M,Prob_AWCGapWaitance,Prob_AWCFullGapDistribution,Prob_AWCFullWaitedGapDistribution,...
Prob_AWCFullCrossedGapDistribution] = ApproachWCProbabilityGaps(GapType,indices,binsize);


% [N,M,Prob_AWCGapWaitance,Prob_AWCFullGapDistribution,Prob_AWCFullWaitedGapDistribution,...
% Prob_AWCFullCrossedGapDistribution] = ApproachWCProbabilitySpeedGaps(SpeedType,indices,Speedbinsize);


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
    
    % gap probabilities from training data
    [N_Train(ii),M_Train(ii),Prob_GapCross_Train(:,ii),Prob_GapDistribution_Train(:,ii),Prob_WaitedGapDistribution_Train(:,ii),...
    Prob_CrossedGapDistribution_Train(:,ii)] = ApproachWCProbability(GapType,tempIndicesTrain,binsize,10);
    

    % gaze probabilities from training data
    [N_GazeTrain(ii),M_GazeTrain(ii),Prob_GazeCross_Train(:,ii),Prob_GazeDistribution_Train(:,ii),Prob_WaitedGazeDistribution_Train(:,ii),...
    Prob_CrossedGazeDistribution_Train(:,ii)] = ApproachWCProbability(GazeType,tempIndicesTrain,Gazebinsize,1);


    % speed probabilities from training data
    [N_SpeedTrain(ii),M_SpeedTrain(ii),Prob_SpeedCross_Train(:,ii),Prob_SpeedDistribution_Train(:,ii),Prob_WaitedSpeedDistribution_Train(:,ii),...
    Prob_CrossedSpeedDistribution_Train(:,ii)] = ApproachWCProbability(SpeedType,tempIndicesTrain,Speedbinsize,3);
    

    
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

    WCDecisionActual = AllGapsDecision(tempIndicesTest.AWCAllGaps);

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


