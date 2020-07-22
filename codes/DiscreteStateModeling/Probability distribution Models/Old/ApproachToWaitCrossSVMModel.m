%% SVM

% clear all
% close all
% 
% 
% % Read data
% vehicleGapTimes = xlsread('VehicleGapTimesV6.xlsx');
% GapData = xlsread('GapWiseCompiledDataV6.xlsx');
% EventIndices = xlsread('DiscreteStateEventIndicesW5.xlsx');

%% Time gap - Dependent variable
ExpectedGap = GapData(:,119:122);
ExpectedGapAcc = GapData(:,123:126);
PedSpeed = GapData(:,148:151);

%% Gaze Data - Dependent variable
GazeRatiosGapStart = GapData(:,127:146);

GapInd = 2;     %1-4 values
GazeInd = 1;    %1-5 values

% GapType = ExpectedGap(:,GapInd);
GapType = ExpectedGapAcc(:,GapInd);

GazeType = GazeRatiosGapStart(:,5*(GazeInd-1)+GapInd);
SpeedType = PedSpeed(:,GapInd);

%% Approach to cross indices
indices.AWCWaitGaps = find(GapData(:,147)==0);
indices.AWCCrossGaps = find(GapData(:,147)==1);
indices.AWCAllGaps = [indices.AWCWaitGaps;indices.AWCCrossGaps];
AWCAllGapsDecision = GapData(indices.AWCAllGaps,147);



ExpectedGapAccSVM = ExpectedGapAcc(indices.AWCAllGaps);
GazeTypeSVM = GazeType(indices.AWCAllGaps);
SpeedTypeSVM = SpeedType(indices.AWCAllGaps);

SVMData = table(AWCAllGapsDecision,ExpectedGapAccSVM,GazeTypeSVM,SpeedTypeSVM);

normExpectedGapAccSVM = (ExpectedGapAccSVM-nanmean(ExpectedGapAccSVM))/nanstd(ExpectedGapAccSVM);
normGazeTypeVM = (GazeTypeSVM-nanmean(GazeTypeSVM))/nanstd(GazeTypeSVM);
normSpeedTypeSVM = (SpeedTypeSVM-nanmean(SpeedTypeSVM))/nanstd(SpeedTypeSVM);

normSVMData = table(AWCAllGapsDecision,normExpectedGapAccSVM,normGazeTypeVM,normSpeedTypeSVM);




