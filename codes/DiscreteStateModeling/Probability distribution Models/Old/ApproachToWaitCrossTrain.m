function Prob_AWC = ApproachToWaitCrossTrain(GapData)

%% tuning inputs

binsize = 0.5;
Gazebinsize = 0.1;
Speedbinsize = 0.2;

GapInd = 1;     %1-4 values
GazeInd = 1;    %1-5 values

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Gap when pedestrian reaches 1.5m 1.0m, 0.5 m from the crosswalk
% %% Time gap - Dependent variable
% ExpectedGap = GapData(:,119:122);
% ExpectedGapAcc = GapData(:,123:126);
% 
% PedSpeed = GapData(:,148:151);
% 
% %% Gaze Data - Dependent variable
% GazeRatiosGapStart = GapData(:,127:146);

% % Gap indices
% tempIndicesTrain.AWCWaitGaps = find(GapData(:,147)==0);
% tempIndicesTrain.AWCCrossGaps = find(GapData(:,147)==1);
% tempIndicesTrain.AWCAllGaps = [tempIndicesTrain.AWCWaitGaps;tempIndicesTrain.AWCCrossGaps];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Gap when start of a vehicle gap
%%Time gap - Dependent variable 
ExpectedGap = GapData(:,5);
ExpectedGapAcc = GapData(:,6);
PedSpeed = GapData(:,7);

tempIndicesTrain.AWCWaitGaps = find(GapData(:,4)==0);
tempIndicesTrain.AWCCrossGaps = find(GapData(:,4)==1);
tempIndicesTrain.AWCAllGaps = [tempIndicesTrain.AWCWaitGaps;tempIndicesTrain.AWCCrossGaps];


%% Gaze Data - Dependent variable
GazeRatiosGapStart = GapData(:,8:12);

% GapType = ExpectedGap(:,GapInd);
GapType = ExpectedGapAcc(:,GapInd);

GazeType = GazeRatiosGapStart(:,5*(GazeInd-1)+GapInd);
SpeedType = PedSpeed(:,GapInd);


% %% Observation combined
% [ObservationType,DataBinned] = CombineObservations([GapType,GazeType],[binsize,Gazebinsize],[10,1]);


%% individual probabilities
ii=1;
% gap probabilities from training data
[N_Train(ii),M_Train(ii),Prob_GapCross_Train(:,ii),Prob_GapDistribution_Train(:,ii),Prob_WaitedGapDistribution_Train(:,ii),...
Prob_CrossedGapDistribution_Train(:,ii)] = ApproachWCProbability(GapType,tempIndicesTrain,binsize,10);


% gaze probabilities from training data
[N_GazeTrain(ii),M_GazeTrain(ii),Prob_GazeCross_Train(:,ii),Prob_GazeDistribution_Train(:,ii),Prob_WaitedGazeDistribution_Train(:,ii),...
Prob_CrossedGazeDistribution_Train(:,ii)] = ApproachWCProbability(GazeType,tempIndicesTrain,Gazebinsize,1);


% speed probabilities from training data
[N_SpeedTrain(ii),M_SpeedTrain(ii),Prob_SpeedCross_Train(:,ii),Prob_SpeedDistribution_Train(:,ii),Prob_WaitedSpeedDistribution_Train(:,ii),...
Prob_CrossedSpeedDistribution_Train(:,ii)] = ApproachWCProbability(SpeedType,tempIndicesTrain,Speedbinsize,3);

% 
% % combined observation probabilities from training data
% [N_SpeedTrain(ii),M_SpeedTrain(ii),Prob_SpeedCross_Train(:,ii),Prob_CombinedDistribution_Train(:,ii),Prob_WaitedCombinedDistribution_Train(:,ii),...
% Prob_CrossedCombinedDistribution_Train(:,ii)] = ApproachWCProbability(ObservationType-1,tempIndicesTrain,[1,1],[ceil([10/binsize,1/Gazebinsize])]);



Prob_AWC.GapDistribution_Train = Prob_GapDistribution_Train;
Prob_AWC.CrossedGapDistribution_Train = Prob_CrossedGapDistribution_Train;

Prob_AWC.GazeDistribution_Train = Prob_GazeDistribution_Train;
Prob_AWC.CrossedGazeDistribution_Train = Prob_CrossedGazeDistribution_Train;

Prob_AWC.SpeedDistribution_Train = Prob_SpeedDistribution_Train;
Prob_AWC.CrossedSpeedDistribution_Train = Prob_CrossedSpeedDistribution_Train;

% Prob_AWC.CombinedDistribution_Train = Prob_CombinedDistribution_Train;
% Prob_AWC.CrossedCombinedDistribution_Train = Prob_CrossedCombinedDistribution_Train;

Prob_AWC.CrossedGap = length(tempIndicesTrain.AWCCrossGaps)/length(tempIndicesTrain.AWCAllGaps);

end