function Prob = DiscreteTransitionProbability(SVMTrainData,DataBinSizes,N_obs,startlimit,endlimit)


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

% startlimit = min(DistributionData);
% endlimit = max(DistributionData);

% select the combination of observations (all observations cannot be
% included; state explosion; either bin size should be increased or number
% of observation variables should be reduced

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

Prob.AcceptedGap = Prob_WCGapAcceptance;
Prob.AcceptedGapDistribution_Train = Prob_AcceptedCombinedDistribution;
Prob.GapDistribution_Train = Prob_WCFullCombinedDistribution;
Prob.IndividualGapDistribution = Prob_WCFullGapDistribution;
Prob.IndividualAcceptedGapDistribution = Prob_WCFullAcceptedGapDistribution;





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%