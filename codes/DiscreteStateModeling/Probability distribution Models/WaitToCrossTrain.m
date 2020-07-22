function Prob_WC = WaitToCrossTrain(GapData,DistributionData,EventIndices,waitThreshold,out,binsize,startlimit,endlimit)


tempIndicesTrain = WaitToCrossGapIndices(GapData,EventIndices,waitThreshold,out);

%% Observation combined
[ObservationType,DataBinned,Num_levels] = CombineObservations(DistributionData,binsize,startlimit,endlimit);


%% probabilities from training data

for ii=1:size(DistributionData,2)
    [N,M,Prob_WCGapAcceptance,Prob_WCFullGapDistribution{ii},Prob_WCFullAcceptedGapDistribution{ii},...
    Prob_WCFullRejectedGapDistribution{ii}] = WCProbability(DistributionData(:,ii),tempIndicesTrain,binsize(ii),startlimit(ii),endlimit(ii));
end

% combined observation probabilities from training data
[N_CombinedTrain,M_CombinedTrain,Prob_CombinedCross_Train,Prob_CombinedDistribution_Train,Prob_AcceptedCombinedDistribution_Train,...
Prob_RejectedCombinedDistribution_Train] = WCProbability(ObservationType-1,tempIndicesTrain,1,1,Num_levels);


Prob_WC.GapDistribution_Train = Prob_WCFullGapDistribution;
Prob_WC.AcceptedGapDistribution_Train = Prob_WCFullAcceptedGapDistribution;
 
Prob_WC.CombinedDistribution_Train = Prob_CombinedDistribution_Train;
Prob_WC.AcceptedCombinedDistribution_Train = Prob_AcceptedCombinedDistribution_Train;

Prob_WC.AcceptedGap = length(tempIndicesTrain.WCAcceptedGaps)/length(tempIndicesTrain.WCAllGaps);

end



