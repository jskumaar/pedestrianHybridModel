function [N,M,Prob_GapAcceptance,Prob_GapDistribution,Prob_WaitGapDistribution,...
    Prob_CrossGapDistribution] = ApproachWCProbability(ExpectedGap,indices,size,limit)

    M = length(indices.AWCWaitGaps);
    N = length(indices.AWCCrossGaps);
    Prob_GapAcceptance = N/(N+M);
    
    AllWaitToCrossGaps = ExpectedGap(indices.AWCAllGaps);
    WaitWaitToCrossGaps = ExpectedGap(indices.AWCWaitGaps);
    CrossWaitToCrossGaps = ExpectedGap(indices.AWCCrossGaps);
    

    h = histogram(AllWaitToCrossGaps,[0:size:limit]);
    Prob_GapDistribution = h.BinCounts/sum(h.BinCounts);

    %figure()
    h = histogram(WaitWaitToCrossGaps,[0:size:limit]);
    Prob_WaitGapDistribution = h.BinCounts/sum(h.BinCounts);

    %figure()
    h = histogram(CrossWaitToCrossGaps,[0:size:limit]);
    Prob_CrossGapDistribution = h.BinCounts/sum(h.BinCounts);
    close
end