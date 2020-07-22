function [N,M,Prob_GapAcceptance,Prob_GapDistribution,Prob_AcceptedGapDistribution,...
    Prob_RejectedGapDistribution] = ApproachWCProbabilityGazeGaps(GazeRatio,indices,n)

    N = length(indices.WCAcceptedGaps);
    M = length(indices.WCRejectedGaps);
    Prob_GapAcceptance = N/(N+M);
    
    
    for ii=1:size(GazeRatio,2)
        AllWaitToCrossRatios = GazeRatio(indices.WCAllGaps,ii);
        AcceptedWaitToCrossRatios = GazeRatio(indices.WCAcceptedGaps,ii);
        RejectedWaitToCrossGaps = GazeRatio(indices.WCRejectedGaps,ii);

        h = histogram(AllWaitToCrossRatios,[0:n:1]);
        Prob_GapDistribution(:,ii) = h.BinCounts/sum(h.BinCounts);


        h = histogram(AcceptedWaitToCrossRatios,[0:n:1]);
        Prob_AcceptedGapDistribution(:,ii) = h.BinCounts/sum(h.BinCounts);


        h = histogram(RejectedWaitToCrossGaps,[0:n:1]);
        Prob_RejectedGapDistribution(:,ii) = h.BinCounts/sum(h.BinCounts);
        
    figure()    
    h=histogram(AllWaitToCrossRatios,[0:n:1],'Normalization','probability');
    xlabel('Gaze ratios')
    ylabel('Probability')
    title('Overall Gaze ratio distribution')
    set(gca,'fontsize', 18)
    
    

    figure()
    h=histogram(AcceptedWaitToCrossRatios,[0:n:1],'Normalization','probability');
    Prob_AcceptedGapDistribution = h.BinCounts/sum(h.BinCounts);
    xlabel('Gaze ratios')
    ylabel('Probability')
    title('Accepted Gaze ratio distribution')
    set(gca,'fontsize', 18)


 
    figure()
    h=histogram(RejectedWaitToCrossGaps,[0:n:1],'Normalization','probability');
    Prob_RejectedGapDistribution = h.BinCounts/sum(h.BinCounts);
    xlabel('Gaze ratios')
    ylabel('Probability')
    title('Rejection Gaze ratio distribution')
    set(gca,'fontsize', 18)
        
        
        
        
        
    end
end