function [N,M,Prob_GapAcceptance,Prob_GapDistribution,Prob_AcceptedGapDistribution,...
    Prob_RejectedGapDistribution] = WCProbability(ExpectedGap,indices,n,startlimit,endlimit)

    N = length(indices.WCAcceptedGaps);
    M = length(indices.WCRejectedGaps);
    Prob_GapAcceptance = N/(N+M);
    
    AllWaitToCrossGaps = ExpectedGap(indices.WCAllGaps);
    AcceptedWaitToCrossGaps = ExpectedGap(indices.WCAcceptedGaps);
    RejectedWaitToCrossGaps = ExpectedGap(indices.WCRejectedGaps);
        
    h = histogram(AllWaitToCrossGaps,[startlimit:n:endlimit]);
    Prob_GapDistribution = h.BinCounts/sum(h.BinCounts);
    
%     figure()
%     h=histogram(AllWaitToCrossGaps,[0:n:8],'Normalization','probability');hold on;
%     xlabel('Gap Duration [s]')
%     ylabel('Gap Probability')
%     axis([0, 8, 0, 0.35])
%     title('Overall Gap distribution')
%     set(gca,'fontsize', 18)   
    
% % plot(conv(h.BinEdges, [0.5 0.5], 'valid'), h.BinCounts/sum(h.BinCounts)); hold on;
%     ft=fittype('gauss1');
%     cf=fit(conv(h.BinEdges, [0.5 0.5], 'valid')',h.BinCounts'/sum(h.BinCounts),ft);
%     hold on;
%     plot(cf,conv(h.BinEdges, [0.5 0.5], 'valid')',h.BinCounts'/sum(h.BinCounts))
    
    

    h = histogram(AcceptedWaitToCrossGaps,[startlimit:n:endlimit]);
    Prob_AcceptedGapDistribution = h.BinCounts/sum(h.BinCounts);
    
%     figure()
%     h=histogram(AcceptedWaitToCrossGaps,[0:n:8],'Normalization','probability');
%     xlabel('Gap Duration  [s]')
%     ylabel('Gap Probability')
%     title('Accepted Gaps distribution')
%     axis([0, 8, 0, 0.35])
%     set(gca,'fontsize', 18)


    h = histogram(RejectedWaitToCrossGaps,[startlimit:n:endlimit]);
    Prob_RejectedGapDistribution = h.BinCounts/sum(h.BinCounts);
    
%     figure()
%     h=histogram(RejectedWaitToCrossGaps,[0:n:8],'Normalization','probability');
%     xlabel('Gap Duration  [s]')
%     ylabel('Gap Probability')
%     title('Rejected Gaps distribution')
%     set(gca,'fontsize', 18)
%     axis([0, 8, 0, 0.35])
    
     close
     
      
     
     %% fit distributions to histograms
     Distributions = {'Beta','Binomial','BirnbaumSaunders','Burr','Exponential','ExtremeValue',...
                        'Gamma','GeneralizedExtremeValue','GeneralizedPareto','HalfNormal','InverseGaussian',...
                        'Kernel','Logistic','Loglogistic','Lognormal','Nakagami','NegativeBinomial','Normal',...
                        'Poisson','Rayleigh','Rician','Stable','tLocationScale','Weibull'};
     
     
%      for ii=1:size(Distributions,2)
%          pd_AllGaps = fitdist(Prob_GapDistribution',Distributions{ii});
%          x=1;
%      end
%   
%      
%      hold on;     
%      gm = fitgmdist(DistributionData(indices.WCAcceptedGaps,ii),2);
%      x = DataBinSizes(ii)*[0:0.1:length(Prob_WCFullAcceptedGapDistribution{ii})]';
%      y = pdf(gm,x);
%      plot(x,y); hold on; 
     
     
     
     
     
     
     
     
     
end