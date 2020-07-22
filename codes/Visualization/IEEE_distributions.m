%% 2) Bar Graph
figure()
ii=2;
b=bar(DataBinSizes(ii)*[1:length(Prob_WCFullAcceptedGapDistribution{ii})],Prob_WCFullAcceptedGapDistribution{ii},'BarWidth',1); hold on;
width = b.BarWidth;
b.FaceColor = [0.5843    0.8157    0.9882];
b.EdgeColor = 'k';
c=bar(DataBinSizes(ii)*[1:length(Prob_WCFullAcceptedGapDistribution{ii})],Prob_WCFullRejectedGapDistribution{ii},'BarWidth',1); hold on;
c.FaceColor = [0.9882    0.8157    0.5843];
c.EdgeColor = 'k';
axis([0,53,0,0.3])


b=bar(DataBinSizes(ii)*[2,4:30,32:36,40],Prob_WCFullAcceptedGapDistribution{ii}([2,4:30,32:36,40]),'BarWidth',1); hold on;
b.FaceColor =  [0.5    0.6    0.8];
c=bar(DataBinSizes(ii)*[1,3,6,7],Prob_WCFullRejectedGapDistribution{ii}([1,3,6,7]),'BarWidth',1); hold on;
c.FaceColor =  [0.5    0.6    0.8];

xlabel('Waiting Time [s]')
ylabel('Probability')
title('Waiting Time Distribution')
set(gca,'fontsize', 24)
legend('Accepted Gaps','Rejected Gaps')



saveas('Pedestrian walking speed comparison','fig')
saveas('Pedestrian walking speed comparison','png')
saveas('Pedestrian walking speed comparison','eps')
saveas('Pedestrian walking speed comparison','eps')




%% Gap distributions vs Driving Behavior

for ii=1:90
ind  = find(ExpectedGapData.DiscreteState==3);
end


DefInd = ([0:179]*3) + 1;
NorInd = ([0:179]*3) + 2;
AggInd = ([0:179]*3) + 3;

figure()
hist(ExpectedGapData.WCExpectedGapStartGap(ind(DefInd)),15)
title('Defensive Driving')
figure()
hist(ExpectedGapData.WCExpectedGapStartGap(ind(NorInd)),15)
title('Normal Driving')
figure()
hist(ExpectedGapData.WCExpectedGapStartGap(ind(AggInd)),15)
title('Aggressive Driving')


%% trajectories

load('AllFeaturesCrossingWise_PW_11.mat')
ind=7
plot(DataPredict{ind}.PedestrianPosition(:,1),DataPredict{ind}.PedestrianPosition(:,2),'.','MarkerSize',20);hold on;

plot(DataPredict{ind}.PedestrianPosition(1,1),DataPredict{ind}.PedestrianPosition(1,2),'square','MarkerSize',15);hold on;
plot(DataPredict{ind}.PedestrianPosition(end,1),DataPredict{ind}.PedestrianPosition(end,2),'d','MarkerSize',15);hold on;









