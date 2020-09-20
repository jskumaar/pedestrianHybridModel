%% Gap data descriptives
% 
















GapFeatures_SVM = GapFeatures;


AcceptedGaps = find(GapFeatures_SVM.CrossDecision==1);
RejectedGaps = find(GapFeatures_SVM.CrossDecision==0);

AcceptedGaps_test = find(SVMTestData.Decision==1 |  SVMTestData.Decision==2);
RejectedGaps_test = find(SVMTestData.Decision==0);

AcceptedGaps_train = find(SVMTrainData.Decision==1 | SVMTrainData.Decision==2);
RejectedGaps_train = find(SVMTrainData.CrossDecision==0);

RejectedGaps_train_subset = RejectedGaps_train(randperm(length(RejectedGaps_train), 850));

AcceptedGaps_train_ss = find(SVMTrainData_subset.CrossDecision==1);
RejectedGaps_train_ss = find(SVMTrainData_subset.CrossDecision==0);

% to find gap outliers
Velocity_Gaps = GapFeatures_SVM.F_pedDistToVeh./GapFeatures_SVM.F_vehVel;
Velocity_Gaps_range = Velocity_Gaps;
Velocity_Gaps_range(GapFeatures_SVM.F_pedDistToVeh<=0 | GapFeatures_SVM.F_vehVel<0.5 | Velocity_Gaps>100) = [];

figure()
b1 = boxplot(Velocity_Gaps_range);
% max outlier = 10.84; min outlier = 0.0;

% find indices of accept and reject gaps for non-outlying gaps
ind_remove_velocity_dist = find(GapFeatures_SVM.F_pedDistToVeh<=0 | GapFeatures_SVM.F_vehVel<0.5 | Velocity_Gaps>10.84);
ind_all_legal_gaps = [1:length(Velocity_Gaps)]';
ind_all_legal_gaps(ind_remove_velocity_dist) = [];

ind_accepted_gaps = AcceptedGaps;
[~,ind_rem,~] = intersect(ind_accepted_gaps, ind_remove_velocity_dist);
ind_accepted_gaps(ind_rem) = [];

ind_rejected_gaps = RejectedGaps;
[~,ind_rem,~] = intersect(ind_rejected_gaps, ind_remove_velocity_dist);
ind_rejected_gaps(ind_rem) = [];

GapFeatures_SVM_NoOutliers = GapFeatures_SVM;



% Gap acceptance parameter distribution
typeOfGap = RejectedGaps_train;

typeOfGap = AcceptedGaps_test;



VehVelGap = GapFeatures_SVM.F_pedDistToVeh(typeOfGap)./GapFeatures_SVM.F_vehVel(typeOfGap);
DistCurb = GapFeatures_SVM.F_pedDistToCurb(typeOfGap);
DistCW = GapFeatures_SVM.F_pedDistToCurb(typeOfGap);
Gaze = GapFeatures_SVM.F_gazeRatio(typeOfGap);
DistVeh = GapFeatures_SVM.F_pedDistToVeh(typeOfGap);
PedVel = GapFeatures_SVM.F_pedSpeed(typeOfGap);
Wait_time = GapFeatures_SVM.F_cumWait(typeOfGap);



%% 1) Histogram


Data = DistVeh;

BinWidth = 1;
NumberOfBins = 15;
figure()
%h=histogram(Data,NumberOfBins);
%h=histogram(Data,NumberOfBins,'Normalization','probability');   %to plot probability densityh=histogram(Data,NumberOfBins,'Normalization','probability');   %to plot probability density
h=histogram(Data,'BinWidth',BinWidth,'Normalization','probability');   %to plot probability densityh=histogram(Data,NumberOfBins,'Normalization','probability');   %to plot probability density
%h.FaceColor = 'none';       %adjust colour of the bin
xlabel('Waiting time [s]')
ylabel('Counts')
title('Vehicle Velocity Gap')
set(gca,'fontsize', 18)
% %To plot the distribution curve
% hold on;
% plot(conv(h.BinEdges, [0.5 0.5], 'valid'), h.BinCounts/sum(h.BinCounts))

%To save figure
saveas(gcf,'Vehicle_VelocityGap_distribution','fig')
saveas(gcf,'Vehicle_VelocityGap_distribution','png')
saveas(gcf,'Vehicle_VelocityGap_distribution','eps')










%new median  for new range
median(Velocity_Gaps_accepted_range)
figure()
b = boxplot(Velocity_Gaps_accepted_range);


cumulative_GapAcceptance_Prob = cumsum(h.Values);
% plot cumulative probability
Msize = 14;
figure()
plot([h.BinEdges(1:end-1) + h.BinWidth/2], cumulative_GapAcceptance_Prob, 'k-'); hold on;
plot([h.BinEdges(1:end-1) + h.BinWidth/2], cumulative_GapAcceptance_Prob, '*', 'MarkerSize', Msize);
grid on;
xlabel('Time Gap [s]')
ylabel('Probability')
title('Gap acceptance cumulative probability distribution')
hold on;
xline(5.16)


% Rejected Gaps
Velocity_Gaps_rejected = GapFeatures_SVM.F_pedDistToVeh(RejectedGaps)./GapFeatures_SVM.F_vehVel(RejectedGaps);
Velocity_Gaps_rejected_range = Velocity_Gaps_rejected;

Velocity_Gaps_rejected_range(GapFeatures_SVM.F_vehVel(RejectedGaps) < 0) = [];
Velocity_Gaps_rejected_range(Velocity_Gaps_rejected_range > 12.90) = [];

% plot histogram
figure()
h2 = histogram(Velocity_Gaps_rejected_range,'Normalization','probability','BinWidth',1);
xlabel('Time Gap [s]')
ylabel('Probability')
title('Rejected Gap probability distribution')
