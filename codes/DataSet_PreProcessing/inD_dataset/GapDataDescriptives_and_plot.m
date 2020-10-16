%% Gap data descriptives
% 

% % a) addpath of necessary directories
p1 = genpath('G:\My Drive\Research\Projects\pedestrianHybridModel\codes');
p2 = genpath('G:\My Drive\Research\Projects\pedestrianHybridModel\datasets');

addpath(p1)
addpath(p2)

% load('GapData_12Scenes_v6.mat');

GapFeatures_SVM = GapFeatures;


AcceptedGaps = find(GapFeatures_SVM.CrossDecision==1);
RejectedGaps = find(GapFeatures_SVM.CrossDecision==0);

% to find gap outliers
Velocity_Gaps = GapFeatures_SVM.F_pedDistToVeh./GapFeatures_SVM.F_vehVel;
Velocity_Gaps_range = Velocity_Gaps;
Velocity_Gaps_range(GapFeatures_SVM.F_pedDistToVeh<=0 | GapFeatures_SVM.F_vehVel<0.5 | Velocity_Gaps>100) = [];
% 
% figure()
% b1 = boxplot(Velocity_Gaps_range);
% max outlier = 10.84; min outlier = 0.0;

% find indices of accept and reject gaps for non-outlying gaps
ind_remove_velocity_dist = find(GapFeatures_SVM.F_pedDistToVeh<=0 | GapFeatures_SVM.F_vehVel<0.5 | Velocity_Gaps>10.84);
ind_all_legal_gaps = [1:length(Velocity_Gaps)]';
% ind_all_legal_gaps(ind_remove_velocity_dist) = [];

ind_accepted_gaps = AcceptedGaps;
[~,ind_rem,~] = intersect(ind_accepted_gaps, ind_remove_velocity_dist);
ind_accepted_gaps(ind_rem) = [];

ind_rejected_gaps = RejectedGaps;
[~,ind_rem,~] = intersect(ind_rejected_gaps, ind_remove_velocity_dist);
ind_rejected_gaps(ind_rem) = [];

GapFeatures_SVM_NoOutliers = GapFeatures_SVM;
GapFeatures_Accepted = GapFeatures_SVM(AcceptedGaps,:);
GapFeatures_Rejected = GapFeatures_SVM(RejectedGaps,:);

% Gap acceptance parameter distribution
% typeOfGap = RejectedGaps;
typeOfGap = AcceptedGaps;


VehVelGap = GapFeatures_SVM.F_pedDistToVeh(typeOfGap)./GapFeatures_SVM.F_vehVel(typeOfGap);
DistCurb = GapFeatures_SVM.F_pedDistToCurb(typeOfGap);
DistCW = GapFeatures_SVM.F_pedDistToCurb(typeOfGap);
Gaze = GapFeatures_SVM.F_gazeRatio(typeOfGap);
DistVeh = GapFeatures_SVM.F_pedDistToVeh(typeOfGap);
PedVel = GapFeatures_SVM.F_pedSpeed(typeOfGap);
Wait_time = GapFeatures_SVM.F_cumWait(typeOfGap)/Params.AdjustedSampFreq;

GapFeatures_SVM.F_cumWait(GapFeatures_SVM.F_cumWait==-1) = 0;

%%%%%%%%%%%%%%%%%%%
% identify outliers
% a)Vehicle Gap
figure()
b1 = boxplot(VehVelGap);
% identify from plot
max_outlier_gap = 12.3136;
min_outlier_gap = 0.0314;

outlier_indices_gap = find(VehVelGap <= min_outlier_gap | VehVelGap > max_outlier_gap);
%%%%%%%%
% b)Vehicle Ped Distance
figure()
b2 = boxplot(DistVeh);
% No outliers observed
% identify from plot
max_outlier_veh_dist = 60.875;
min_outlier_veh_dist = 0.1773;

outlier_indices_veh_dist = find(DistVeh <= min_outlier_veh_dist | DistVeh > max_outlier_veh_dist);
%%%%%%%%
% c)Curb Distance
figure()
b3 = boxplot(DistCurb);
% No outliers observed
% identify from plot
max_outlier_DTCurb = 10.4577;
min_outlier_DTCurb = 0.114;

outlier_indices_DTCurb = find(DistCurb <= min_outlier_DTCurb | DistCurb > max_outlier_DTCurb);
%%%%%%%%
% d)Wait time
figure()
b3 = boxplot(Wait_time);
% No outliers observed
% identify from plot
max_outlier_wait = 10.4577;
min_outlier_wait = 0.114;

outlier_indices_wait = find(Wait_time <= min_outlier_wait | DistCurb > max_outlier_wait);


Wait_time_notZero = Wait_time;
Wait_time_notZero(Wait_time==0) = [];

% gaps with zero wait
indices_noWait = find(GapFeatures_SVM.F_cumWait==-1);



%% 1) Histogram


Data = VehVelGap;

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
h2 = histogram(Wait_time,'Normalization','probability','BinWidth',1);
xlabel('Time Gap [s]')
ylabel('Probability')
title('Rejected Gap probability distribution')
