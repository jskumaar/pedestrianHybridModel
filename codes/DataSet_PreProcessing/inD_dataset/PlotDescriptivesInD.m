%% This scripts plots the track descriptives


function [] = PlotDescriptivesInD(tracks, trackDescriptivesComplete)

pedApproachSpeed = [];
pedCrossSpeed = [];
pedWalkAwaySpeed = [];
waitDuration = [];


for ii=1:12
    
    ped_tracks = tracks{ii}.ped_tracks;
    ped_waiting_tracks = tracks{ii}.ped_waiting_tracks;
    trackDescriptivesData  = trackDescriptivesComplete{ii};
    
    pedApproachSpeed = [pedApproachSpeed; trackDescriptivesData.approachSpeed(ped_tracks)];
    pedCrossSpeed   = [pedCrossSpeed; trackDescriptivesData.crossSpeed(ped_tracks)];
    pedWalkAwaySpeed = [pedWalkAwaySpeed; trackDescriptivesData.walkawaySpeed(ped_tracks)];
    waitDuration = [waitDuration; trackDescriptivesData.waitDuration(ped_waiting_tracks)/25];
    
end


waitDuration_2 = waitDuration;
waitDuration_2(waitDuration_2>12.6) = [];

boxplot(waitDuration_2)

disp('Average approach speed of pedestrians... \n')
mean( pedApproachSpeed)

disp('Average cross speed of pedestrians... \n')
mean( pedCrossSpeed)

disp('Average walkaway speed of pedestrians... \n')
mean( pedWalkAwaySpeed)


figure()
plotbar = bar([mean( pedApproachSpeed), mean( pedCrossSpeed), mean( pedWalkAwaySpeed)]);

figure()
subplot(2,2,1)
histogram(pedApproachSpeed(pedApproachSpeed>0.4),'Normalization','probability');
ylabel('Probability')
xlabel('Approach speed')

subplot(2,2,2)
histogram(pedCrossSpeed(pedCrossSpeed>0.2), 'Normalization','probability');
ylabel('Probability')
xlabel('Cross speed')

subplot(2,2,3)
histogram(pedWalkAwaySpeed(pedWalkAwaySpeed>0.3), 'Normalization','probability');
ylabel('Probability')
xlabel('Walkaway speed')

subplot(2,2,4)
histogram(waitDuration_2, 'Normalization','probability');
ylabel('Probability')
xlabel('Wait Time')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%

%% Gap data descriptives

load('GapData_12Scenes.mat');
GapFeatures_SVM = GapFeatures;


AcceptedGaps = find(GapFeatures_SVM.Decision==1 | GapFeatures_SVM.Decision==2);
RejectedGaps = find(GapFeatures_SVM.Decision==0);

AcceptedGaps_test = find(SVMTestData.Decision==1 |  SVMTestData.Decision==2);
RejectedGaps_test = find(SVMTestData.Decision==0);

AcceptedGaps_train = find(SVMTrainData.Decision==1 | SVMTrainData.Decision==2);
RejectedGaps_train = find(SVMTrainData.CrossDecision==0);

RejectedGaps_train_subset = RejectedGaps_train(randperm(length(RejectedGaps_train), 850));

AcceptedGaps_train_ss = find(SVMTrainData_subset.CrossDecision==1);
RejectedGaps_train_ss = find(SVMTrainData_subset.CrossDecision==0);


Velocity_Gaps = GapFeatures_SVM.F_pedDistToVeh./GapFeatures_SVM.F_vehVel;
Velocity_Gaps_range = Velocity_Gaps;
Velocity_Gaps_range(Velocity_Gaps_range<0) = [];

figure()
b1 = boxplot(Velocity_Gaps_range);
% max outlier = 12.90; min outlier = 0.01; 25th = 2.51; 75th = 6.60; 50th =
% 4.21

Velocity_Gaps_range(Velocity_Gaps_range > 12.90) = [];



Velocity_Gaps_accepted = GapFeatures_SVM.F_pedDistToVeh(AcceptedGaps)./GapFeatures_SVM.F_vehVel(AcceptedGaps);
Velocity_Gaps_accepted_range = Velocity_Gaps_accepted;

Velocity_Gaps_accepted_range(GapFeatures_SVM.F_vehVel(AcceptedGaps) < 0) = [];
% Velocity_Gaps_accepted_range(abs(GapFeatures_SVM.F_vehVel(AcceptedGaps)) < 0.2) = [];
% Velocity_Gaps_accepted_range(abs(GapFeatures_SVM.F_vehVel(AcceptedGaps)) < 3) = [];


%identify outliers
figure()
b = boxplot(Velocity_Gaps_accepted_range);
% max outlier = 13.22; min outlier = 0.01; 25th = 4.10; 75th = 7.76; 50th =
% 5.44
% 

% remove outliers of accepted gaps
Velocity_Gaps_accepted_range(Velocity_Gaps_accepted_range > 12.90) = [];


% plot histogram
figure()
h = histogram(Velocity_Gaps_accepted_range,'Normalization','probability');
xlabel('Time Gap [s]')
ylabel('Probability')
title('Gap acceptance probability')

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









end