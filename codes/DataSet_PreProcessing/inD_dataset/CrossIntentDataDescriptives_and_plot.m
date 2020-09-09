%% This script caluclates the cross intent descriptives

% a) addpath of necessary directories
p1 = genpath('G:\My Drive\Research\Projects\pedestrianHybridModel\codes');
p2 = genpath('G:\My Drive\Research\Projects\pedestrianHybridModel\datasets');
addpath(p1)
addpath(p2)

%load('CrossIntentData.mat')
AllIndices = [1:height(CrossIntentData)];

IntentDataIndices = find(CrossIntentData.cross_intent==1);
NoIntentDataIndices = find(CrossIntentData.cross_intent==0);





% plot
typeOfGap = NoIntentDataIndices;

VehSpeed = CrossIntentData.mean_veh_speed(typeOfGap);
VehAcc = CrossIntentData.mean_veh_acc(typeOfGap);
VehPedDist = CrossIntentData.mean_veh_ped_dist(typeOfGap);
DTCurb = CrossIntentData.mean_DTCurb(typeOfGap);
DTCW = CrossIntentData.mean_DTCW(typeOfGap);
duration = CrossIntentData.duration_ego_vehicle(typeOfGap);
gazeRatio = CrossIntentData.gaze_ratio(typeOfGap);
direction = CrossIntentData.direction(typeOfGap);

%% Histograms
figure()

subplot(2,4,1)
BinWidth = 1;
%NumberOfBins = 15;
h=histogram(VehSpeed,'BinWidth',BinWidth,'Normalization','probability');   %to plot probability densityh=histogram(Data,NumberOfBins,'Normalization','probability');   %to plot probability density
%h.FaceColor = 'none';       %adjust colour of the bin
title('VehSpeed')
set(gca,'fontsize', 11)

subplot(2,4,2)
BinWidth = 0.5;
%NumberOfBins = 15;
h=histogram(VehAcc,'BinWidth',BinWidth,'Normalization','probability');   %to plot probability densityh=histogram(Data,NumberOfBins,'Normalization','probability');   %to plot probability density
%h.FaceColor = 'none';       %adjust colour of the bin
title('VehAcc')
set(gca,'fontsize', 11)

subplot(2,4,3)
BinWidth = 0.5;
%NumberOfBins = 15;
h=histogram(VehPedDist,'BinWidth',BinWidth,'Normalization','probability');   %to plot probability densityh=histogram(Data,NumberOfBins,'Normalization','probability');   %to plot probability density
%h.FaceColor = 'none';       %adjust colour of the bin
title('VehPedDist')
set(gca,'fontsize', 11)

subplot(2,4,4)
BinWidth = 0.25;
%NumberOfBins = 15;
h=histogram(DTCurb,'BinWidth',BinWidth,'Normalization','probability');   %to plot probability densityh=histogram(Data,NumberOfBins,'Normalization','probability');   %to plot probability density
%h.FaceColor = 'none';       %adjust colour of the bin
title('DTCurb')
set(gca,'fontsize', 11)

subplot(2,4,5)
BinWidth = 0.25;
%NumberOfBins = 15;
h=histogram(DTCW,'BinWidth',BinWidth,'Normalization','probability');   %to plot probability densityh=histogram(Data,NumberOfBins,'Normalization','probability');   %to plot probability density
%h.FaceColor = 'none';       %adjust colour of the bin
title('DTCW')
set(gca,'fontsize', 11)

subplot(2,4,6)
BinWidth = 0.1;
%NumberOfBins = 15;
h=histogram(gazeRatio,'BinWidth',BinWidth,'Normalization','probability');   %to plot probability densityh=histogram(Data,NumberOfBins,'Normalization','probability');   %to plot probability density
%h.FaceColor = 'none';       %adjust colour of the bin
title('gazeRatio')
set(gca,'fontsize', 11)

subplot(2,4,7)
BinWidth = 0.1;
%NumberOfBins = 15;
h=histogram(duration,'BinWidth',BinWidth,'Normalization','probability');   %to plot probability densityh=histogram(Data,NumberOfBins,'Normalization','probability');   %to plot probability density
%h.FaceColor = 'none';       %adjust colour of the bin
title('duration')
set(gca,'fontsize', 11)

subplot(2,4,8)
%BinWidth = 0.5;
NumberOfBins = 2;
h=histogram(direction,NumberOfBins,'Normalization','probability');   %to plot probability densityh=histogram(Data,NumberOfBins,'Normalization','probability');   %to plot probability density
%h.FaceColor = 'none';       %adjust colour of the bin
title('direction')
set(gca,'fontsize', 11)