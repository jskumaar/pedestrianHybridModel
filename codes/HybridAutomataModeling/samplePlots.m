%% plot performance

% initialize variables
MeanPP_MHP = [];
MeanPP_HBase = [];
MeanPP_CV = [];
MeanFDE_MHO = [];
MeanLL_HBase = [];
MeanLL_CV = [];


% run 'main_prediction_performance.m' first

% % unique predictions
% [ADE_MHP_unique, unique_rows] = unique(ADE_MHP,'rows');
% ADE_HBase_unique = ADE_HBase(unique_rows,:);
% ADE_CV_unique = ADE_CV(unique_rows,:);
% 
% [FDE_MHP_unique, unique_rows] = unique(FDE_MHP,'rows');
% FDE_HBase_unique = FDE_HBase(unique_rows,:);
% FDE_CV_unique = FDE_CV(unique_rows,:);
% 
% MeanADE_MHP = mean(ADE_MHP_unique);
% MeanADE_HBase = mean(ADE_HBase_unique);
% MeanADE_CV = mean(ADE_CV_unique);
% 
% MeanFDE_MHP = mean(FDE_MHP_unique);
% MeanFDE_HBase = mean(FDE_HBase_unique);
% MeanFDE_CV = mean(FDE_CV_unique);




% plot properties
markSize = 10;
lineThickness = 3;
fontSize = 38;
timeStep = [0.2:0.2:6];

% % plot average displacement error
% figure()
% plot(timeStep,MeanADE_MHP,'*b','MarkerSize',markSize,'LineWidth',lineThickness);hold on;
% plot(timeStep,MeanADE_HBase,'*k','MarkerSize',markSize,'LineWidth',lineThickness);hold on;
% plot(timeStep,MeanADE_CV,'*r','MarkerSize',markSize,'LineWidth',lineThickness);hold on;
% legend('MHP','Hybrid','Constant velocity')
% title('ADE')
% xlabel('Prediction Horizon [s]')
% ylabel ('Position Error [m]')
% xlim([0,6.2])
% ylim([0,3])
% set(gca,'fontsize', fontSize)
% grid on

% % % plot final displacement error
% figure()
% plot(timeStep,MeanFDE_MHP,'*b','MarkerSize',markSize,'LineWidth',lineThickness);hold on;
% plot(timeStep,MeanFDE_HBase,'*k','MarkerSize',markSize,'LineWidth',lineThickness);hold on;
% plot(timeStep,MeanFDE_CV,'*r','MarkerSize',markSize,'LineWidth',lineThickness);hold on;
% legend('MHP','Hybrid','Constant velocity')
% title('FDE')
% xlabel('Prediction Horizon [s]')
% ylabel ('Position Error [m]')
% xlim([0,6.2])
% ylim([0,5])
% set(gca,'fontsize', fontSize)
% grid on

%%%%%%%%%%%%%%%%%%%%%%%

%% probabilistic metrics
% unique predictions
[PP_MHP_unique, unique_rows] = unique(PP_MHP,'rows');
PP_HBase_unique = PP_HBase(unique_rows,:);
PP_CV_unique = PP_CV(unique_rows,:);

[LL_MHP_unique, unique_rows] = unique(LL_MHP,'rows');
LL_HBase_unique = LL_HBase(unique_rows,:);
LL_CV_unique = LL_CV(unique_rows,:);

MeanPP_MHP = mean(PP_MHP_unique);
MeanPP_HBase = mean(PP_HBase_unique);
MeanPP_CV = mean(PP_CV_unique);

MeanLL_MHP = mean(LL_MHP_unique);
MeanLL_HBase = mean(LL_HBase_unique);
MeanLL_CV = mean(LL_CV_unique);

% plot average displacement error
figure()
plot(timeStep,MeanPP_MHP,'*b','MarkerSize',markSize,'LineWidth',lineThickness);hold on;
plot(timeStep,MeanPP_HBase,'*k','MarkerSize',markSize,'LineWidth',lineThickness);hold on;
plot(timeStep,MeanPP_CV,'*r','MarkerSize',markSize,'LineWidth',lineThickness);hold on;
legend('MHP','Hybrid','Constant velocity')
title('ADE')
xlabel('Prediction Horizon [s]')
ylabel ('Position Error [m]')
xlim([0,6.2])
ylim([0,0.2])
set(gca,'fontsize', fontSize)
grid on









%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
