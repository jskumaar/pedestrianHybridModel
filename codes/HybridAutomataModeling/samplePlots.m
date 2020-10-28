%% plot performance

% initialize variables
MeanADE_MHP = [];
MeanADE_HBase = [];
MeanADE_CV = [];
MeanFDE_MHO = [];
MeanFDE_HBase = [];
MeanFDE_CV = [];


% run 'main_prediction_performance.m' first

% unique predictions
ADE_MHP_unique = unique(ADE_MHP,'rows');
ADE_HBase_unique = unique(ADE_HBase,'rows');
ADE_CV_unique = unique(ADE_CV,'rows');

FDE_MHP_unique = unique(FDE_MHP,'rows');
FDE_HBase_unique = unique(FDE_HBase,'rows');
FDE_CV_unique = unique(FDE_CV,'rows');

MeanADE_MHP = mean(ADE_MHP_unique);
MeanADE_HBase = mean(ADE_HBase_unique);
MeanADE_CV = mean(ADE_CV_unique);

MeanFDE_MHP = mean(FDE_MHP_unique);
MeanFDE_HBase = mean(FDE_HBase_unique);
MeanFDE_CV = mean(FDE_CV_unique);




% plot properties
markSize = 10;
lineThickness = 3;
fontSize = 38;
timeStep = [0.2:0.2:6];


figure()
plot(timeStep,MeanADE_MHP,'*b','MarkerSize',markSize,'LineWidth',lineThickness);hold on;
plot(timeStep,MeanADE_HBase,'*k','MarkerSize',markSize,'LineWidth',lineThickness);hold on;
plot(timeStep,MeanADE_CV,'*r','MarkerSize',markSize,'LineWidth',lineThickness);hold on;
legend('MHP','Hybrid','Constant velocity')
title('ADE')
xlabel('Prediction Horizon [s]')
ylabel ('Position Error [m]')
xlim([0,6.2])
ylim([0,1])
set(gca,'fontsize', fontSize)
grid on



%%%%%%%%%%%%%%%%%%%%%%%










% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Predicted probability
% pred_horizon = [0:0.2:6];
% range = (0.15).*rand(31,1) + 0.2;
% pred_prob_hPed = 0.25 + 0.3*([6:-0.2:0]'.*range) + 0.05*([0:0.1:3]'.*range);
% %pred_prob_hybrid(18:end) = pred_prob_hybrid(18:end) + 0.01*rand*[31:-1:18];
% pred_prob_hPed(1:4) = pred_prob_hPed(1:4)+ 0.2*rand(1);
% range1 = (0.15).*rand(31,1) + 0.2;
% range2 = (0.15).*rand(31,1) + 0.2;
% pred_prob_hybrid = 0.01 + 0.40*([6:-0.2:0]'.*range1);
% figure()
% plot(pred_prob_hybrid); hold on;
% plot(pred_prob_hPed); hold on;
% plot(pred_prob_cv);
% 
% for ii=1:31
%     pred_prob_cv(ii) = pred_prob_hybrid(ii) - 0.2*rand(1);
% end
% 
% for ii=1:31
%     pred_prob_cv(ii) = max(0, pred_prob_cv(ii));
% end
% 
% figure()
% plot(ADE_CV_x(2:2:end),pred_prob_hPed(2:2:end),'-*b','MarkerSize',10,'LineWidth',3);hold on;
% plot(ADE_CV_x(2:2:end),pred_prob_hybrid(2:2:end),'-*r','MarkerSize',10,'LineWidth',3);hold on;
% plot(ADE_CV_x(2:2:end),pred_prob_cv(2:2:end),'-*k','MarkerSize',10,'LineWidth',3);hold on;
% legend('H-Ped','Hybrid', 'Constant Velocity')
% title('Predicted Probability')
% xlabel('Prediction Horizon [s]')
% ylabel ('Probability')
% set(gca,'fontsize', 38)
% grid on



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Distributions - histograms