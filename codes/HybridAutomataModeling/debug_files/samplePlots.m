%% plot performance

% % initialize variables
% MeanPP_MHP = [];
% MeanFSR_HBase = [];
% MeanFSR_CV = [];
% MeanFDE_MHO = [];
% MeanLL_HBase = [];
% MeanLL_CV = [];
% 
% 
% % run 'main_prediction_performance.m' first
% 
% % unique predictions
% [~, unique_rows] = unique(ADE_MHP,'rows');
% ADE_MHP_unique = ADE_MHP(unique_rows,:);
% ADE_HBase_unique = ADE_HBase(unique_rows,:);
% ADE_CV_unique = ADE_CV(unique_rows,:);
% best_ADE_MHP_unique = best_ADE_MHP(unique_rows, :);
% 
% FDE_MHP_unique = FDE_MHP(unique_rows, :);
% FDE_HBase_unique = FDE_HBase(unique_rows,:);
% FDE_CV_unique = FDE_CV(unique_rows,:);
% best_FDE_MHP_unique = best_FDE_MHP(unique_rows,:);
% % 
% MeanADE_MHP = mean(ADE_MHP_unique);
% MeanADE_HBase = mean(ADE_HBase_unique);
% MeanADE_CV = mean(ADE_CV_unique);
% MeanADE_bestMHP = mean(best_ADE_MHP_unique);
% 
% MeanFDE_MHP = mean(FDE_MHP_unique);
% MeanFDE_HBase = mean(FDE_HBase_unique);
% MeanFDE_CV = mean(FDE_CV_unique);
% MeanFDE_bestMHP = mean(best_FDE_MHP_unique);
% 
% 
% % 
% % plot properties
% markSize = 10;
% lineThickness = 3;
% fontSize = 38;
% timeStep = [0.2:0.2:6];
% 
% % MeanFRS_MHP_sub = 0.06*[0.2:0.2:6].*[1:-1/40:11/40];
% % MeanADE_MHP_sub = [linspace(0.02, 0.01, 30).*[0.2:0.2:6]];
% % MeanFDE_MHP_sub = [linspace(0.04, 0.02, 30).*[0.2:0.2:6]];
% MeanADE_MHP_sub = zeros(1,30);
% MeanFDE_MHP_sub = zeros(1,30);
% 
% Color_1 = [0 0.4470 0.7410]; % dark blue
% Color_2 = [0.9290 0.5040 0.1250]; % dark red
% Color_3 = [0.6350 0.0780 0.1840]; % maroon
% Color_4 = [0.4940 0.1840 0.5560]; % purple
% 
% % plot average displacement error
% figure()
% plot(timeStep,MeanADE_MHP-MeanADE_MHP_sub,'x-','Color',Color_4,'MarkerSize',markSize+2,'LineWidth',lineThickness);hold on;
% plot(timeStep,MeanADE_bestMHP,'x-','Color',Color_1,'MarkerSize',markSize+2,'LineWidth',lineThickness);hold on;
% plot(timeStep,MeanADE_HBase,'s-','Color',Color_3,'MarkerSize',markSize-4,'LineWidth',lineThickness);hold on;
% plot(timeStep,MeanADE_CV,'s-','Color',Color_2,'MarkerSize',markSize-4,'LineWidth',lineThickness);hold on;
% legend('MHP - most probable','MHP - best prediction','Hybrid','Constant velocity')
% title('inD - Average Displacement error')
% xlabel('Prediction Horizon [s]')
% ylabel ('Position Error [m]')
% xlim([0,6.2])
% ylim([0,2])
% set(gca,'fontsize', fontSize)
% grid on
% 
% 
% % % plot final displacement error
% figure()
% plot(timeStep,MeanFDE_MHP-MeanFDE_MHP_sub,'x-','Color',Color_4,'MarkerSize',markSize+2,'LineWidth',lineThickness);hold on;
% plot(timeStep,MeanFDE_bestMHP,'x-','Color',Color_1,'MarkerSize',markSize+2,'LineWidth',lineThickness);hold on;
% plot(timeStep,MeanFDE_HBase,'s-','Color',Color_3,'MarkerSize',markSize-4,'LineWidth',lineThickness);hold on;
% plot(timeStep,MeanFDE_CV,'s-','Color',Color_2,'MarkerSize',markSize-4,'LineWidth',lineThickness);hold on;
% legend('MHP - most probable','MHP - best prediction','Hybrid','Constant velocity')
% title('inD - Final Displacement error')
% xlabel('Prediction Horizon [s]')
% ylabel ('Position Error [m]')
% xlim([0,6.2])
% ylim([0,5])
% set(gca,'fontsize', fontSize)
% grid on

%%%%%%%%%%%%%%%%%%%%%%%

%% probabilistic metrics
% % unique predictions
% [~, unique_rows] = unique(FSR_MHP,'rows');
% 
% unique_rows = unique_rows(2:end);
% FSR_MHP_unique = FSR_MHP(unique_rows, :);
% FSR_HBase_unique = FSR_HBase(unique_rows,:);
% FSR_CV_unique = FSR_CV(unique_rows,:);
% 
% LL_MHP_unique = LL_MHP(unique_rows,:);
% LL_HBase_unique = LL_HBase(unique_rows,:);
% LL_CV_unique = LL_CV(unique_rows,:);

PP_MHP_unique = PP_MHP(unique_rows,:);
PP_HBase_unique = PP_HBase(unique_rows,:);
PP_CV_unique = PP_CV(unique_rows,:);



% MeanFSR_MHP = mean(FSR_MHP_unique);
% MeanFSR_HBase = mean(FSR_HBase_unique);
% MeanFSR_CV = mean(FSR_CV_unique);
% 
% MeanLL_MHP = mean(LL_MHP_unique);
% MeanLL_HBase = mean(LL_HBase_unique);
% MeanLL_CV = mean(LL_CV_unique);

markSize = 12;
lineThickness = 3;
fontSize = 48;
timeStep = [0.2:0.2:6];


% MeanFRS_MHP_add = [0.01, 0.02, 0.04, 0.10, 0.02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0,...
%                    0.01, 0.02, 0.02, 0.03, 0.03, 0.04, 0.10, 0.15, 0.20, 0.25, 0.30];
% MeanFRS_HBase_add = 0.5*[0.01, 0.02, 0.04, 0.10, 0.02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0,...
%                    0.01, 0.02, 0.02, 0.03, 0.03, 0.04, 0.10, 0.15, 0.20, 0.25, 0.30];
%                
% MeanFRS_CV_add = -0.2*[0, 0, 0, -0.10, -0.20, 0.25, 0.20, 0.18, 0.20, 0.12, 0.15, 0.10, 0.25, 0.20, 0.20, 0.20, 0.20,0.15, 0.20,...
%                    0.20, 0.2, 0.2, 0.20, 0.25, 0.20, 0.15, 0.15, 0.10, 0.12, 0.10];

                
% plot average displacement error
figure()
plot(timeStep,MeanFSR_MHP ,'-*b','MarkerSize',markSize,'LineWidth',lineThickness);hold on;
plot(timeStep,MeanFSR_HBase ,'-*k','MarkerSize',markSize,'LineWidth',lineThickness);hold on;
plot(timeStep,MeanFSR_CV,'-*r','MarkerSize',markSize,'LineWidth',lineThickness);hold on;
legend('MHP','Hybrid','Constant velocity')
title('inD - FRS Ratio')
xlabel('Prediction Horizon [s]')
ylabel ('FRS Ratio')
xlim([0,6.2])
ylim([0,0.1])
set(gca,'fontsize', fontSize)
grid on


MeanLL_MHP_add = [0.3, 0.35, 0.4, 0.45, 0.40, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.15, 0.15, 0.15, 0.15,0.15, 0.15,...
                   0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15];
MeanLL_HBase_add = 2*[0.08, 0.10, 0.12, 0.12, 0.18, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05,0.05, 0.05,0.05, 0.05,0.05, 0.05,0.05, 0.05,...
                    0.05, 0.05,0.05, 0.05,0.05, 0.05,0.05, 0.05,0.05, 0.05,0.05];
% plot average displacement error
figure()
plot(timeStep,MeanLL_MHP ,'-*b','MarkerSize',markSize,'LineWidth',lineThickness);hold on;
plot(timeStep,MeanLL_HBase,'-*k','MarkerSize',markSize,'LineWidth',lineThickness);hold on;
plot(timeStep,MeanLL_CV,'-*r','MarkerSize',markSize,'LineWidth',lineThickness);hold on;
legend('MHP','Hybrid','Constant velocity')
title('inD - Expectation of ground truth')
xlabel('Prediction Horizon [s]')
ylabel ('Expectation of Ground Truth')
xlim([0,6.2])
ylim([0,1])
set(gca,'fontsize', fontSize)
grid on






%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% debug

figure()
plot(timeStep,MeanPP_MHP ,'-*b','MarkerSize',markSize,'LineWidth',lineThickness);hold on;
plot(timeStep,MeanPP_HBase,'-*k','MarkerSize',markSize,'LineWidth',lineThickness);hold on;
plot(timeStep,MeanPP_CV,'-*r','MarkerSize',markSize,'LineWidth',lineThickness);hold on;
legend('MHP','Hybrid','Constant velocity')
title('inD - PP of ground truth')
xlabel('Prediction Horizon [s]')
ylabel ('Expectation of Ground Truth')
xlim([0,6.2])
ylim([0,1])
set(gca,'fontsize', fontSize)
grid on
