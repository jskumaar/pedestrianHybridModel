%% Performance compilation and plots

clear all
delT = 0.1;
% %% Plot the overall error
% 
CV_GTEuclideanDistanceError = [];
Hybrid_GTEuclideanDistanceError = [];
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
filename = strcat('Performance_CV_KF_',num2str(100),'_SVM_StartGap.mat');
load(filename);
% 
for mm=1:105
    CV_GTEuclideanDistanceError = [CV_GTEuclideanDistanceError;Performance{mm}.GTEuclideanDistanceError(Performance{mm}.WaitCrossCheckIndices,:)];
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

filename = strcat('Performance_Hybrid_KF_',num2str(100),'_SVM_StartGap.mat');
load(filename);

GTRMSE_TestTraj_CV         = [];
GTRMSE_TestTraj_Hybrid     = [];
Duration_TestTraj       = [];
WaitDuration_TestTraj   = [];

for mm=1:96 
   GTRMSE_TestTraj_CV = [GTRMSE_TestTraj_CV;Performance_CV{mm}.MeanGTAverageEuclideanError];
   GTRMSE_TestTraj_Hybrid = [GTRMSE_TestTraj_Hybrid;Performance_Hybrid{mm}.MeanGTAverageEuclideanError];
   
   Duration_TestTraj = [Duration_TestTraj; length(Performance_Hybrid{mm}.Estimated_x)];
   
   if(~isempty(Performance_Hybrid{mm}.WaitStart))
        WaitDuration_TestTraj = [WaitDuration_TestTraj;(Performance_Hybrid{mm}.CrossStart - Performance_Hybrid{mm}.WaitStart)* delT];
   else
        WaitDuration_TestTraj = [WaitDuration_TestTraj;0];
   end
   
end


figure()
subplot(2,2,1)
plot(GTRMSE_TestTraj_Hybrid.*(Duration_TestTraj-WaitDuration_TestTraj),'*');hold on;
plot(GTRMSE_TestTraj_CV.*(Duration_TestTraj-WaitDuration_TestTraj),'k.','MarkerSize',8);hold on;
ylabel('Overall Euc. error for non-waiting time instances');

subplot(2,2,2)
plot(Duration_TestTraj,'*');
ylabel('Overall duration');

subplot(2,2,3)
plot(WaitDuration_TestTraj,'*');
ylabel('Wait Duration');


subplot(2,2,4)
plot(GTRMSE_TestTraj_Hybrid,'*');hold on;
plot(GTRMSE_TestTraj_CV,'k.','MarkerSize',8);hold on;
ylabel('Mean RMSE error');


%%
% 1) from the above plots,
% cross indices = 45 (Subject 3 UnSig Nor (2) Crossing 3), 238 (Subject 14 Unsig Agg (1) Crossing 4) have good prediction results
% 2) copy the vehicle trajectories from raw excel sheet for the corresponding
% approach to retreat indices

% crossing45Veh = [];
% crossing238Veh = [];

crossing45approachStart = 1360;
crossing45retreatEnd = 2107;

crossing238approachStart = 1747;
crossing238retreatEnd = 2118;

%% Correlations
% In what situations are the prediction errors high? low? What factors
% characterize such situations? Try a GLMP model to check significance.
% Normalize the variables.

R = corrcoef(WaitDuration_TestTraj,GTRMSE_TestTraj);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


CompiledTransitionCheckData = [];
for mm=1:105

    if ~isempty(Performance{mm}.TransitionCheck)
%         tempDiff = diff(Performance{mm}.TransitionCheck(:,6));
%         ind = find(tempDiff>15);
%         ind=[ind;length(Performance{mm}.TransitionCheck(:,6))];  %the most recent check for a gap

        CompiledTransitionCheckData = [CompiledTransitionCheckData;Performance{mm}.TransitionCheck];
    end
%         count = count + 1;   
end
    
x=1;
% ApproachToWaitIndices = find(CompiledTransitionCheckData(:,13)==1 & CompiledTransitionCheckData(:,18)==2);
% ApproachToCrossIndices = find(CompiledTransitionCheckData(:,13)==1 & CompiledTransitionCheckData(:,18)==3);
% WaitToWaitIndices = find(CompiledTransitionCheckData(:,13)==2 & CompiledTransitionCheckData(:,18)==2);
% WaitToCrossIndices = find(CompiledTransitionCheckData(:,13)==2 & CompiledTransitionCheckData(:,18)==3);
% 
% [ApproachToWaitPerformance] = probabilisticPredictionPerformance(CompiledTransitionCheckData(ApproachToWaitIndices,16),CompiledTransitionCheckData(ApproachToWaitIndices,15));
% [ApproachToCrossPerformance] = probabilisticPredictionPerformance(CompiledTransitionCheckData(ApproachToCrossIndices,16),CompiledTransitionCheckData(ApproachToCrossIndices,15));
% [WaitToWaitPerformance] = probabilisticPredictionPerformance(CompiledTransitionCheckData(WaitToWaitIndices,16),CompiledTransitionCheckData(WaitToWaitIndices,15));
% [WaitToCrossIndicesPerformance] = probabilisticPredictionPerformance(CompiledTransitionCheckData(WaitToCrossIndices,16),CompiledTransitionCheckData(WaitToCrossIndices,15));

%%%%%%%
% %% Only for logistic regression  - indices vary
% ApproachToWaitIndices = find(CompiledTransitionCheckData(:,13)==1 & CompiledTransitionCheckData(:,17)==2);
% ApproachToCrossIndices = find(CompiledTransitionCheckData(:,13)==1 & CompiledTransitionCheckData(:,17)==3);
% WaitToWaitIndices = find(CompiledTransitionCheckData(:,13)==2 & CompiledTransitionCheckData(:,17)==2);
% WaitToCrossIndices = find(CompiledTransitionCheckData(:,13)==2 & CompiledTransitionCheckData(:,17)==3);
% 
% [ApproachToWaitPerformance] = probabilisticPredictionPerformance(CompiledTransitionCheckData(ApproachToWaitIndices,15),CompiledTransitionCheckData(ApproachToWaitIndices,14));
% [ApproachToCrossPerformance] = probabilisticPredictionPerformance(CompiledTransitionCheckData(ApproachToCrossIndices,15),CompiledTransitionCheckData(ApproachToCrossIndices,14));
% [WaitToWaitPerformance] = probabilisticPredictionPerformance(CompiledTransitionCheckData(WaitToWaitIndices,15),CompiledTransitionCheckData(WaitToWaitIndices,14));
% [WaitToCrossIndicesPerformance] = probabilisticPredictionPerformance(CompiledTransitionCheckData(WaitToCrossIndices,15),CompiledTransitionCheckData(WaitToCrossIndices,14));



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
CV_FDE = mean(CV_GTEuclideanDistanceError);
CV_FDE_SD = std(CV_GTEuclideanDistanceError);

for ii=1:100
    CV_ADE(ii) = mean(mean(CV_GTEuclideanDistanceError(:,1:ii),2));
    CV_ADE_SD(ii) = std(mean(CV_GTEuclideanDistanceError(:,1:ii),2));
    
    CV_RMSE(ii) = mean(sqrt(sum(CV_GTEuclideanDistanceError(:,1:ii).^2,2)/(ii)));
    CV_RMSE_SD(ii) = std(sqrt(sum(CV_GTEuclideanDistanceError(:,1:ii).^2,2)/(ii)));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Hybrid_FDE = mean(Hybrid_GTEuclideanDistanceError);
Hybrid_FDE_SD = std(Hybrid_GTEuclideanDistanceError);

for ii=1:100
    Hybrid_ADE(ii) = mean(mean(Hybrid_GTEuclideanDistanceError(:,1:ii),2));
    Hybrid_ADE_SD(ii) = std(mean(Hybrid_GTEuclideanDistanceError(:,1:ii),2));
    
    Hybrid_RMSE(ii) = mean(sqrt(sum(Hybrid_GTEuclideanDistanceError(:,1:ii).^2,2)/(ii)));
    Hybrid_RMSE_SD(ii) = std(sqrt(sum(Hybrid_GTEuclideanDistanceError(:,1:ii).^2,2)/(ii)));
end

hor=60;


figure()
subplot(1,3,3)
plot([1:hor]*0.1,Hybrid_RMSE(1:hor),'.b','MarkerSize',30,'LineWidth',5);hold on;
plot([1:hor]*0.1,CV_RMSE(1:hor),'.r','MarkerSize',30,'LineWidth',5);hold on;
legend('Hybrid','Constant velocity')
title('RMSE Error')
xlabel('Prediction Horizon [s]')
ylabel ('Position Error [m]')
ylim([0,5])
grid on
set(gca,'fontsize', 48)

subplot(1,3,1)
plot([1:hor]*0.1,Hybrid_ADE(1:hor),'.b','MarkerSize',30,'LineWidth',5);hold on;
plot([1:hor]*0.1,CV_ADE(1:hor),'.r','MarkerSize',30,'LineWidth',5);hold on;
legend('Hybrid','Constant velocity')
title('Average displacement error')
xlabel('Prediction Horizon [s]')
ylabel ('Position Error [m]')
ylim([0,5])
grid on
set(gca,'fontsize', 48)

subplot(1,3,2)
plot([1:hor]*0.1,Hybrid_FDE(1:hor),'.b','MarkerSize',30,'LineWidth',5);hold on;
plot([1:hor]*0.1,CV_FDE(1:hor),'.r','MarkerSize',30,'LineWidth',5);hold on;
legend('Hybrid','Constant velocity')
title('Final displacement error')
xlabel('Prediction Horizon [s]')
ylabel ('Position Error [m]')
ylim([0,5])
grid on
set(gca,'fontsize', 48)

% figure()
% plot(pred_Horizon_matrix*0.1,MeanGTMHD_Hybrid,'-*b','MarkerSize',10,'LineWidth',3);hold on;
% plot(pred_Horizon_matrix*0.1,MeanGTMHD_CV,'-*r','MarkerSize',10,'LineWidth',3);hold on;
% legend('Hybrid','Constant velocity')
% title('Modified Hausdorff Distance')
% xlabel('Prediction Horizon [s]')
% ylabel ('Position Error [m]')
% set(gca,'fontsize', 38)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure()
plot(pred_Horizon_matrix*0.1,TranisitionMeanGTRMSE_Hybrid,'-*b','MarkerSize',10,'LineWidth',3);hold on;
plot(pred_Horizon_matrix*0.1,TranisitionMeanGTRMSE_CV,'-*r','MarkerSize',10,'LineWidth',3);hold on;
legend('Hybrid','Constant velocity')
title('RMSE Error')
xlabel('Prediction Horizon [s]')
ylabel ('Position Error [m]')
set(gca,'fontsize', 38)

figure()
plot(pred_Horizon_matrix*0.1,TranisitionMeanGTAverageEuclideanError_Hybrid,'-*b','MarkerSize',10,'LineWidth',3);hold on;
plot(pred_Horizon_matrix*0.1,TranisitionMeanGTAverageEuclideanError_CV,'-*r','MarkerSize',10,'LineWidth',3);hold on;
legend('Hybrid','Constant velocity')
title('Average displacement error')
xlabel('Prediction Horizon [s]')
ylabel ('Position Error [m]')
set(gca,'fontsize', 38)

figure()
plot(pred_Horizon_matrix*0.1,TranisitionGTEndEuclideanError_Hybrid,'-*b','MarkerSize',10,'LineWidth',3);hold on;
plot(pred_Horizon_matrix*0.1,TranisitionGTEndEuclideanError_CV,'-*r','MarkerSize',10,'LineWidth',3);hold on;
legend('Hybrid','Constant velocity')
title('Final displacement error')
xlabel('Prediction Horizon [s]')
ylabel ('Position Error [m]')
set(gca,'fontsize', 38)

figure()
plot(pred_Horizon_matrix*0.1,TranisitionMeanGTMHD_Hybrid,'-*b','MarkerSize',10,'LineWidth',3);hold on;
plot(pred_Horizon_matrix*0.1,TranisitionMeanGTMHD_CV,'-*r','MarkerSize',10,'LineWidth',3);hold on;
legend('Hybrid','Constant velocity')
title('Modified Hausdorff Distance')
xlabel('Prediction Horizon [s]')
ylabel ('Position Error [m]')
set(gca,'fontsize', 38)
















