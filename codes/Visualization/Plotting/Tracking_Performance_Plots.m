%% Performance compilation and plots

clear all

%% Plot the overall error

pred_Horizon_matrix = [10,15,20,25,30,35,40,45,50,55,60];

N = length(pred_Horizon_matrix);
CV_MeanGTRMSE = cell(N,1);
CV_MeanGTAverageEuclideanError = cell(N,1);
CV_MeanGTEndEuclideanError = cell(N,1);
CV_MeanGTMHD = cell(N,1);
CV_TranisitionMeanGTRMSE = cell(N,1);
CV_TranisitionMeanGTAverageEuclideanError = cell(N,1);
CV_TranisitionGTEndEuclideanError = cell(N,1);
CV_TranisitionMeanGTMHD = cell(N,1);


for gg=1:11

    filename = strcat('Performance_CV_',num2str(pred_Horizon_matrix(gg)),'_SVM_StartGap.mat');
    load(filename);
    
    for mm=1:105
        CV_MeanGTRMSE{gg} = [CV_MeanGTRMSE{gg};Performance{mm}.MeanGTRMSE];
        CV_MeanGTAverageEuclideanError{gg} = [CV_MeanGTAverageEuclideanError{gg};Performance{mm}.MeanGTAverageEuclideanError];
        CV_MeanGTEndEuclideanError{gg} = [CV_MeanGTEndEuclideanError{gg};Performance{mm}.MeanGTEndEuclideanError];
        CV_MeanGTMHD{gg} = [CV_MeanGTMHD{gg};Performance{mm}.MeanGTMHD];
        CV_TranisitionMeanGTRMSE{gg} = [CV_TranisitionMeanGTRMSE{gg};Performance{mm}.TranisitionMeanGTRMSE];
        CV_TranisitionMeanGTAverageEuclideanError{gg} = [CV_TranisitionMeanGTAverageEuclideanError{gg};Performance{mm}.TranisitionMeanGTAverageEuclideanError];
        CV_TranisitionGTEndEuclideanError{gg} = [CV_TranisitionGTEndEuclideanError{gg};Performance{mm}.TranisitionGTEndEuclideanError];
        CV_TranisitionMeanGTMHD{gg} = [CV_TranisitionMeanGTMHD{gg};Performance{mm}.TranisitionMeanGTMHD];
    end
    
    

end

MeanGTRMSE_CV = cellfun(@mean,CV_MeanGTRMSE);
MeanGTAverageEuclideanError_CV = cellfun(@mean,CV_MeanGTAverageEuclideanError);
MeanGTEndEuclideanError_CV = cellfun(@mean,CV_MeanGTEndEuclideanError);
MeanGTMHD_CV = cellfun(@mean,CV_MeanGTMHD);

TranisitionMeanGTRMSE_CV = cellfun(@mean,CV_TranisitionMeanGTRMSE);
TranisitionMeanGTAverageEuclideanError_CV = cellfun(@mean,CV_TranisitionMeanGTAverageEuclideanError);
TranisitionGTEndEuclideanError_CV = cellfun(@mean,CV_TranisitionGTEndEuclideanError);
TranisitionMeanGTMHD_CV = cellfun(@mean,CV_TranisitionMeanGTMHD);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





Hybrid_MeanGTRMSE = cell(N,1);
Hybrid_MeanGTAverageEuclideanError = cell(N,1);
Hybrid_MeanGTEndEuclideanError = cell(N,1);
Hybrid_MeanGTMHD = cell(N,1);
Hybrid_TranisitionMeanGTRMSE = cell(N,1);
Hybrid_TranisitionMeanGTAverageEuclideanError = cell(N,1);
Hybrid_TranisitionGTEndEuclideanError = cell(N,1);
Hybrid_TranisitionMeanGTMHD = cell(N,1);


ApproachToWaitTTE_Error = cell(N,1);
ApproachToCrossTTE_Error = cell(N,1);
WaitToCrossTTE_Error = cell(N,1);
CrossToRetreatTTE_Error = cell(N,1);

for gg=1:11

    filename = strcat('Performance_Hybrid_',num2str(pred_Horizon_matrix(gg)),'_SVM_StartGap.mat');
    load(filename);
    
    for mm=1:105
        Hybrid_MeanGTRMSE{gg} = [Hybrid_MeanGTRMSE{gg};Performance{mm}.MeanGTRMSE];
        Hybrid_MeanGTAverageEuclideanError{gg} = [Hybrid_MeanGTAverageEuclideanError{gg};Performance{mm}.MeanGTAverageEuclideanError];
        Hybrid_MeanGTEndEuclideanError{gg} = [Hybrid_MeanGTEndEuclideanError{gg};Performance{mm}.MeanGTEndEuclideanError];
        Hybrid_MeanGTMHD{gg} = [Hybrid_MeanGTMHD{gg};Performance{mm}.MeanGTMHD];
        Hybrid_TranisitionMeanGTRMSE{gg} = [Hybrid_TranisitionMeanGTRMSE{gg};Performance{mm}.TranisitionMeanGTRMSE];
        Hybrid_TranisitionMeanGTAverageEuclideanError{gg} = [Hybrid_TranisitionMeanGTAverageEuclideanError{gg};Performance{mm}.TranisitionMeanGTAverageEuclideanError];
        Hybrid_TranisitionGTEndEuclideanError{gg} = [Hybrid_TranisitionGTEndEuclideanError{gg};Performance{mm}.TranisitionGTEndEuclideanError];
        Hybrid_TranisitionMeanGTMHD{gg} = [Hybrid_TranisitionMeanGTMHD{gg};Performance{mm}.TranisitionMeanGTMHD];
    end
    
    CompiledTransitionCheckData = [];
    for mm=1:105
        
        if ~isempty(Performance{mm}.TransitionCheck)
            tempDiff = diff(Performance{mm}.TransitionCheck(:,6));
            ind = find(tempDiff>15);
            ind=[ind;length(Performance{mm}.TransitionCheck(:,6))];  %the most recent check for a gap

            CompiledTransitionCheckData = [CompiledTransitionCheckData;Performance{mm}.TransitionCheck(ind,:)];
        end
%         count = count + 1;   
    end
    
    ApproachToWaitIndices = find(CompiledTransitionCheckData(:,13)==1 & CompiledTransitionCheckData(:,18)==2);
    ApproachToCrossIndices = find(CompiledTransitionCheckData(:,13)==1 & CompiledTransitionCheckData(:,18)==3);
    WaitToWaitIndices = find(CompiledTransitionCheckData(:,13)==2 & CompiledTransitionCheckData(:,18)==2);
    WaitToCrossIndices = find(CompiledTransitionCheckData(:,13)==2 & CompiledTransitionCheckData(:,18)==3);

    [ApproachToWaitPerformance{gg}] = probabilisticPredictionPerformance(CompiledTransitionCheckData(ApproachToWaitIndices,16),CompiledTransitionCheckData(ApproachToWaitIndices,15));
    [ApproachToCrossPerformance{gg}] = probabilisticPredictionPerformance(CompiledTransitionCheckData(ApproachToCrossIndices,16),CompiledTransitionCheckData(ApproachToCrossIndices,15));
    [WaitToWaitPerformance{gg}] = probabilisticPredictionPerformance(CompiledTransitionCheckData(WaitToWaitIndices,16),CompiledTransitionCheckData(WaitToWaitIndices,15));
    [WaitToCrossIndicesPerformance{gg}] = probabilisticPredictionPerformance(CompiledTransitionCheckData(WaitToCrossIndices,16),CompiledTransitionCheckData(WaitToCrossIndices,15));

    
    for mm=1:105
        if ~isempty(Performance{mm}.WaitStart)
            ApproachToWaitTTE_Error{gg} = [ApproachToWaitTTE_Error{gg};[Performance{mm}.BestWaitPredInstance,Performance{mm}.BestWaitPredError]];
        end
        ApproachToCrossTTE_Error{gg} = [ApproachToCrossTTE_Error{gg};[Performance{mm}.BestApproachToCrossPredInstance,Performance{mm}.BestApproachToCrossPredError]];
        WaitToCrossTTE_Error{gg} = [WaitToCrossTTE_Error{gg};[Performance{mm}.BestWaitToCrossPredInstance,Performance{mm}.BestWaitToCrossPredError]];
        CrossToRetreatTTE_Error{gg} = [CrossToRetreatTTE_Error{gg};[Performance{mm}.BestRetreatPredInstance,Performance{mm}.BestRetreatPredError]];
    end
    

end

MeanGTRMSE_Hybrid = cellfun(@mean,Hybrid_MeanGTRMSE);
MeanGTAverageEuclideanError_Hybrid = cellfun(@mean,Hybrid_MeanGTAverageEuclideanError);
MeanGTEndEuclideanError_Hybrid = cellfun(@mean,Hybrid_MeanGTEndEuclideanError);
MeanGTMHD_Hybrid = cellfun(@mean,Hybrid_MeanGTMHD);

TranisitionMeanGTRMSE_Hybrid = cellfun(@mean,Hybrid_TranisitionMeanGTRMSE);
TranisitionMeanGTAverageEuclideanError_Hybrid = cellfun(@mean,Hybrid_TranisitionMeanGTAverageEuclideanError);
TranisitionGTEndEuclideanError_Hybrid = cellfun(@mean,Hybrid_TranisitionGTEndEuclideanError);
TranisitionMeanGTMHD_Hybrid = cellfun(@mean,Hybrid_TranisitionMeanGTMHD);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
























figure()
plot(pred_Horizon_matrix*0.1,MeanGTRMSE_Hybrid,'-*b','MarkerSize',10,'LineWidth',3);hold on;
plot(pred_Horizon_matrix*0.1,MeanGTRMSE_CV,'-*r','MarkerSize',10,'LineWidth',3);hold on;
legend('Hybrid','Constant velocity')
title('RMSE Error')
xlabel('Prediction Horizon [s]')
ylabel ('Position Error [m]')
set(gca,'fontsize', 38)

figure()
plot(pred_Horizon_matrix*0.1,MeanGTAverageEuclideanError_Hybrid,'-*b','MarkerSize',10,'LineWidth',3);hold on;
plot(pred_Horizon_matrix*0.1,MeanGTAverageEuclideanError_CV,'-*r','MarkerSize',10,'LineWidth',3);hold on;
legend('Hybrid','Constant velocity')
title('Average displacement error')
xlabel('Prediction Horizon [s]')
ylabel ('Position Error [m]')
set(gca,'fontsize', 38)

figure()
plot(pred_Horizon_matrix*0.1,MeanGTEndEuclideanError_Hybrid,'-*b','MarkerSize',10,'LineWidth',3);hold on;
plot(pred_Horizon_matrix*0.1,MeanGTEndEuclideanError_CV,'-*r','MarkerSize',10,'LineWidth',3);hold on;
legend('Hybrid','Constant velocity')
title('Final displacement error')
xlabel('Prediction Horizon [s]')
ylabel ('Position Error [m]')
set(gca,'fontsize', 38)

figure()
plot(pred_Horizon_matrix*0.1,MeanGTMHD_Hybrid,'-*b','MarkerSize',10,'LineWidth',3);hold on;
plot(pred_Horizon_matrix*0.1,MeanGTMHD_CV,'-*r','MarkerSize',10,'LineWidth',3);hold on;
legend('Hybrid','Constant velocity')
title('Modified Hausdorff Distance')
xlabel('Prediction Horizon [s]')
ylabel ('Position Error [m]')
set(gca,'fontsize', 38)

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
















