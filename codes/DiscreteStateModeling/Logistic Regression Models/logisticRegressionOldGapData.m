%% logistic regression model

clear all

% inputs
Threshold = 0.5;    %classifier probability thershold
k = 5; %k-fold cross validation

%% Waiting to Crossing Model

% Read data
%Test Data
WaitToCrossModelData_Test = xlsread('GapWiseCompiledDataV2.xlsx');
%Train Data
WaitToCrossModelData_Train = xlsread('GapWiseCompiledDataV2.xlsx');


indices.WaitGap = find(WaitToCrossModelData_Train(:,4)==0);
indices.CrossGap = find(WaitToCrossModelData_Train(:,4)==1);

N = length(indices.WaitGap);
M = length(indices.CrossGap);

%randomize indices order
temp = randperm(N,N);
indices.WaitGap = indices.WaitGap(temp);

temp = randperm(M,M);
indices.CrossGap = indices.CrossGap(temp);

%intialize cross validation performance matrix
ModelCVPerformance.ConfusionMatrix = [];
ModelCVPerformance.Accuracy = [];
ModelCVPerformance.Precision = [];
ModelCVPerformance.Recall = [];
ModelCVPerformance.F1Score = [];

for ii=1:k
       
    if ii==1
        indices.WaitGapCVTrain = indices.WaitGap(fix(N/k)+1:end);
        indices.CrossGapCVTrain = indices.CrossGap(fix(M/k)+1:end);
        indices.WaitGapCVTest = indices.WaitGap(1:fix(N/k));
        indices.CrossGapCVTest = indices.CrossGap(1:fix(M/k));
    else      
        indices.WaitGapCVTrain = indices.WaitGap([1:fix(N/k)*(ii-1),fix(N/k)*(ii)+1:end]);
        indices.CrossGapCVTrain = indices.CrossGap([1:fix(M/k)*(ii-1),fix(M/k)*(ii)+1:end]);
        indices.WaitGapCVTest = indices.WaitGap(fix(N/k)*(ii-1)+1:fix(N/k)*(ii));
        indices.CrossGapCVTest = indices.CrossGap(fix(M/k)*(ii-1)+1:fix(M/k)*(ii));       
    end
     
     WaitToCrossModelData_CVTrain = WaitToCrossModelData_Train([indices.WaitGapCVTrain;indices.CrossGapCVTrain],:);
     WaitToCrossModelData_CVTest = WaitToCrossModelData_Train([indices.WaitGapCVTest;indices.CrossGapCVTest],:);
 
%% create table data for CV training set
WaitToCrossData_CVTrainTable = createTableOldGapData(WaitToCrossModelData_CVTrain);
 
                           
%% create table data for validation set
 WaitToCrossData_CVTestTable = createTableOldGapData(WaitToCrossModelData_CVTest);

 
%% fit GLME (Mixed-Logistic Regression Model)
%% Model I - All input variablles

% glme = fitglme(WaitToCrossData_CVTrainTable,...
% 		'CrossFromWaitGapDecision ~ 1 + CumulativeWait + GapDuration + CrossDirection + GazeRatioEntireDuration + PedestrianSpeed + MovingWindowGazeRatio + PedestrianDistanceToCurb + PedestrianDistanceToCW + SameLaneVehicleDistanceToPedestrian + SameLaneVehicleSpeed + SameLaneVehicleAcceleration + AdjacentLaneVehicleDistanceToPedestrian + AdjacentLaneVehicleSpeed + AdjacentLaneVehicleAcceleration + SameLaneVehicleDistanceGap + SameLaneVehicleTimeToCW + SameLaneVehicleTimeToCollision + (1|SubjectID)',...
%         'Distribution','Binomial','Link','logit','FitMethod','Laplace','DummyVarCoding','effects');

%% Individual sub models
% glmeModel{ii} = fitglme(WaitToCrossData_CVTrainTable,...
% 		'CrossFromWaitGapDecision ~ 1 + CumulativeWait + (1|SubjectID)',...
%         'Distribution','Binomial','Link','logit','FitMethod','Laplace','DummyVarCoding','effects');

% glmeModel32{ii} = fitglme(WaitToCrossData_CVTrainTable,...
% 		'CrossFromWaitGapDecision ~ 1 + CumulativeWait + GapDuration + (1|SubjectID)',...
%         'Distribution','Binomial','Link','logit','FitMethod','Laplace','DummyVarCoding','effects');

% glmeModel32{ii} = fitglme(WaitToCrossData_CVTrainTable,...
% 		'CrossFromWaitGapDecision ~ 1 + CumulativeWait + GapDuration + GazeRatioEntireDuration + PedestrianDistanceToCurb + (1|SubjectID)',...
%         'Distribution','Binomial','Link','logit','FitMethod','Laplace','DummyVarCoding','effects');
% 

% glmeModel32{ii} = fitglme(WaitToCrossData_CVTrainTable,...
% 		'CrossFromWaitGapDecision ~ 1 + CumulativeWait + GapDuration + GazeRatioEntireDuration + MovingWindowGazeRatio + PedestrianDistanceToCurb + (1|SubjectID)',...
%         'Distribution','Binomial','Link','logit','FitMethod','Laplace','DummyVarCoding','effects');

% glmeModel32{ii} = fitglme(WaitToCrossData_CVTrainTable,...
% 		'CrossFromWaitGapDecision ~ 1 + CumulativeWait + GapDuration + GazeRatioEntireDuration + MovingWindowGazeRatio + PedestrianDistanceToCurb + SameLaneVehicleDistanceToPedestrian + (1|SubjectID)',...
%         'Distribution','Binomial','Link','logit','FitMethod','Laplace','DummyVarCoding','effects');
%     
    
% glmeModel32{ii} = fitglme(WaitToCrossData_CVTrainTable,...
% 		'CrossFromWaitGapDecision ~ 1 + CumulativeWait + GapDuration + GazeRatioEntireDuration + MovingWindowGazeRatio + PedestrianDistanceToCurb + SameLaneVehicleDistanceToPedestrian + SameLaneVehicleSpeed + (1|SubjectID)',...
%         'Distribution','Binomial','Link','logit','FitMethod','Laplace','DummyVarCoding','effects');

% glmeModel32{ii} = fitglme(WaitToCrossData_CVTrainTable,...
% 		'CrossFromWaitGapDecision ~ 1 + CumulativeWait + GapDuration + GazeRatioEntireDuration + MovingWindowGazeRatio + PedestrianDistanceToCurb + SameLaneVehicleDistanceToPedestrian + SameLaneVehicleSpeed + SameLaneVehicleAcceleration + (1|SubjectID)',...
%         'Distribution','Binomial','Link','logit','FitMethod','Laplace','DummyVarCoding','effects');


% glmeModel32{ii} = fitglme(WaitToCrossData_CVTrainTable,...
% 		'CrossFromWaitGapDecision ~ 1 + CumulativeWait + GapDuration + GazeRatioEntireDuration + MovingWindowGazeRatio + PedestrianDistanceToCurb + SameLaneVehicleDistanceToPedestrian + SameLaneVehicleSpeed +  SameLaneVehicleAcceleration + AdjacentLaneVehicleDistanceToPedestrian + (1|SubjectID)',...
%         'Distribution','Binomial','Link','logit','FitMethod','Laplace','DummyVarCoding','effects');

% glmeModel32{ii} = fitglme(WaitToCrossData_CVTrainTable,...
% 		'CrossFromWaitGapDecision ~ 1 + CumulativeWait + GapDuration + GazeRatioEntireDuration + MovingWindowGazeRatio + PedestrianDistanceToCurb + SameLaneVehicleDistanceToPedestrian + SameLaneVehicleSpeed + SameLaneVehicleAcceleration + AdjacentLaneVehicleDistanceToPedestrian + SameLaneVehicleDistanceGap + (1|SubjectID)',...
%         'Distribution','Binomial','Link','logit','FitMethod','Laplace','DummyVarCoding','effects');
% 
% glmeModel32{ii} = fitglme(WaitToCrossData_CVTrainTable,...
% 		'CrossFromWaitGapDecision ~ 1 + CumulativeWait + GapDuration + GazeRatioEntireDuration + MovingWindowGazeRatio + PedestrianDistanceToCurb + SameLaneVehicleDistanceToPedestrian + SameLaneVehicleSpeed + SameLaneVehicleAcceleration + AdjacentLaneVehicleDistanceToPedestrian + SameLaneVehicleDistanceGap + SameLaneVehicleTimeToCollision + (1|SubjectID)',...
%         'Distribution','Binomial','Link','logit','FitMethod','Laplace','DummyVarCoding','effects');    

%% only independent

% glmeModel32{ii} = fitglme(WaitToCrossData_CVTrainTable,...
% 		'CrossFromWaitGapDecision ~ 1 + CumulativeWait + GapDuration + MovingWindowGazeRatio + PedestrianDistanceToCurb + SameLaneVehicleDistanceToPedestrian + SameLaneVehicleSpeed + SameLaneVehicleAcceleration +  SameLaneVehicleDistanceGap + (1|SubjectID)',...
%         'Distribution','Binomial','Link','logit','FitMethod','Laplace','DummyVarCoding','effects');    


glmeModel32{ii} = fitglme(WaitToCrossData_CVTrainTable,...
		'CrossFromWaitGapDecision ~ 1 + GapDuration +  PedestrianDistanceToCurb + SameLaneVehicleDistanceToPedestrian + SameLaneVehicleSpeed + SameLaneVehicleAcceleration +  (1|SubjectID)',...
        'Distribution','Binomial','Link','logit','FitMethod','Laplace','DummyVarCoding','effects');    





% glmeModel32{ii} = fitglme(WaitToCrossData_CVTrainTable,...
% 		'CrossFromWaitGapDecision ~ 1 + CumulativeWait + GapDuration + MovingWindowGazeRatio + PedestrianDistanceToCurb + SameLaneVehicleDistanceToPedestrian + SameLaneVehicleSpeed + SameLaneVehicleAcceleration +  SameLaneVehicleDistanceGap + (1|SubjectID)',...
%         'Distribution','Binomial','Link','logit','FitMethod','Laplace','DummyVarCoding','effects');    
% 



%% Check model parameters
disp(glmeModel32{ii})

%% Check model performance
WaitDecisionCVpred(ii,:) = predict(glmeModel32{ii},WaitToCrossData_CVTestTable);

[Performance,ActualTestOutput,PredictedTestOutput] = classifierPerformance(WaitToCrossData_CVTestTable.CrossFromWaitGapDecision,WaitDecisionCVpred(ii,:),Threshold);

ModelCVPerformance = [ModelCVPerformance,Performance];

end
%% average cross-validation performance

ModelCVPerformance(1)=[];
CVAccuracy = [];
CVPrecision =[];
CVRecall=[];
CVF1Score=[];

for ii=1:k
   CVAccuracy=[CVAccuracy,ModelCVPerformance(ii).Accuracy];
   CVPrecision=[CVPrecision,ModelCVPerformance(ii).Precision];
   CVRecall=[CVRecall,ModelCVPerformance(ii).Recall];
   CVF1Score=[CVF1Score,ModelCVPerformance(ii).F1Score];  
end

ModelPerformance.CVAccuracy = mean(CVAccuracy);
ModelPerformance.CVPrecision = mean(CVPrecision);
ModelPerformance.CVRecall = mean(CVRecall);
ModelPerformance.CVF1Score = mean(CVF1Score);

%% Choose the best and worst performing CV model based on F1 score
% [~,ModelIndex(1)] = max(CVF1Score);
% [~,ModelIndex(2)] = min(CVF1Score);

%% Choose the best and worst performing CV model based on Recall; i.e. less error in missing crossing pedestrians
[~,ModelIndex(1)] = max(CVRecall);
[~,ModelIndex(2)] = min(CVRecall);


%% Testing performance on the test set
WaitToCrossData_TestTable = createTableOldGapData(WaitToCrossModelData_Test);                                
                                                                                       
for ii=1:2
    WaitDecisionPred(ii,:) = predict(glmeModel32{ModelIndex(ii)},WaitToCrossData_TestTable);
    [Performance,ActualTestOutput,PredictedTestOutput] = classifierPerformance(WaitToCrossData_TestTable.CrossFromWaitGapDecision,WaitDecisionPred(ii,:),Threshold);

    if ii==1
        ModelPerformance.BestAccuracy = Performance.Accuracy;
        ModelPerformance.BestPrecision = Performance.Precision; 
        ModelPerformance.BestRecall = Performance.Recall; 
        ModelPerformance.BestF1Score = Performance.F1Score; 
    else
        ModelPerformance.WorstAccuracy = Performance.Accuracy;
        ModelPerformance.WorstPrecision = Performance.Precision; 
        ModelPerformance.WorstRecall = Performance.Recall; 
        ModelPerformance.WorstF1Score = Performance.F1Score;
    end
end

save('glmeModel32.mat','ModelPerformance','ModelCVPerformance','glmeModel32');
