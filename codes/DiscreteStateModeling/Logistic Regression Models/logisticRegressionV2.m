%% logistic regression model

clear all

% inputs
Threshold = 0.5;    %classifier probability thershold
k = 5; %k-fold cross validation

%% Waiting to Crossing Model

% Read data
% %Test Data
% WaitToCrossModelData_Test = xlsread('GapWiseCompiledDataV2.xlsx');
% %Train Data
% WaitToCrossModelData_Train = xlsread('GapWiseCompiledDataV2.xlsx');

% load('SVMTrainData_ExtremeThreshold_CombinedExpectedGap_CorrectDTCurbDTCW_StartingGapTimesUpatedForCrossingGaps.mat');
% load('SVMTestData_ExtremeThreshold_CombinedExpectedGap_CorrectDTCurbDTCW_StartingGapTimesUpatedForCrossingGaps.mat');


% load('SVMTrainData_MildThreshold_CombinedExpectedGap_CorrectDTCurbDTCW_StartingGapTimesUpatedForCrossingGaps.mat');
% load('SVMTestData_MildThreshold_CombinedExpectedGap_CorrectDTCurbDTCW_StartingGapTimesUpatedForCrossingGaps.mat');

% 
% load('SVMTrainData_MildThreshold_CombinedExpectedGap_CorrectDTCurbDTCW_StartingGapTimesUpatedForCrossingGaps.mat');
% load('SVMTestData_MildThreshold_CombinedExpectedGap_CorrectDTCurbDTCW_StartingGapTimesUpatedForCrossingGaps.mat');


% 
load('SVMTrainData.mat');
load('SVMTestData.mat');


WaitToCrossModelData_Train = SVMTrainData;
WaitToCrossModelData_Train.SubjectID = ceil(WaitToCrossModelData_Train.CrossingNumber/18);


WaitToCrossModelData_Test = SVMTestData;
WaitToCrossModelData_Test.SubjectID = ceil(WaitToCrossModelData_Test.CrossingNumber/18);


ModelPerformance.Mean = nanmean(WaitToCrossModelData_Train{1:end,:},1);                                
ModelPerformance.StdDev = nanstd(WaitToCrossModelData_Train{1:end,:},1);                                 
 
% Normalized training data
WaitToCrossData_TrainNorm = normalize(WaitToCrossModelData_Train);
WaitToCrossData_TrainNorm.WCAllGapsDecision = WaitToCrossModelData_Train.WCAllGapsDecision;
WaitToCrossData_TrainNorm.SubjectID = WaitToCrossModelData_Train.SubjectID;


%  Normalized data  for CV test
WaitToCrossData_TestNorm = normalize(WaitToCrossModelData_Test);
WaitToCrossData_TestNorm.WCAllGapsDecision = WaitToCrossModelData_Test.WCAllGapsDecision;
WaitToCrossData_TestNorm.SubjectID = WaitToCrossModelData_Test.SubjectID;

glmeModel32 = fitglme(WaitToCrossData_TrainNorm,...
		'WCAllGapsDecision ~ 1 + ExpectedGap_SVM + Gaze_SVM + VelocityAverage_SVM + PedestrianDistancetoCurb_SVM + PedestrianDistancetoCW_SVM + PedestrianCumulativeWaitTime_SVM  + (1|SubjectID)',...
        'Distribution','Binomial','Link','logit','FitMethod','Laplace','DummyVarCoding','effects');   
    
glmeModel32 = fitglme(dataCorr,...
		'DecisionBeforeGapStart ~ 1 + DTCurb_dec + (1|SubjectID)',...
        'Distribution','Normal','Link','log','FitMethod','Laplace','DummyVarCoding','effects');   
    
WaitDecisionCVpred = predict(glmeModel32,WaitToCrossData_TestNorm);

[Performance,ActualTestOutput,PredictedTestOutput] = classifierPerformance(WaitToCrossData_TestNorm.WCAllGapsDecision,WaitDecisionCVpred,Threshold);    
    

%% create table data for training set
%  CrossFromWaitGapDecision = WaitToCrossModelData_Train(:,13);
%  CrossFromWaitGapDecision(CrossFromWaitGapDecision==1)=0;
%  CrossFromWaitGapDecision(CrossFromWaitGapDecision==2|CrossFromWaitGapDecision==3)=1;
%  
%  CumulativeWait = WaitToCrossModelData_Train(:,10);    
%  GapDuration = WaitToCrossModelData_Train(:,16);  
%  CrossDirection = WaitToCrossModelData_Train(:,19);
%  GazeRatioEntireDuration = WaitToCrossModelData_Train(:,20);
%  PedestrianSpeed = WaitToCrossModelData_Train(:,21);
%  MovingWindowGazeRatio = WaitToCrossModelData_Train(:,27);
%  PedestrianDistanceToCurb = WaitToCrossModelData_Train(:,39);
%  PedestrianDistanceToCW = WaitToCrossModelData_Train(:,45);
%  SameLaneVehicleDistanceToPedestrian = WaitToCrossModelData_Train(:,51);
%  SameLaneVehicleSpeed = WaitToCrossModelData_Train(:,57);
%  SameLaneVehicleAcceleration = WaitToCrossModelData_Train(:,63);
%  AdjacentLaneVehicleDistanceToPedestrian = WaitToCrossModelData_Train(:,69);
%  AdjacentLaneVehicleSpeed = WaitToCrossModelData_Train(:,75);
%  AdjacentLaneVehicleAcceleration = WaitToCrossModelData_Train(:,81);
%  SameLaneVehicleDistanceGap = WaitToCrossModelData_Train(:,87);
%  SameLaneVehicleTimeToCW = WaitToCrossModelData_Train(:,93);
%  SameLaneVehicleTimeToCollision = WaitToCrossModelData_Train(:,99);
%  SubjectID = WaitToCrossModelData_Train(:,1);
% 
%  
%  WaitToCrossData_TrainTable = table(SubjectID,CrossFromWaitGapDecision,CumulativeWait,GapDuration,...
%                                     CrossDirection,GazeRatioEntireDuration,PedestrianSpeed,MovingWindowGazeRatio,...
%                                     PedestrianDistanceToCurb,PedestrianDistanceToCW,SameLaneVehicleDistanceToPedestrian,...
%                                     SameLaneVehicleSpeed,SameLaneVehicleAcceleration,AdjacentLaneVehicleDistanceToPedestrian,...
%                                     AdjacentLaneVehicleSpeed,AdjacentLaneVehicleAcceleration,SameLaneVehicleDistanceGap,...
%                                     SameLaneVehicleTimeToCW,SameLaneVehicleTimeToCollision);
%  
% 
% 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Old Gapwise Data - before identifying the wait to cross, segregarting the start to cross and reached road

 CrossFromWaitGapDecision = WaitToCrossModelData_Train(:,4);

 
 CumulativeWait = WaitToCrossModelData_Train(:,9);    
 CrossDirection = WaitToCrossModelData_Train(:,13);
 GazeRatioEntireDuration = WaitToCrossModelData_Train(:,14);
 PedestrianSpeed = WaitToCrossModelData_Train(:,15);
 MovingWindowGazeRatio = WaitToCrossModelData_Train(:,21);
 PedestrianDistanceToCurb = WaitToCrossModelData_Train(:,33);
 PedestrianDistanceToCW = WaitToCrossModelData_Train(:,39);
 SameLaneVehicleDistanceToPedestrian = WaitToCrossModelData_Train(:,45);
 SameLaneVehicleSpeed = WaitToCrossModelData_Train(:,51);
 SameLaneVehicleAcceleration = WaitToCrossModelData_Train(:,57);
 AdjacentLaneVehicleDistanceToPedestrian = WaitToCrossModelData_Train(:,63);
 AdjacentLaneVehicleSpeed = WaitToCrossModelData_Train(:,69);
 AdjacentLaneVehicleAcceleration = WaitToCrossModelData_Train(:,75);
 SameLaneVehicleDistanceGap = WaitToCrossModelData_Train(:,81);
 SameLaneVehicleTimeToCW = WaitToCrossModelData_Train(:,87);
 SameLaneVehicleTimeToCollision = WaitToCrossModelData_Train(:,93);
 SubjectID = WaitToCrossModelData_Train(:,1);

 
 WaitToCrossData_TrainTable = table(SubjectID,CrossFromWaitGapDecision,CumulativeWait,...
                                    CrossDirection,GazeRatioEntireDuration,PedestrianSpeed,MovingWindowGazeRatio,...
                                    PedestrianDistanceToCurb,PedestrianDistanceToCW,SameLaneVehicleDistanceToPedestrian,...
                                    SameLaneVehicleSpeed,SameLaneVehicleAcceleration,AdjacentLaneVehicleDistanceToPedestrian,...
                                    AdjacentLaneVehicleSpeed,AdjacentLaneVehicleAcceleration,SameLaneVehicleDistanceGap,...
                                    SameLaneVehicleTimeToCW,SameLaneVehicleTimeToCollision);
 




indices.WaitGap = find(WaitToCrossModelData_Train(:,13)==1);
indices.CrossGap = find(WaitToCrossModelData_Train(:,13)==2 | WaitToCrossModelData_Train(:,13)==3);

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
     
%      WaitToCrossModelData_CVTrain = WaitToCrossModelData_Train([indices.WaitGapCVTrain;indices.CrossGapCVTrain],:);
%      WaitToCrossModelData_CVTest = WaitToCrossModelData_Train([indices.WaitGapCVTest;indices.CrossGapCVTest],:);
%  
%% create table data for CV training set
%WaitToCrossData_CVTrainTable = createTable(WaitToCrossModelData_CVTrain);
 
                           
%% create table data for validation set
% WaitToCrossData_CVTestTable = createTable(WaitToCrossModelData_CVTest);
                                             
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


glmeModel32{ii} = fitglme(WaitToCrossData_TrainTable,...
		'CrossFromWaitGapDecision ~ 1 + MovingWindowGazeRatio + PedestrianDistanceToCurb + SameLaneVehicleDistanceToPedestrian + SameLaneVehicleSpeed + SameLaneVehicleAcceleration +  (1|SubjectID)',...
        'Distribution','Binomial','Link','logit','FitMethod','Laplace','DummyVarCoding','effects');    

glmeModel32{ii} = fitglme(WaitToCrossData_TrainTable,...
		'CrossFromWaitGapDecision ~ 1 + MovingWindowGazeRatio + PedestrianDistanceToCurb + SameLaneVehicleDistanceToPedestrian + SameLaneVehicleSpeed + SameLaneVehicleAcceleration',...
        'Distribution','Binomial','Link','logit','FitMethod','Laplace','DummyVarCoding','effects');    
   




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
WaitToCrossData_TestTable = createTable(WaitToCrossModelData_Test);                                
                                                                                       
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
