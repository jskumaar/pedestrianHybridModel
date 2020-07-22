%% logistic regression model

clear all

% inputs
Threshold = 0.5;    %classifier probability threshold
k = 10; %k-fold cross validation

%% Waiting to Crossing Model

% Read data
%Test Data
WaitToCrossModelData_Test = xlsread('ApproachToCrossModelData_Test_65535_removed.xlsx');
%Train Data
WaitToCrossModelData_Train = xlsread('ApproachToCrossModelData_Train_65535_removed.xlsx');

%% create table data for training set
%  CrossFromWaitGapDecision = WaitToCrossModelData_Train(:,13);
%  CrossFromWaitGapDecision(CrossFromWaitGapDecision==1)=0;
%  CrossFromWaitGapDecision(CrossFromWaitGapDecision==2|CrossFromWaitGapDecision==3)=1;

% indices = [1:length(WaitToCrossModelData_Train)];

%indices = find(WaitToCrossModelData_Train(:,12)==1 | WaitToCrossModelData_Train(:,12)==2);

% indices = find(WaitToCrossModelData_Train(:,12)==1 | WaitToCrossModelData_Train(:,12)==3);
% 

indices = find(WaitToCrossModelData_Train(:,12)==2 | WaitToCrossModelData_Train(:,12)==3);

 ApproachToWaitCrossGapDecision = WaitToCrossModelData_Train(indices,12);
%  CrossFromWaitGapDecision(CrossFromWaitGapDecision==1)=0;
%  CrossFromWaitGapDecision(CrossFromWaitGapDecision==2|CrossFromWaitGapDecision==3)=1;

 
 CumulativeWait = WaitToCrossModelData_Train(indices,10);    
 GapDuration = WaitToCrossModelData_Train(indices,16);  
 CrossDirection = WaitToCrossModelData_Train(indices,19);
 GazeRatioEntireDuration = WaitToCrossModelData_Train(indices,20);
 PedestrianSpeed = WaitToCrossModelData_Train(indices,21);
 MovingWindowGazeRatio = WaitToCrossModelData_Train(indices,27);
 PedestrianDistanceToCurb = WaitToCrossModelData_Train(indices,39);
 PedestrianDistanceToCW = WaitToCrossModelData_Train(indices,45);
 SameLaneVehicleDistanceToPedestrian = WaitToCrossModelData_Train(indices,51);
 SameLaneVehicleSpeed = WaitToCrossModelData_Train(indices,57);
 SameLaneVehicleAcceleration = WaitToCrossModelData_Train(indices,63);
 AdjacentLaneVehicleDistanceToPedestrian = WaitToCrossModelData_Train(indices,69);
 AdjacentLaneVehicleSpeed = WaitToCrossModelData_Train(indices,75);
 AdjacentLaneVehicleAcceleration = WaitToCrossModelData_Train(indices,81);
 SameLaneVehicleDistanceGap = WaitToCrossModelData_Train(indices,87);
 SameLaneVehicleTimeToCW = WaitToCrossModelData_Train(indices,93);
 SameLaneVehicleTimeToCollision = WaitToCrossModelData_Train(indices,99);
 SubjectID = WaitToCrossModelData_Train(indices,1);

 
 WaitToCrossData_TrainTable = table(SubjectID,ApproachToWaitCrossGapDecision,CumulativeWait,GapDuration,...
                                    CrossDirection,GazeRatioEntireDuration,PedestrianSpeed,MovingWindowGazeRatio,...
                                    PedestrianDistanceToCurb,PedestrianDistanceToCW,SameLaneVehicleDistanceToPedestrian,...
                                    SameLaneVehicleSpeed,SameLaneVehicleAcceleration,AdjacentLaneVehicleDistanceToPedestrian,...
                                    AdjacentLaneVehicleSpeed,AdjacentLaneVehicleAcceleration,SameLaneVehicleDistanceGap,...
                                    SameLaneVehicleTimeToCW,SameLaneVehicleTimeToCollision);
 



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
indices.WaitGap = find(WaitToCrossModelData_Train(:,13)==1);
indices.CrossGap = find(WaitToCrossModelData_Train(:,13)==2 | WaitToCrossModelData_Train(:,13)==3);

N = length(indices.WaitGap);
M = length(indices.CrossGap);


%randomize indices order
temp = randperm(N,N);
indices.WaitGap = indices.WaitGap(temp);

temp = randperm(M,M);
indices.CrossGap = indices.CrossGap(temp);

%intialize
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
 
%% create table data for training set
 CrossFromWaitGapDecision = WaitToCrossModelData_CVTrain(:,13);
 CrossFromWaitGapDecision(CrossFromWaitGapDecision==1)=0;
 CrossFromWaitGapDecision(CrossFromWaitGapDecision==2|CrossFromWaitGapDecision==3)=1;
 
 CumulativeWait = WaitToCrossModelData_CVTrain(:,10);    
 GapDuration = WaitToCrossModelData_CVTrain(:,16);  
 CrossDirection = WaitToCrossModelData_CVTrain(:,19);
 GazeRatioEntireDuration = WaitToCrossModelData_CVTrain(:,20);
 PedestrianSpeed = WaitToCrossModelData_CVTrain(:,21);
 MovingWindowGazeRatio = WaitToCrossModelData_CVTrain(:,27);
 PedestrianDistanceToCurb = WaitToCrossModelData_CVTrain(:,39);
 PedestrianDistanceToCW = WaitToCrossModelData_CVTrain(:,45);
 SameLaneVehicleDistanceToPedestrian = WaitToCrossModelData_CVTrain(:,51);
 SameLaneVehicleSpeed = WaitToCrossModelData_CVTrain(:,57);
 SameLaneVehicleAcceleration = WaitToCrossModelData_CVTrain(:,63);
 AdjacentLaneVehicleDistanceToPedestrian = WaitToCrossModelData_CVTrain(:,69);
 AdjacentLaneVehicleSpeed = WaitToCrossModelData_CVTrain(:,75);
 AdjacentLaneVehicleAcceleration = WaitToCrossModelData_CVTrain(:,81);
 SameLaneVehicleDistanceGap = WaitToCrossModelData_CVTrain(:,87);
 SameLaneVehicleTimeToCW = WaitToCrossModelData_CVTrain(:,93);
 SameLaneVehicleTimeToCollision = WaitToCrossModelData_CVTrain(:,99);
 SubjectID = WaitToCrossModelData_CVTrain(:,1);

 
 WaitToCrossData_CVTrainTable = table(SubjectID,CrossFromWaitGapDecision,CumulativeWait,GapDuration,...
                                    CrossDirection,GazeRatioEntireDuration,PedestrianSpeed,MovingWindowGazeRatio,...
                                    PedestrianDistanceToCurb,PedestrianDistanceToCW,SameLaneVehicleDistanceToPedestrian,...
                                    SameLaneVehicleSpeed,SameLaneVehicleAcceleration,AdjacentLaneVehicleDistanceToPedestrian,...
                                    AdjacentLaneVehicleSpeed,AdjacentLaneVehicleAcceleration,SameLaneVehicleDistanceGap,...
                                    SameLaneVehicleDistanceGap,SameLaneVehicleTimeToCW,SameLaneVehicleTimeToCollision);
 
 %% create table data for validation set
 
 CrossFromWaitGapDecision = WaitToCrossModelData_CVTest(:,13);
 CrossFromWaitGapDecision(CrossFromWaitGapDecision==1)=0;
 CrossFromWaitGapDecision(CrossFromWaitGapDecision==2|CrossFromWaitGapDecision==3)=1;
 
 CumulativeWait = WaitToCrossModelData_CVTest(:,10);    
 GapDuration = WaitToCrossModelData_CVTest(:,16);  
 CrossDirection = WaitToCrossModelData_CVTest(:,19);
 GazeRatioEntireDuration = WaitToCrossModelData_CVTest(:,20);
 PedestrianSpeed = WaitToCrossModelData_CVTest(:,21);
 MovingWindowGazeRatio = WaitToCrossModelData_CVTest(:,27);
 PedestrianDistanceToCurb = WaitToCrossModelData_CVTest(:,39);
 PedestrianDistanceToCW = WaitToCrossModelData_CVTest(:,45);
 SameLaneVehicleDistanceToPedestrian = WaitToCrossModelData_CVTest(:,51);
 SameLaneVehicleSpeed = WaitToCrossModelData_CVTest(:,57);
 SameLaneVehicleAcceleration = WaitToCrossModelData_CVTest(:,63);
 AdjacentLaneVehicleDistanceToPedestrian = WaitToCrossModelData_CVTest(:,69);
 AdjacentLaneVehicleSpeed = WaitToCrossModelData_CVTest(:,75);
 AdjacentLaneVehicleAcceleration = WaitToCrossModelData_CVTest(:,81);
 SameLaneVehicleDistanceGap = WaitToCrossModelData_CVTest(:,87);
 SameLaneVehicleTimeToCW = WaitToCrossModelData_CVTest(:,93);
 SameLaneVehicleTimeToCollision = WaitToCrossModelData_CVTest(:,99);
 SubjectID = WaitToCrossModelData_CVTest(:,1);
 
 WaitToCrossData_CVTestTable = table(SubjectID,CrossFromWaitGapDecision,CumulativeWait,GapDuration,...
                                    CrossDirection,GazeRatioEntireDuration,PedestrianSpeed,MovingWindowGazeRatio,...
                                    PedestrianDistanceToCurb,PedestrianDistanceToCW,SameLaneVehicleDistanceToPedestrian,...
                                    SameLaneVehicleSpeed,SameLaneVehicleAcceleration,AdjacentLaneVehicleDistanceToPedestrian,...
                                    AdjacentLaneVehicleSpeed,AdjacentLaneVehicleAcceleration,SameLaneVehicleDistanceGap,...
                                    SameLaneVehicleDistanceGap,SameLaneVehicleTimeToCW,SameLaneVehicleTimeToCollision);
 

%% fit GLME (Mixed-Logistic Regression Model)
%% Model I - All input variablles

% glme = fitglme(WaitToCrossData_CVTrainTable,...
% 		'CrossFromWaitGapDecision ~ 1 + CumulativeWait + GapDuration + CrossDirection + GazeRatioEntireDuration + PedestrianSpeed + MovingWindowGazeRatio + PedestrianDistanceToCurb + PedestrianDistanceToCW + SameLaneVehicleDistanceToPedestrian + SameLaneVehicleSpeed + SameLaneVehicleAcceleration + AdjacentLaneVehicleDistanceToPedestrian + AdjacentLaneVehicleSpeed + AdjacentLaneVehicleAcceleration + SameLaneVehicleDistanceGap + SameLaneVehicleTimeToCW + SameLaneVehicleTimeToCollision + (1|SubjectID)',...
%         'Distribution','Binomial','Link','logit','FitMethod','Laplace','DummyVarCoding','effects');

%% Individual sub models
% glmeModel{ii} = fitglme(WaitToCrossData_CVTrainTable,...
% 		'CrossFromWaitGapDecision ~ 1 + CumulativeWait + (1|SubjectID)',...
%         'Distribution','Binomial','Link','logit','FitMethod','Laplace','DummyVarCoding','effects');

% glmeModel10{ii} = fitglme(WaitToCrossData_CVTrainTable,...
% 		'CrossFromWaitGapDecision ~ 1 + CumulativeWait + GapDuration + (1|SubjectID)',...
%         'Distribution','Binomial','Link','logit','FitMethod','Laplace','DummyVarCoding','effects');

% glmeModel10{ii} = fitglme(WaitToCrossData_CVTrainTable,...
% 		'CrossFromWaitGapDecision ~ 1 + CumulativeWait + GapDuration + GazeRatioEntireDuration + PedestrianDistanceToCurb + (1|SubjectID)',...
%         'Distribution','Binomial','Link','logit','FitMethod','Laplace','DummyVarCoding','effects');

% 

% glmeModel10{ii} = fitglme(WaitToCrossData_CVTrainTable,...
% 		'CrossFromWaitGapDecision ~ 1 + CumulativeWait + GapDuration + GazeRatioEntireDuration + PedestrianDistanceToCurb + SameLaneVehicleDistanceToPedestrian + (1|SubjectID)',...
%         'Distribution','Binomial','Link','logit','FitMethod','Laplace','DummyVarCoding','effects');
%     
    
% glmeModel10{ii} = fitglme(WaitToCrossData_CVTrainTable,...
% 		'CrossFromWaitGapDecision ~ 1 + CumulativeWait + GapDuration + GazeRatioEntireDuration + PedestrianDistanceToCurb + SameLaneVehicleDistanceToPedestrian + SameLaneVehicleSpeed + (1|SubjectID)',...
%         'Distribution','Binomial','Link','logit','FitMethod','Laplace','DummyVarCoding','effects');

% glmeModel10{ii} = fitglme(WaitToCrossData_CVTrainTable,...
% 		'CrossFromWaitGapDecision ~ 1 + CumulativeWait + GapDuration + GazeRatioEntireDuration + PedestrianDistanceToCurb + SameLaneVehicleDistanceToPedestrian + AdjacentLaneVehicleDistanceToPedestrian + (1|SubjectID)',...
%         'Distribution','Binomial','Link','logit','FitMethod','Laplace','DummyVarCoding','effects');

% glmeModel10{ii} = fitglme(WaitToCrossData_CVTrainTable,...
% 		'CrossFromWaitGapDecision ~ 1 + CumulativeWait + GapDuration + GazeRatioEntireDuration + PedestrianDistanceToCurb + SameLaneVehicleDistanceToPedestrian + AdjacentLaneVehicleDistanceToPedestrian + AdjacentLaneVehicleSpeed + (1|SubjectID)',...
%         'Distribution','Binomial','Link','logit','FitMethod','Laplace','DummyVarCoding','effects');

% glmeModel10{ii} = fitglme(WaitToCrossData_CVTrainTable,...
% 		'CrossFromWaitGapDecision ~ 1 + CumulativeWait + GapDuration + GazeRatioEntireDuration + PedestrianDistanceToCurb + SameLaneVehicleDistanceToPedestrian + AdjacentLaneVehicleDistanceToPedestrian + AdjacentLaneVehicleSpeed + SameLaneVehicleDistanceGap + (1|SubjectID)',...
%         'Distribution','Binomial','Link','logit','FitMethod','Laplace','DummyVarCoding','effects');
% 
% glmeModel10{ii} = fitglme(WaitToCrossData_CVTrainTable,...
% 		'CrossFromWaitGapDecision ~ 1 + CumulativeWait + GapDuration + GazeRatioEntireDuration + PedestrianDistanceToCurb + SameLaneVehicleDistanceToPedestrian +  SameLaneVehicleDistanceGap + SameLaneVehicleTimeToCollision + (1|SubjectID)',...
%         'Distribution','Binomial','Link','logit','FitMethod','Laplace','DummyVarCoding','effects');    

glmeModel10{ii} = fitglme(WaitToCrossData_CVTrainTable,...
		'CrossFromWaitGapDecision ~ 1 + CumulativeWait + GapDuration + GazeRatioEntireDuration + PedestrianDistanceToCurb + SameLaneVehicleDistanceToPedestrian +  SameLaneVehicleSpeed + (1|SubjectID)',...
        'Distribution','Binomial','Link','logit','FitMethod','Laplace','DummyVarCoding','effects');    

     


%% Logistic Regression classifer using ML app





    
%% Check model parameters
disp(glmeModel10{ii})

%% Check model performance
WaitDecisionCVpred(ii,:) = predict(glmeModel10{ii},WaitToCrossData_CVTestTable);

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
[~,ModelIndex(1)] = max(CVF1Score);
[~,ModelIndex(2)] = min(CVF1Score);

%% Testing performance on the test set

%Recategorize classes into 2, 0 - waiting, 1- crossing
CrossFromWaitGapDecision = WaitToCrossModelData_Test(:,12);
CrossFromWaitGapDecision(CrossFromWaitGapDecision==1)=0;
CrossFromWaitGapDecision(CrossFromWaitGapDecision==2|CrossFromWaitGapDecision==3)=1;
WaitToCrossData_TestTable.CrossFromWaitGapDecision = CrossFromWaitGapDecision;

 CumulativeWait = WaitToCrossModelData_Test(:,10);    
 GapDuration = WaitToCrossModelData_Test(:,16);  
 CrossDirection = WaitToCrossModelData_Test(:,19);
 GazeRatioEntireDuration = WaitToCrossModelData_Test(:,20);
 PedestrianSpeed = WaitToCrossModelData_Test(:,21);
 MovingWindowGazeRatio = WaitToCrossModelData_Test(:,27);
 PedestrianDistanceToCurb = WaitToCrossModelData_Test(:,39);
 PedestrianDistanceToCW = WaitToCrossModelData_Test(:,44);
 SameLaneVehicleDistanceToPedestrian = WaitToCrossModelData_Test(:,51);
 SameLaneVehicleSpeed = WaitToCrossModelData_Test(:,57);
 SameLaneVehicleAcceleration = WaitToCrossModelData_Test(:,63);
 AdjacentLaneVehicleDistanceToPedestrian = WaitToCrossModelData_Test(:,69);
 AdjacentLaneVehicleSpeed = WaitToCrossModelData_Test(:,75);
 AdjacentLaneVehicleAcceleration = WaitToCrossModelData_Test(:,81);
 SameLaneVehicleDistanceGap = WaitToCrossModelData_Test(:,87);
 SameLaneVehicleTimeToCW = WaitToCrossModelData_Test(:,93);
 SameLaneVehicleTimeToCollision = WaitToCrossModelData_Test(:,99);
 SubjectID = WaitToCrossModelData_Test(:,1);
 
 WaitToCrossData_TestTable = table(SubjectID,CrossFromWaitGapDecision,CumulativeWait,GapDuration,...
                                    CrossDirection,GazeRatioEntireDuration,PedestrianSpeed,MovingWindowGazeRatio,...
                                    PedestrianDistanceToCurb,PedestrianDistanceToCW,SameLaneVehicleDistanceToPedestrian,...
                                    SameLaneVehicleSpeed,SameLaneVehicleAcceleration,AdjacentLaneVehicleDistanceToPedestrian,...
                                    AdjacentLaneVehicleSpeed,AdjacentLaneVehicleAcceleration,SameLaneVehicleDistanceGap,...
                                    SameLaneVehicleDistanceGap,SameLaneVehicleTimeToCW,SameLaneVehicleTimeToCollision);
 
for ii=1:2
    WaitDecisionPred(ii,:) = predict(glmeModel10{ModelIndex(ii)},WaitToCrossData_TestTable);
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

save('glmeModel10.mat','ModelPerformance','ModelCVPerformance','glmeModel10');
