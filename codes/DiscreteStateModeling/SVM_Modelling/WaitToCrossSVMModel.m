%% SVM

clear all
close all
% 
% 
% Read data
% vehicleGapTimes = xlsread('VehicleGapTimesV6.xlsx');
% GapData = xlsread('GapWiseCompiledDataV6.xlsx');
% EventIndices = xlsread('DiscreteStateEventIndicesW5.xlsx');


% load model
% load('SVMTrainData_ExtremeOutlier_StartGapExpectedGap_CorrectDTCurbDTCW.mat')
% Dependent variable = ;

% Independent variables
% 1) WCExpectedGapStartGap
% 2) PedestrianAbsoluteVelocityAverage
% 3) PedestrianDistancetoCW
% 4) PedestrianDistancetoCurb
% 5) PedestrianCumulativeWaitTime
% 6) GazeRatioGapStart



%% Step 1: Build the model using the classification learner app
%a) Load the SVM training data
load('SVMTrain_woHighDec_extreme_10_28_19.mat');

%b) Using the Classifier app, train and save the model

%% Step 2: Convert SVM model to probabilistic model
% a) load the saved model if not already available in the workspace
% load("SVM_ExtremeOutlier_NoHighDeceleration_6Features_noGap_noGaze.mat") 

% 
% SVM model with probabilities
Prob_ExtremeOutlier_NoGap_NoGaze_CubicSVM_TrainedModel = fitSVMPosterior(SVM_ExtremeOutlier_NoHighDeceleration_6Features_noGap_noGaze.ClassificationSVM);

CumulativeWaitTime = 0;
PedestrianVelocity = 0;
DTCurb = 0;
DTCW = 0;
VehicleDistancetoPed = 0;
VehicleSpeed = 0;

features = [CumulativeWaitTime, PedestrianVelocity, DTCurb, DTCW, ...
            VehicleDistancetoPed, VehicleSpeed];
    

[predicted_decision,predicted_score] = predict(Prob_ExtremeOutlier_NoGap_NoGaze_CubicSVM_TrainedModel,features);  


% %% Step 3: check model predictions of saved probabilistic model
% % a) load the saved model if not already available in the workspace
% 
% 
% % b) load the testing data
% load('SVMTestData_ExtremeOutlier_StartGapExpectedGap_CorrectDTCurbDTCW.mat')
% 
% % make sure the order matches the order in the SVM model!
% features = [SVMTestData.WCExpectedGapStartGap,SVMTestData.PedestrianAbsoluteVelocityAverage, SVMTestData.PedestrianDistancetoCW,...
%             SVMTestData.PedestrianDistancetoCurb, SVMTestData.PedestrianCumulativeWaitTime];
% 
% % features = [SVMTrainData.WCExpectedGapStartGap,SVMTrainData.PedestrianAbsoluteVelocityAverage, SVMTrainData.PedestrianDistancetoCW,...
% %             SVMTrainData.PedestrianDistancetoCurb, SVMTrainData.PedestrianCumulativeWaitTime];
% 
%         
% actual_label = SVMTestData.WCAllGapsDecision_CrossDecisionOnRoadGap;
% 
% [predicted_decision,predicted_score] = predict(ExtremeOutlier_StartGap_NoGaze_CubicSVM_TrainedModel.ClassificationSVM,features);            
% 
% 
% [Performance,~,~] = classifierPerformance(actual_label,predicted_decision,0.5);






%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Irrelevant Old codes

% %% Time gap - Dependent variable
% ExpectedGap = GapData(:,105);
% ExpectedGapAcc = GapData(:,106);
% ExpectedGapStartGap = GapData(:,107);
% ExpectedGapStartGapAcc = GapData(:,108);
% 
% %% Gaze Data - Dependent variable
% % GapData = GapData(indData,:);
% % vehicleGapTimes = vehicleGapTimes(indData,:);
% 
% 
% GazeRatiosBeforeCrossing = GapData(:,109:113);
% GazeRatiosGapStart = GapData(:,114:118);
% 
% 
% %% inputs
% 
% waitThreshold=300;
% Nk=5;        %cross-validation fold
% 
% out=1;              % 1 - include wait time outliers for crossing indices
% binsize = 0.5;
% Gazebinsize = 0.2;
% 
% % GapType = ExpectedGap;
% GapType = ExpectedGapAcc;
% % GapType = ExpectedGapStartGap;
% % GapType = ExpectedGapStartGapAcc;
% 
% gazeInd = 1;        %1-10, 2-15, 3-20. 4-25, 5-30, gaze windows
% % GazeType = GazeRatiosBeforeCrossing;
%  GazeType = GazeRatiosGapStart;
%  
%  %indices of vehicle gaps for each crossing
% ind = find(diff(vehicleGapTimes(:,3))~=0);
% indStart = [1;ind+1];
% indEnd = [ind;length(vehicleGapTimes)];
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% %% Wait to Cross: find vehicle gaps indices of interest
% 
% % crossing and approach indices; do not count for gap rejection
% indices.Gap_Approach = find(GapData(:,11)==1);
% indices.Gap_Cross = find(GapData(:,11)==3);
% 
% 
% %use gap acceptance data for only crossings with nominal waiting times;
% %remove the outliers data! These indices are crossing indices
% indices.Cross_WaitActivityOutliers = find((EventIndices(:,7)-EventIndices(:,6))>waitThreshold);
% 
% % vehicle gaps indices corresponding to the waiting outliers
% indices.Gap_ToRemoveWaitTimeOutliers =[];
% for ii=1:length(indices.Cross_WaitActivityOutliers)
%      indices.Gap_ToRemoveWaitTimeOutliers = [indices.Gap_ToRemoveWaitTimeOutliers;[indStart(indices.Cross_WaitActivityOutliers(ii)):indEnd(indices.Cross_WaitActivityOutliers(ii))]'];
% end
% 
% %unique vehicle gap indices for rejected gaps
% indices.Gap_uniqueRemoveRejectedGap = unique([indices.Gap_Approach;indices.Gap_ToRemoveWaitTimeOutliers;indices.Gap_Cross]);
% % indices.Gap_uniqueRemoveRejectedGap = unique([indices.Gap_Approach;indices.Gap_Cross]);
% 
% %% Rejected Gap indices
% indices.Gap_RejectedGapIndices = [1:length(GapData)]';
% indices.Gap_RejectedGapIndices(indices.Gap_uniqueRemoveRejectedGap) = [];
% 
% %crossing indices within wait time threshold
% indices.Cross_WaitActivityNotOutliers = find((EventIndices(:,7)-EventIndices(:,6))<=waitThreshold);
% 
% %% Accepted Gaps without outliers
% indices.Gap_AcceptedGapIndices = indices.Gap_Cross(indices.Cross_WaitActivityNotOutliers);
% 
% %% Accepted gaps while waiting
% indices.Gap_AcceptedGapWhileWaitIndices = find(GapData(:,11)==3 & (GapData(:,10)~=0));
% indices.Gap_AcceptedGapsWhileApproachIndices = find(GapData(:,11)==3 & (GapData(:,10)==0));
% 
% [~,ind,~] = intersect(indices.Gap_AcceptedGapWhileWaitIndices,indices.Gap_AcceptedGapIndices);
% indices.Gap_AcceptedGapWhileWaitIndicesNoOutlier = indices.Gap_AcceptedGapWhileWaitIndices(ind);
% 
% [~,ind,~] = intersect(indices.Gap_AcceptedGapsWhileApproachIndices,indices.Gap_AcceptedGapIndices);
% indices.Gap_AcceptedGapsWhileApproachIndicesNoOutlier = indices.Gap_AcceptedGapsWhileApproachIndices(ind);
% 
% 
% %% indices including waiting time outliers
% indices.WCRejectedGaps = indices.Gap_RejectedGapIndices;        %always without outliers for rejected gaps; 
% if out==0
%     indices.WCAllGaps = [indices.Gap_AcceptedGapWhileWaitIndicesNoOutlier;indices.Gap_RejectedGapIndices];
%     indices.WCAcceptedGaps = indices.Gap_AcceptedGapWhileWaitIndicesNoOutlier;
% else
%     indices.WCAllGaps = [indices.Gap_AcceptedGapWhileWaitIndices;indices.Gap_RejectedGapIndices];
%     indices.WCAcceptedGaps = indices.Gap_AcceptedGapWhileWaitIndices;
% end
% 
% 
% %% decision
% 
% AllGapsDecision = zeros(length(GapData),1);
% AllGapsDecision(indices.WCAcceptedGaps)=1;
% 
% WCAllGapsDecision = [ones(N,1);zeros(M,1)];
% 
% ExpectedGapAccSVM = ExpectedGapAcc(indices.WCAllGaps);
% GazeTypeSVM = GazeType(indices.WCAllGaps);
% 
% 
% SVMData = table(WCAllGapsDecision,ExpectedGapAccSVM,GazeTypeSVM);

% normExpectedGapAccSVM = (ExpectedGapAccSVM-nanmean(ExpectedGapAccSVM))/nanstd(ExpectedGapAccSVM);
% normGazeTypeVM = (GazeTypeSVM-nanmean(GazeTypeSVM))/nanstd(GazeTypeSVM);
% 
% 
% normSVMData = table(AWCAllGapsDecision,normExpectedGapAccSVM,normGazeTypeVM,normSpeedTypeSVM);




