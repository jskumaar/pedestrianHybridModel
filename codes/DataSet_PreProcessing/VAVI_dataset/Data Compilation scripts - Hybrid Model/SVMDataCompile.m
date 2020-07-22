% For SVM
clear all

addpath('G:\my drive\Research\Pedestrian Modelling Project\Modelling Scripts and Results\Mat Data\')
addpath('G:\My Drive\Research\Pedestrian Modelling Project\Study I Data for Modeling\Compiled Data\');

% E1 =  load('ExpectedGapData_W5_NewDTCurbDTCW_StartingTimeforCrossingGaps.mat');
load('ExpectedGapData_10_17_2019.mat');


% save('ExpectedGapData_W5_OldDTCurbDTCW_CrossingTimeforCrossingGaps.mat','ExpectedGapData')

load('HybridModelTestTrainIndices.mat')
S = vartype('numeric');
out=1;

% E2.ExpectedGapData.CrossingNumber = E1.ExpectedGapData.CrossingNumber;
% E2.ExpectedGapData.DecisionBeforeGapStart = E1.ExpectedGapData.DecisionBeforeGapStart;
% E2.ExpectedGapData.WCAllGapsDecision_CrossDecisionOnRoadGap = E1.ExpectedGapData.WCAllGapsDecision_CrossDecisionOnRoadGap;
% E2.ExpectedGapData.DiscreteState = E1.ExpectedGapData.DiscreteState;
% E2.ExpectedGapData.PedestrianCumulativeWaitTime = E2.ExpectedGapData.VehicleGapTimes(:,10);
% 
% ExpectedGapData = E2.ExpectedGapData;

% ind = find(ExpectedGapData.VehicleGapTimes(:,4)==2);
% WCAllGapsDecision_CrossDecisionOnRoadGap(ind,1)=1;
 
% ExpectedGap_SVM = normalize(ExpectedGap(indices.WCAllGaps));
% Gaze_SVM = normalize(ExpectedGapData.GazeRatiosGapStart(indices.WCAllGaps,1));
% VelocityAverage_SVM = normalize(ExpectedGapData.PedestrianAbsoluteVelocityAverage(indices.WCAllGaps,1));
% PedestrianDistancetoCurb_SVM = normalize(ExpectedGapData.PedestrianDistancetoCurb(indices.WCAllGaps));
% PedestrianDistancetoCW_SVM = normalize(ExpectedGapData.PedestrianDistancetoCW(indices.WCAllGaps));
% PedestrianCumulativeWaitTime_SVM = normalize(ExpectedGapData.PedestrianCumulativeWaitTime(indices.WCAllGaps));
% GazeAngle_SVM = normalize(ExpectedGapData.GazeAngle(indices.WCAllGaps));
% VehicleLaneID_SVM = normalize(ExpectedGapData.VehicleLaneID(indices.WCAllGaps));
% PedestrianHeading_SVM = normalize(ExpectedGapData.PedestrianHeading(indices.WCAllGaps));
% 
% ExpectedGapData.WCAllGapsDecision_CrossDecisionOnRoadGap = WCAllGapsDecision_CrossDecisionOnRoadGap;

% DiscreteState = GapData(:,11);
% 
% ExpectedGapData.DiscreteState = DiscreteState;

CrossingNumber = 18*ExpectedGapData.VehicleGapTimes(:,1) + 3*ExpectedGapData.VehicleGapTimes(:,2) + ExpectedGapData.VehicleGapTimes(:,3);



%% remove high deceleration data

high_deceleration_data = xlsread('High_acceleration_data.xlsx',1);

high_deceleration_cross_number = 18*high_deceleration_data(:,1) + 3*high_deceleration_data(:,2) + [high_deceleration_data(:,3):6];


no_high_deceleration = [1:540];
no_high_deceleration(high_deceleration_cross_number)=[];


TrainIndices_woDec = intersect(TrainIndices_woExtremeOutlier,no_high_deceleration);
TestIndices_woDec = intersect(TestIndices_woExtremeOutlier,no_high_deceleration);


% TrainIndices_woDec = TrainIndices_woExtremeOutlier;
% TestIndices_woDec = TestIndices_woExtremeOutlier;



GapIndicesTrain=[];
for ii=1:length(TrainIndices_woDec)
    temp=[];
    temp(:,1) = find(CrossingNumber==TrainIndices_woDec(ii));   
    GapIndicesTrain = [GapIndicesTrain; temp];
end
% SVMIndicesTrain = ExpectedGapData([ExpectedGapData.VehicleGapTimes(GapIndicesTrain,:),ExpectedGapData.DiscreteState(GapIndicesTrain)],out);


GapIndicesTest=[];
for ii=1:length(TestIndices_woDec)
    temp=[];
    temp(:,1) = find(CrossingNumber==TestIndices_woDec(ii));   
    GapIndicesTest = [GapIndicesTest; temp];
end
% SVMIndicesTest = ExpectedGapData([ExpectedGapData.VehicleGapTimes(GapIndicesTest,:),ExpectedGapData.DiscreteState(GapIndicesTest)],out);



ExpectedGapData.GazeRatiosGapStart = ExpectedGapData.GazeRatiosGapStart(:,1);
ExpectedGapData.VehicleGapTimes = [];
ExpectedGapData.GazeRatiosBeforeCrossing = [];
ExpectedGapData.GazeRatiosBeforeOnRoad = [];
ExpectedGapData.PedestrianAbsoluteVelocityAverage = ExpectedGapData.PedestrianAbsoluteVelocityAverage(:,1);



% SVMTrainData = ExpectedGapData(GapIndicesTrain(SVMIndicesTrain.WCAllGaps),S);
% SVMTestData = ExpectedGapData(GapIndicesTest(SVMIndicesTest.WCAllGaps),S);
% SVMData = [SVMTrainData;SVMTestData];

SVMTrainData = ExpectedGapData(GapIndicesTrain,S);
SVMTestData = ExpectedGapData(GapIndicesTest,S);
SVMData = [SVMTrainData;SVMTestData];

save('SVMData_ExtremeOutlier_StartGapExpectedGap_NoHighDeceleration.mat','SVMData');
save('SVMTrainData_ExtremeOutlier_StartGapExpectedGap_NoHighDeceleration.mat','SVMTrainData');
save('SVMTestData_ExtremeOutlier_StartGapExpectedGap_NoHighDeceleration.mat','SVMTestData');


% 
% ExpectedGap_SVM = ExpectedGapData.WCExpectedGapStartGap(ind);
% Gaze_SVM = ExpectedGapData.GazeRatiosGapStart(ind,1);
% VelocityAverage_SVM = ExpectedGapData.PedestrianAbsoluteVelocityAverage(ind,1);
% PedestrianDistancetoCurb_SVM = ExpectedGapData.PedestrianDistancetoCurb(ind);
% PedestrianDistancetoCW_SVM = ExpectedGapData.PedestrianDistancetoCW(ind);
% PedestrianCumulativeWaitTime_SVM = ExpectedGapData.VehicleGapTimes(ind,10);
% GazeAngle_SVM = ExpectedGapData.GazeAngle(ind);
% VehicleLaneID_SVM = ExpectedGapData.VehicleLaneID(ind);
% PedestrianHeading_SVM = ExpectedGapData.PedestrianHeading(ind);
% CrossingNumber = 18*(ExpectedGapData.VehicleGapTimes(ind,1)-1) + 6*(ExpectedGapData.VehicleGapTimes(ind,2)-1) + ExpectedGapData.VehicleGapTimes(ind,3);
% WCAllGapsDecision_CrossDecisionOnRoadGap = ExpectedGapData.WCAllGapsDecision_CrossDecisionOnRoadGap(ind);
% 
% 
% ExpectedGap_SVM = ExpectedGapData.WCExpectedGapStartGap(ind);
% Gaze_SVM = ExpectedGapData.GazeRatiosGapStart(ind,1);
% VelocityAverage_SVM = ExpectedGapData.PedestrianAbsoluteVelocityAverage(ind,1);
% PedestrianDistancetoCurb_SVM = ExpectedGapData.PedestrianDistancetoCurb(ind);
% PedestrianDistancetoCW_SVM = ExpectedGapData.PedestrianDistancetoCW(ind);
% PedestrianCumulativeWaitTime_SVM = ExpectedGapData.VehicleGapTimes(ind,10);
% GazeAngle_SVM = ExpectedGapData.GazeAngle(ind);
% VehicleLaneID_SVM = ExpectedGapData.VehicleLaneID(ind);
% PedestrianHeading_SVM = ExpectedGapData.PedestrianHeading(ind);
% CrossingNumber = 18*(ExpectedGapData.VehicleGapTimes(ind,1)-1) + 6*(ExpectedGapData.VehicleGapTimes(ind,2)-1) + ExpectedGapData.VehicleGapTimes(ind,3);
% WCAllGapsDecision_CrossDecisionOnRoadGap = ExpectedGapData.WCAllGapsDecision_CrossDecisionOnRoadGap(ind);

% 
% SVMTrainData = SVMData(SVMTrainIndices,S);
% SVMTestData = SVMData(SVMTestIndices,S);











