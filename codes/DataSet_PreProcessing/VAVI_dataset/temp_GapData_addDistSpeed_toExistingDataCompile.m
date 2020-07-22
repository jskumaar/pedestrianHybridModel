%% Use the previously compiled expected gap data and add the distance and vehicle speeds

%% Updated: 10/28/2019
% The files uses the 'ExpectedGapData' file used for the RA-L paper and
% adds vehicle distance to pedestrian and vehicle speed to it. (Did not use
% the new compilation 'temp_GapData_adds_distance_fullDataCompile.m'
% because there seemed to be a mismatch between the Crossing Decision
% variables.


clear all

%add path to the MATLAB variables and excel sheets
addpath('G:\my drive\Research\Pedestrian Modelling Project\Modelling Scripts and Results\2. Mat Data\')
addpath('G:\My Drive\Research\Pedestrian Modelling Project\Modelling Scripts and Results\1. Study I Data for Modeling\Compiled Data\');

% load the files
load('AllFeaturesCrossingWise_PW_5_SpeedHist_CorrectDTCurbDTCW.mat')
load('HybridModelTestTrainIndices.mat')
load('ExpectedGapData_W5_NewDTCurbDTCW_StartingTimeforCrossingGaps.mat')
[high_dec_data,~] = xlsread('High_acceleration_data.xlsx',1);
[EventIndices,~] = xlsread('DiscreteStateEventIndicesW5.xlsx');
[GapData,~] = xlsread('VehicleGapTimesV6.xlsx');   %changed the -0.1 gap duratiosn to 0.1
S = vartype('numeric');


%% add the vehicle distance and speed data
%variables to add
VehicleDistancetoPed  = [];
VehicleSpeed  = [];

CrossingStartTimeStep = EventIndices(:,8);
ApproachStartTimeStep = EventIndices(:,4);

GapStartTimeStep = GapData(:,5);
GapEndTimeStep = GapData(:,8);
GapEndOnRoadTimeStep = GapData(:,9);

%indices of vehicle gaps for each crossing
ind = find(diff(GapData(:,3))~=0);
indStart = [1;ind+1];
indEnd = [ind;length(GapData)];

%variables
gap_count=1;

%% Data compile loop 
for cross_no=1:540
        
                syms x
                indtoFind = [indStart(cross_no):indEnd(cross_no)];

                % for all vehicle gaps in that crossing
                for gap_in_cross_no=1:length(indtoFind)

                    GapStartIndex = GapStartTimeStep(indtoFind(gap_in_cross_no)) - ApproachStartTimeStep(cross_no)+1;      %when vehicle gap starts before the pedestrian starts their approach, assume the vehicle gap starts as when they first start approaching
                    if GapStartIndex<=0
                        GapStartIndex=1;
                    end

                       % new variables - added 10/17/19
                       VehicleDistancetoPed = [VehicleDistancetoPed; DataPredict{cross_no}.PedestrianVehicleDistance(GapStartIndex,:)];
                       VehicleSpeed = [VehicleSpeed; abs(DataPredict{cross_no}.VehicleSpeed(GapStartIndex,:))];
                    %update the overall gap counter
                    gap_count = gap_count+1;
                end
                clear gap_no

end


%% add the new data to the expected gap data

ExpectedGapData.VehicleDistancetoPed = VehicleDistancetoPed;
ExpectedGapData.VehicleSpeed = VehicleSpeed;

save('ExpectedGapData_W5_wVehDistSpeed_10_28_19.mat','ExpectedGapData');

%% SVM Full Data
CrossingDecision = ExpectedGapData.WCAllGapsDecision_CrossDecisionOnRoadGap;
ExpectedTimeGap_speed = ExpectedGapData.WCExpectedGapStartGap;
ExpectedTimeGap_acc = ExpectedGapData.WCExpectedGapStartGapAcc;
CumulativeWaitTime = ExpectedGapData.PedestrianCumulativeWaitTime(:,10);
PedestrianVelocity = ExpectedGapData.PedestrianAbsoluteVelocityAverage(:,1);
GazeRatio = ExpectedGapData.GazeRatiosGapStart(:,1);
DTCurb = ExpectedGapData.PedestrianDistancetoCurb;
DTCW = ExpectedGapData.PedestrianDistancetoCW;



SVMFullData = table(CrossingDecision,ExpectedTimeGap_speed,ExpectedTimeGap_acc,...
                    CumulativeWaitTime,PedestrianVelocity,GazeRatio,DTCurb,DTCW, VehicleDistancetoPed, VehicleSpeed);



% identify the crossing indices that have high deceleration
high_dec_index_to_remove = [];
for ii=1:size(high_dec_data,1)
    high_dec_index_to_remove = [high_dec_index_to_remove, 18*(high_dec_data(ii,1)-1) + 6*(high_dec_data(ii,2)-1) + [high_dec_data(ii,3):6]]; 
end
high_dec_index_to_remove = high_dec_index_to_remove';


%find gap indices that have the crossing number of high_dec
EG_CrossingNumber = 18*(ExpectedGapData.VehicleGapTimes(:,1)-1) + 6*(ExpectedGapData.VehicleGapTimes(:,2)-1) + ExpectedGapData.VehicleGapTimes(:,3);

EG_high_dec_index = [];
for ii=1:length(high_dec_index_to_remove)
    EG_high_dec_index = [EG_high_dec_index; find(EG_CrossingNumber == high_dec_index_to_remove(ii))];
end

TrainGapIndices_extreme = [];
for ii=1:length(TrainIndices_woExtremeOutlier)
    TrainGapIndices_extreme = [TrainGapIndices_extreme; find(EG_CrossingNumber == TrainIndices_woExtremeOutlier(ii))];
end

TrainGapIndices_mild = [];
for ii=1:length(TrainIndices_woMildOutlier)
    TrainGapIndices_mild = [TrainGapIndices_mild; find(EG_CrossingNumber == TrainIndices_woMildOutlier(ii))];
end

[~,to_rem_ind,~] = intersect(TrainGapIndices_extreme,EG_high_dec_index);
TrainGapIndices_extreme_noHigh_dec = TrainGapIndices_extreme;
TrainGapIndices_extreme_noHigh_dec(to_rem_ind) = [];

[~,to_rem_ind,~] = intersect(TrainGapIndices_mild,EG_high_dec_index);
TrainGapIndices_mild_noHigh_dec = TrainGapIndices_mild;
TrainGapIndices_mild_noHigh_dec(to_rem_ind) = [];


TestGapIndices_extreme = [];
for ii=1:length(TestIndices_woExtremeOutlier)
    TestGapIndices_extreme = [TestGapIndices_extreme; find(EG_CrossingNumber == TestIndices_woExtremeOutlier(ii))];
end
 
TestGapIndices_mild = [];
for ii=1:length(TestIndices_woMildOutlier)
    TestGapIndices_mild = [TestGapIndices_mild; find(EG_CrossingNumber == TestIndices_woMildOutlier(ii))];
end
 
[~,to_rem_ind,~] = intersect(TestGapIndices_extreme,EG_high_dec_index);
TestGapIndices_extreme_noHigh_dec = TestGapIndices_extreme;
TestGapIndices_extreme_noHigh_dec(to_rem_ind) = [];
 
[~,to_rem_ind,~] = intersect(TestGapIndices_mild,EG_high_dec_index);
TestGapIndices_mild_noHigh_dec = TestGapIndices_mild;
TestGapIndices_mild_noHigh_dec(to_rem_ind) = [];

         
%SVM data
SVMTrain_woHighDec_extreme = SVMFullData(TrainGapIndices_extreme_noHigh_dec,S);
SVMTest_woHighDec_extreme = SVMFullData(TestGapIndices_extreme_noHigh_dec,S);

% SVMTrain_woHighDec_mild = SVMFullData(TrainGapIndices_mild_noHigh_dec,S);
% 
% SVMTrain_HighDec_extreme = SVMFullData(TrainGapIndices_extreme,S);
% 
% SVMTrain_HighDec_mild = SVMFullData(TrainGapIndices_mild,S);


save('SVMTrain_woHighDec_extreme.mat','SVMTrain_woHighDec_extreme')
save('SVMTest_woHighDec_extreme.mat','SVMTest_woHighDec_extreme')


% save('SVMTrain_woHighDec_mild.mat','SVMTrain_woHighDec_mild')
% save('SVMTrain_HighDec_extreme.mat','SVMTrain_HighDec_extreme')
% save('SVMTrain_HighDec_mild.mat','SVMTrain_HighDec_mild')
