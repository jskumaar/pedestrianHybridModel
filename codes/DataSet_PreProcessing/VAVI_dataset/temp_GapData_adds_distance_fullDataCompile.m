%% File to compile crossing wise based on the vehicle gaps
% this is a cleaner version with only the most relevant variables; for
% other gap related variables (e.g. gap when pedestrian on road, gap when
% pedestrian started crossing, etc., codes are available in
% 'GapFromSpeed.m'

%% Updated: 10/28/2019
% 1) Vehicle Distance from pedestrian and vehicle speed data also compiled
% 2) High deceleration data removed (32 crossings in total out of 540);
% data from crossings where the AV stopped really fast really close to the
% pedestrian has been removed as users might have thought the AVs will
% always stop for them, which in turn can bias their behavior. For this
% reason, data from all crossings following the above crossing instances
% were also removed.

%Issues (10/28/2019)
%The crossing decision mentioned in this dataset does not seem to match the
%crossing decision variable used in the previous data (must check!) -
%accuracy drops to 79% in the former case from ~85% in the latter case.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all

%addpath
addpath('G:\My Drive\Research\Pedestrian Modelling Project\Modelling Scripts and Results\1. Study I Data for Modeling\Compiled Data')
addpath('G:\My Drive\Research\Pedestrian Modelling Project\Modelling Scripts and Results\2. Mat Data')

%Read data
[EventIndices,~] = xlsread('DiscreteStateEventIndicesW5.xlsx');
[GapData,~] = xlsread('VehicleGapTimesV6.xlsx');   %changed the -0.1 gap duratiosn to 0.1
load('AllFeaturesCrossingWise_PW_5_SpeedHist_CorrectDTCurbDTCW.mat')



% identify the crossing indices that have high deceleration
% (not needed to run the function as the data is available on an excel
% sheet).
% high_dec_data = HighDeceleration_data_separate(10,3);   %1st input - deceleration limit, 2nd input - stopped distance from pedestrian
[high_dec_data,~] = xlsread('High_acceleration_data.xlsx',1);

high_dec_index_to_remove = [];
for ii=1:size(high_dec_data,1)
    high_dec_index_to_remove = [high_dec_index_to_remove, 18*(high_dec_data(ii,1)-1) + 6*(high_dec_data(ii,2)-1) + [high_dec_data(ii,3):6]]; 
end
high_dec_index_to_remove = high_dec_index_to_remove';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Step 2: Create the table with the data for ML

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

Gaps_high_dec_rem_data = [];
CrossingDecision = [];
ExpectedTimeGap_speed = [];
ExpectedTimeGap_acc  = [];
GazeRatio  = [];
PedestrianVelocity  = [];
DTCW  = [];
DTCurb  = [];
PedestrianHeading  = [];
CumulativeWaitTime = [];
GazeAngle  = [];
VehicleLaneID  = [];
VehicleDistancetoPed  = [];
VehicleSpeed  = [];
% 
% 
% 
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

                    % collect the data when the gap starts; it is assumed that decision is made during that instant!
                    if GapStartIndex>0
                        ExpectedTimeGap_speed = [ExpectedTimeGap_speed; DataPredict{cross_no}.VehicleTimeGaptoPedestrian(GapStartIndex)];

                       %1) acceleration based gap when crossed gap starts
                        soln = double(real(solve(-DataPredict{cross_no}.VehicleAcceleration(GapStartIndex)/2*x^2 - DataPredict{cross_no}.VehicleSpeed(GapStartIndex)*x...
                                                            - DataPredict{cross_no}.PedestrianVehicleDistance(GapStartIndex)==0,x,'IgnoreAnalyticConstraints', true))); 

                        % find the solution that is closed to the previous speed based
                        % gap; if no solution exists use the speed gap for acceleration
                        % also
                       [~,temp] = min(abs(soln-ExpectedTimeGap_speed(end)));
                       if ~isempty(temp)
                           ExpectedTimeGap_acc = [ExpectedTimeGap_acc; soln(temp)];
                       else
                           ExpectedTimeGap_acc = [ExpectedTimeGap_acc; ExpectedTimeGap_speed(end)];
                       end

                       %2) Other features for the gap model
                       GazeRatio = [GazeRatio; DataPredict{cross_no}.GazeAtVehicleRatio(GapStartIndex,1)];
                       PedestrianVelocity = [PedestrianVelocity; DataPredict{cross_no}.PedestrianAbsoluteVelocityAverage(GapStartIndex,1)];
                       PedestrianHeading = [PedestrianHeading; DataPredict{cross_no}.PedestrianHeading(GapStartIndex,:)];
                       CumulativeWaitTime = [CumulativeWaitTime; DataPredict{cross_no}.PedestrianCumulativeWaitTime(GapStartIndex,:)];
                       GazeAngle = [GazeAngle; DataPredict{cross_no}.GazeAngle(GapStartIndex,:)];
                       VehicleLaneID = [VehicleLaneID; DataPredict{cross_no}.VehicleLaneID(GapStartIndex,:)];
                       DTCurb = [DTCurb; DataPredict{cross_no}.PedestrianDistancetoCurb_new(GapStartIndex,:)];
                       DTCW = [DTCW; DataPredict{cross_no}.PedestrianDistancetoCW_new(GapStartIndex,:)];

                       % new variables - added 10/17/19
                       VehicleDistancetoPed = [VehicleDistancetoPed; DataPredict{cross_no}.PedestrianVehicleDistance(GapStartIndex,:)];
                       VehicleSpeed = [VehicleSpeed; abs(DataPredict{cross_no}.VehicleSpeed(GapStartIndex,:))];

                       %check of it is a crossing gap
                       if CrossingStartTimeStep(cross_no) >= GapStartTimeStep(indtoFind(gap_in_cross_no)) & CrossingStartTimeStep(cross_no) <= GapEndTimeStep(indtoFind(gap_in_cross_no))
                           CrossingDecision = [CrossingDecision; 1];
                       else
                           CrossingDecision = [CrossingDecision; 0];
                       end
                       
                       
                       %GapData
                       Gaps_high_dec_rem_data = [Gaps_high_dec_rem_data; GapData(indtoFind(gap_in_cross_no),:)];


                    end

                    %update the overall gap counter
                    gap_count = gap_count+1;


                end
                clear gap_no

end



GapDataML = table(GapData,ExpectedTimeGap_speed,ExpectedTimeGap_acc,GazeRatio, PedestrianVelocity,DTCW,...
                        DTCurb,PedestrianHeading,CumulativeWaitTime,GazeAngle,VehicleLaneID,VehicleDistancetoPed,VehicleSpeed,CrossingDecision);
                    
save('GapDataML_10_21_2019.mat','GapDataML');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Step 3: Split the data into test and train based on the previously separated indices

load('HybridModelTestTrainIndices.mat')
S = vartype('numeric');

TrainIndices = TrainIndices_woMildOutlier;
TestIndices = TestIndices_woMildOutlier;

% crossing number for the gaps
CrossingNumber = 18*(Gaps_high_dec_rem_data(:,1)-1) + 6*(Gaps_high_dec_rem_data(:,2)-1) + Gaps_high_dec_rem_data(:,3);


GapIndicesTrain=[];
for ii=1:length(TrainIndices)
    temp=[];
    temp(:,1) = find(CrossingNumber==TrainIndices(ii));   
    GapIndicesTrain = [GapIndicesTrain; temp];
end

GapIndicesTest=[];
for ii=1:length(TestIndices)
    temp=[];
    temp(:,1) = find(CrossingNumber==TestIndices(ii));   
    GapIndicesTest = [GapIndicesTest; temp];
end

SVMTrainData_new = GapDataML(GapIndicesTrain,S);
SVMTestData_new = GapDataML(GapIndicesTest,S);
SVMData_new = [SVMTrainData_new;SVMTestData_new];

save('SVMData_ExtremeOutlier_NoHighDeceleration_10_22_19.mat','SVMData');
save('SVMTrainData_ExtremeOutlier_NoHighDeceleration_10_22_19.mat','SVMTrainData');
save('SVMTestData_ExtremeOutlier_NoHighDeceleration_10_22_19.mat','SVMTestData');




