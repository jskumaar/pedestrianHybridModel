%% 
clear all

load('ExpectedGapData_W5_SpeedHist_Correct.mat')
load('AllFeaturesCrossingWise_PW_5_SpeedHist.mat')

for jj=1:1
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% PedestrianAbsoluteVelocity = DataPredict{jj}.PedestrianAbsoluteVelocity;
% 
% %Rolling Window gaze ratio at vehicle
% WindowSize = [10,15,20,25,30]; % 1 seconds
% PedestrianAbsoluteVelocityAverage = [];
% for ii=1:5
%     window = ones(WindowSize(ii),1)/WindowSize(ii);
%     PedestrianAbsoluteVelocityAverage(:,ii) = conv(PedestrianAbsoluteVelocity,window,'same');
% end
% 
% DataPredict{jj}.PedestrianAbsoluteVelocityAverage = PedestrianAbsoluteVelocityAverage;
% 


% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PedestrianDistancetoCurb = DataPredict{jj}.PedestrianDistancetoCurb;
% ind = find(DataPredict{jj}.VehicleLaneID==2);
% PedestrianDistancetoCurb(ind) = PedestrianDistancetoCurb(ind)-7;
% DataPredict{jj}.PedestrianDistancetoCurb_new = PedestrianDistancetoCurb;
% 
% 
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PedestrianDistancetoCW = DataPredict{jj}.PedestrianDistancetoCW;
% 
% % pedestrian x-distance to crosswalk
% PedestrianDistancetoCW_new = abs(DataPredict{jj}.PedestrianPosition(:,1));
% DataPredict{jj}.PedestrianDistancetoCW_new = PedestrianDistancetoCW_new;






end


% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Expected Gap - 1
ExpectedGap = ExpectedGapData.WCExpectedGapStartGap;
WCExpectedNextVehicleGapCrossStart = ExpectedGapData.WCExpectedNextVehicleGapCrossStart;

CrossIndices = find(ExpectedGapData.VehicleGapTimes(:,4)==2 & ExpectedGapData.WCExpectedGapStartGap<20 & ExpectedGapData.WCExpectedGapCrossStart<20 & ExpectedGapData.WCExpectedGapOnRoad<20);
NotSameGapIndices = find(ExpectedGapData.VehicleGapTimes(:,4)==1);
NotSameGapIndicesCheck = NotSameGapIndices+1;
[CommonIndices,~,~] = intersect(CrossIndices,NotSameGapIndicesCheck);

ExpectedGap(CommonIndices) = WCExpectedNextVehicleGapCrossStart(CommonIndices);

ExpectedGapData.ExpectedGap_bothCurrentNextVehicle = ExpectedGap;

save('ExpectedGapData_W5_SpeedHist_Correct','ExpectedGapData');