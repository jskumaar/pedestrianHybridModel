%% Compile all data to mat files

%The current script uses the indices of the various discrete states based
%on a 1s moving average (MA) filter, but the actual values use 0.5 s MA
%filter (not sure why!!).

clear all
close all

%addpath (for HybridReadData function)
addpath('G:\My Drive\Research\Pedestrian Modelling Project\Modelling Scripts and Results\Scripts - HybridModel\Helper Functions')

%% Combines and Converts all the excel data to .mat files. Saves a lot of read time!!

%Pedestrian Window 5 time steps
% EventIndices = xlsread('DiscreteStateEventIndicesW5.xlsx');

%Pedestrian Window 11
EventIndices = xlsread('DiscreteStateEventIndicesW11_onlyforMLmodel_DONT_USE.xlsx');

ApproachStart =  EventIndices(:,4);
RetreatEnd = EventIndices(:,11);

for kk=1:30
    for jj=1:3
          Data = HybridReadData(kk,jj);
%           PedData = table(Data.SubjectID,Data.ScenarioID,Data.CrossingID,Data.Time,Data.PedestrianPosition,Data.PedestrianCartesianVelocity,Data.PedestrianAbsoluteVelocity,...
%                         Data.PedestrianDiscreteState,Data.PedestrianGazeAtVehicle,Data.GazeAtVehicleRatio,Data.GazeAngle,Data.PedestrianDistancetoCW,Data.PedestrianDistancetoCurb,Data.PedestrianHeading,...
%                         Data.PedestrianCumulativeWaitTime,VehicleLaneID,VehiclePosition,VehicleSpeed,VehicleAcceleration,NextVehiclePosition,...
%                         NextVehicleSpeed,NextVehicleAcceleration,AdjacentVehiclePosition,AdjacentVehicleSpeed,AdjacentVehicleAcceleration,...
%                         PedestrianVehicleDistance,VehicleTimeGaptoPedestrian,PedestrianNextVehicleDistance);
%         
%           VehData = table(Data.SubjectID,Data.ScenarioID,Data.CrossingID,Data.Time,Data.PedestrianPosition,Data.PedestrianCartesianVelocity,Data.PedestrianAbsoluteVelocity,...
%                         Data.PedestrianDiscreteState,Data.PedestrianGazeAtVehicle,Data.GazeAtVehicleRatio,Data.GazeAngle,Data.PedestrianDistancetoCW,Data.PedestrianDistancetoCurb,Data.PedestrianHeading,...
%                         Data.PedestrianCumulativeWaitTime,VehicleLaneID,VehiclePosition,VehicleSpeed,VehicleAcceleration,NextVehiclePosition,...
%                         NextVehicleSpeed,NextVehicleAcceleration,AdjacentVehiclePosition,AdjacentVehicleSpeed,AdjacentVehicleAcceleration,...
%                         PedestrianVehicleDistance,VehicleTimeGaptoPedestrian,PedestrianNextVehicleDistance);
%        
        % compile data for every crossing
        S = vartype('numeric');
        for ii=1:6
           
           indCrossing = 18*(kk-1)+6*(jj-1)+ii;
           indStart = ApproachStart(indCrossing);
           indEnd = RetreatEnd(indCrossing);
           
           DataPredict{indCrossing,1} = Data(indStart:indEnd,S);
         
        end
    end
end

save('AllFeaturesCrossingWise_PW_5.mat','DataPredict');
% save('PedestianDataCrossingWise.mat',PedMat);
% save('VehicleDataCrossingWise.mat',VehMat);
