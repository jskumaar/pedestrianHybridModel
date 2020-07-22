function Data = HybridReadData(kk,jj)

roundDen = 100;
%Read activiy data
[ActivityData,~] = xlsread('DiscreteStateEventIndicesW5.xlsx');

% [ActivityData,~] = xlsread('DiscreteStateEventIndicesW11_onlyforMLmodel_DONT_USE.xlsx');


%Read nearest vehicle DTC, position, next vehicle position,
%adjacent vehicle position data
[vehicleData,~] = xlsread('VehicleDTCSpeedData_allFourVehicles.xlsx',6*(kk-1)+jj);       
vehicleData = ceil(vehicleData*roundDen)/roundDen;

%Read gaze data
[GazeNumericData,GazeObjects] = xlsread('GazeNoDuplicate.xlsx',6*(kk-1)+jj);
        %Gaze at Vehicle categorical vector
        GazeAtVehicle = zeros(size(vehicleData,1),1);
        GazeAtVehicle(1) = 0;
        
        if isnan(GazeNumericData(1,4:6))
            GazeNumericData(1,4:6) = [0,0,0];
        end
        for ii=2:size(GazeNumericData,1)-1
                if (isnan(GazeNumericData(ii,4:6)))          
                    GazeNumericData(ii,4:6) = GazeNumericData(ii-1,4:6);
                end   
        end 
                
        for ii=2:size(GazeObjects,1)-1
            if (strcmp(GazeObjects{ii+1,11},'Looking at approach Vehicle'))
                GazeAtVehicle(ii) = 1;
            elseif(isempty(GazeObjects{ii+1,11}))
                 GazeAtVehicle(ii) = GazeAtVehicle(ii-1);       %if no data is available previous gaze state is retained
            end
                  
        end

        %Gaze Angle
        GazeX = GazeNumericData(:,4);
        GazeZ = GazeNumericData(:,6);
        % Filter    
        [b,a] = butter(6,0.2);      %In FFT most information is in +/- 1 Hz range
        GazeZNaNButterFilt = filter(b,a,GazeZ);
        GazeXNaNButterFilt = filter(b,a,GazeX);
        GazeAngle = atan2(GazeZNaNButterFilt,GazeXNaNButterFilt); 


%Rolling Window gaze ratio at vehicle
WindowSize = [10,15,20,25,30]; % 1 seconds

for ii=1:5
    window = ones(WindowSize(ii),1)/WindowSize(ii);
    GazeAtVehicleRatio(:,ii) = conv(GazeAtVehicle,window,'same');
    PedestrianAbsoluteVelocityAverage(:,ii) = conv(PedestrianData(:,8),window,'same');
end



%Read pedestrian position data
% [PedestrianData,~] = xlsread('PedestrianDiscreteDataW11_onlyforMLmodel.xlsx',6*(kk-1)+jj);
[PedestrianData,~] = xlsread('PedestrianDiscreteDataW5.xlsx',6*(kk-1)+jj);

 N = length(PedestrianData);
%% Combine all Data
% General parameters
SubjectID = PedestrianData(:,1);
ScenarioID = PedestrianData(:,2);
Time = PedestrianData(:,3);
CrossingID = zeros(N,1);
for ii=1:6
    CrossingID(ActivityData(18*(kk-1)+6*(jj-1)+ii,4):ActivityData(18*(kk-1)+6*(jj-1)+ii,12),1) = ii;
end


%Pedestrian Parameters
PedestrianPosition = PedestrianData(:,4:5);
PedestrianCartesianVelocity = PedestrianData(:,6:7);
PedestrianAbsoluteVelocity = PedestrianData(:,8);
PedestrianDiscreteState = PedestrianData(:,9);
PedestrianGazeAtVehicle = GazeAtVehicle;
PedestrianDistancetoCW = sqrt(PedestrianPosition(:,1).^2+(abs(PedestrianPosition(:,2))-3.5).^2);    %CW centers at each side of road is at (0,3.5) and (0,-3.5)
PedestrianDistancetoCurb = abs(PedestrianPosition(:,2)-3.5);
PedestrianHeading = atan2(PedestrianCartesianVelocity(:,2),PedestrianCartesianVelocity(:,1));

%cumulative wait
indStart= find(PedestrianDiscreteState==2 & [0;diff(PedestrianDiscreteState)]==1);
indEnd= find(PedestrianDiscreteState==2 & [diff(PedestrianDiscreteState);0]==1);
PedestrianCumulativeWaitTime = zeros(N,1);

for ii=1:length(indStart)
    PedestrianCumulativeWaitTime(indStart(ii):indEnd(ii),1) = [0:0.1:(indEnd(ii)-indStart(ii))*0.1];
end


% %time-to-event
% tempInd = find(PedestrianDiscreteState==1);
% indA = find(diff(tempInd_~=0));
% tempInd = find(PedestrianDiscreteState==2);
% indB = find(diff(tempInd_~=0));
% tempInd = find(PedestrianDiscreteState==3);
% indC = find(diff(tempInd_~=0));
% tempInd = find(PedestrianDiscreteState==4);
% indD = find(diff(tempInd_~=0));


%VehicleParameters
VehicleLaneID = vehicleData(:,4);
VehiclePosition = vehicleData(:,6);
VehicleSpeed = vehicleData(:,7);
VehicleAcceleration = vehicleData(:,8);
NextVehiclePosition = vehicleData(:,9);
NextVehicleSpeed = vehicleData(:,10);
NextVehicleAcceleration = vehicleData(:,11);
AdjacentVehiclePosition = vehicleData(:,12);
AdjacentVehicleSpeed = vehicleData(:,13);
AdjacentVehicleAcceleration = vehicleData(:,14);

%Interaction parameters
PedestrianVehicleDistance = VehiclePosition - PedestrianPosition(:,1);
PedestrianNextVehicleDistance = NextVehiclePosition - PedestrianPosition(:,1);
VehicleTimeGaptoPedestrian = PedestrianVehicleDistance./abs(VehicleSpeed);
NextVehicleTimeGaptoPedestrian = PedestrianNextVehicleDistance./abs(NextVehicleSpeed);

check=1;
while (check)
    ind = find(isinf(VehicleTimeGaptoPedestrian)==1);
    VehicleTimeGaptoPedestrian(ind) = VehicleTimeGaptoPedestrian(ind+1);
    NextVehicleTimeGaptoPedestrian(ind) = NextVehicleTimeGaptoPedestrian(ind+1);
    check = find(ind~=0);
end

%Table data
Data = table(SubjectID,ScenarioID,CrossingID,Time,PedestrianPosition,PedestrianCartesianVelocity,PedestrianAbsoluteVelocity,...
            PedestrianDiscreteState,PedestrianGazeAtVehicle,GazeAtVehicleRatio,GazeAngle,PedestrianDistancetoCW,PedestrianDistancetoCurb,PedestrianHeading,...
            PedestrianCumulativeWaitTime,VehicleLaneID,VehiclePosition,VehicleSpeed,VehicleAcceleration,NextVehiclePosition,...
            NextVehicleSpeed,NextVehicleAcceleration,AdjacentVehiclePosition,AdjacentVehicleSpeed,AdjacentVehicleAcceleration,...
            PedestrianVehicleDistance,VehicleTimeGaptoPedestrian,PedestrianNextVehicleDistance,NextVehicleTimeGaptoPedestrian);
end