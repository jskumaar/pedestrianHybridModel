%% This script calculates the active agents that are within the sensing region of the AV

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% initialize
activePedWithinRange = zeros(100,1);
activeCarWithinRange = zeros(100,1);
pedId = 1;
carId = 1;

%% Find the active car and pedestrian tracks for this time step
pedTracks = [tracks{sceneId}.pedCrossingTracks; tracks{sceneId}.pedNotCrossingTracks]; % exclude jaywalking pedestrian data
activeTracks = find(tracksMetaData{sceneId}.finalFrame >= trackTime &... 
                    tracksMetaData{sceneId}.initialFrame <= trackTime);  
activeCarTracks = intersect(tracks{sceneId}.carMovingTracks, activeTracks);
activePedTracks = intersect(pedTracks, activeTracks);

%% find which pedestrians are within the sensing region of the
% vehicle
for id = 1: length(activePedTracks)
    tempPedId = activePedTracks(id);
    tempPedData = formattedTracksData{sceneId}{tempPedId};
    tempPedTimeStep = find(tempPedData.frame==trackTime);
    tempPedPosPixels = [tempPedData.xCenterPix(tempPedTimeStep), tempPedData.yCenterPix(tempPedTimeStep)];
    tempPedCarHeading = atan2(double(tempPedPosPixels(2)-carPosPixels(2)), double(tempPedPosPixels(1))-carPosPixels(1))*180/pi;
        
    distPedEgo = norm(double(tempPedPosPixels-carPosPixels));   
    if  ( distPedEgo < Params.sensingRange && (abs(carHeading - tempPedCarHeading) < 90  || abs(carHeading - tempPedCarHeading) > 270 ) )
        activePedWithinRange(pedId) = tempPedId;
        pedId = pedId + 1;
    end
end

%% find which cars are within the sensing region of the ego-vehicle
% include the ego-car

for id = 1: length(activeCarTracks)
    tempCarId = activeCarTracks(id);
    tempCarData = formattedTracksData{sceneId}{tempCarId};
    tempCarTimeStep = find(tempCarData.frame==trackTime);
    tempCarPosPixels = double([tempCarData.xCenterPix(tempCarTimeStep), tempCarData.yCenterPix(tempCarTimeStep)]);
    tempCarHeading = atan2(double(tempCarPosPixels(2)-carPosPixels(2)), double(tempCarPosPixels(1))-carPosPixels(1))*180/pi;
        
    
    distCarEgo = norm(double(tempCarPosPixels - carPosPixels));  

    if ( (distCarEgo==0) || ( distCarEgo < Params.sensingRange && ( abs(carHeading - tempCarHeading) < 90 || abs(carHeading - tempCarHeading) > 270 ) ) )
        activeCarWithinRange(carId) = tempCarId;
        carId = carId + 1;
    end
end

% remove
activePedWithinRange(pedId:end) = [];
activeCarWithinRange(carId:end) = [];