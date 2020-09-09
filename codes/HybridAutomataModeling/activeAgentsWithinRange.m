%% This script calculates the active agents that are within the sensing region of the AV

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% initialize
activePedWithinRange = [];
activeCarWithinRange = [];

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
    tempPedCarHeading = atan2(double(carPosPixels(2)-tempPedPosPixels(2)), double(carPosPixels(1)-tempPedPosPixels(1)));
        
    distPedEgo = norm(double(tempPedPosPixels-carPosPixels));   
    if  ( distPedEgo < Params.sensingRange && abs(carHeading - tempPedCarHeading) < 90 )
        activePedWithinRange = [activePedWithinRange; tempPedId];
    end
end

%% find which pedestrians are within the sensing region of the
% vehicle
for id = 1: length(activeCarTracks)
    tempCarId = activeCarTracks(id);
    tempCarData = formattedTracksData{sceneId}{tempCarId};
    tempCarTimeStep = find(tempCarData.frame==trackTime);
    tempCarPosPixels = double([tempCarData.xCenterPix(tempCarTimeStep), tempCarData.yCenterPix(tempCarTimeStep)]);
    tempCarHeading = tempCarData.calcHeading(tempCarTimeStep);
    
    distCarEgo = norm(double(tempCarPosPixels - carPosPixels));  

    if  ( distCarEgo < Params.sensingRange && abs(carHeading - tempCarHeading) < 90 )
        activeCarWithinRange = [activeCarWithinRange; tempCarId];
    end
end