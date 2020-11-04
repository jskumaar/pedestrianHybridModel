%% temp check wait time


for sceneId = 1:N_Scenes
    
    N_tracks = size(formattedTracksData,1);
    
    for trackId = 1:N_tracks
        if strcmp(formattedTracksData{sceneId}{trackId}.class(1), 'pedestrian')
            pedData = formattedTracksData{sceneId}{trackId};
            N1 = length(pedData.waitTimeSteps);
            N2 = length(pedData.frame);
            if N1~=N2
                x=1;
            end
        end
 
    end
    
   allPedTracks = [tracks{sceneId}.pedCrossingTracks; tracks{sceneId}.pedNotCrossingTracks];
   N_pedTracks = length(allPedTracks);
   
   for trackNo = 1:N_pedTracks
       pedTrackId = allPedTracks(trackNo);
       pedData = formattedTracksData{sceneId}{pedTrackId};
        if ~isfield(pedData, 'isNearLane')
            x=1;
        end
   end
   
end
