for sceneId = 1:12
    N_tracks = size(formattedTracksData{sceneId},1);
    for track_id = 1:N_tracks
        if strcmp(formattedTracksData{sceneId}{track_id}.class(1), 'pedestrian')
            trackData = formattedTracksData{sceneId}{track_id};
            N_ts = length(trackData.frame);
            crossStart = [];
            crossedCW = [];
            for ii=2:N_ts
                if (strcmp(trackData.HybridState(ii),'Crossing')||strcmp(trackData.HybridState(ii),'Jaywalking'))&& (~strcmp(trackData.HybridState(ii-1),'Crossing')&&~strcmp(trackData.HybridState(ii),'Jaywalking'))
                    crossStart = [crossStart; ii];
                end
                
                if trackData.longDispPedCw(ii)<0 && trackData.longDispPedCw(ii-1)>0 
                    crossedCW  = [crossedCW; ii];
                end
                
            end
            crossStartTimeSteps{sceneId}{track_id} = crossStart;
            crossedCWTimeSteps{sceneId}{track_id} = crossedCW;
 
        end
    end
end