for sceneId = 1:12
    N_tracks = size(formattedTracksData{sceneId},1);
    for trackId = 1:N_tracks
       if strcmp(formattedTracksData{sceneId}{trackId}.class{1}, 'pedestrian')    
            if isfield(formattedTracksData{sceneId}{trackId},'calcLonVelocity')
                formattedTracksData{sceneId}{trackId}.calcLonVelocity = formattedTracksData{sceneId}{trackId}.calcLonVelocity(:,1);
                formattedTracksData{sceneId}{trackId}.swInd = formattedTracksData{sceneId}{trackId}.swInd(:,1);
            end
       end
    end
end