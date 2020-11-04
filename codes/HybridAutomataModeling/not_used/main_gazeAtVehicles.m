%% This script calculates the ego-vehicle and the gaze; run after compilation the hybrid state information

% parameters
SampFreq = 25;
skip_ts = 5;

for scene_id = 1:N_scenes
    % exclude jaywalking pedestrians
    ped_tracks = tracks{scene_id}.ped_tracks;
    ped_jaywalking_tracks = tracks{scene_id}.ped_jaywalking_tracks;
    [~,ind,~] = intersect(ped_tracks, ped_jaywalking_tracks);
    ped_tracks(ind) = [];
    
    for  ped_index = 1:length(ped_tracks)      
        ped_id = ped_tracks(ped_index);
        currentPedData = formattedTracksData{scene_id}{ped_id};
        %initialize variables
        currentTSPedEgoData = table();     
        loop_time = 1;
        % Find the time instances of the ped tracks
        scene_start_time = formattedTracksData{scene_id}{ped_id}.frame(1);
        scene_end_time = formattedTracksData{scene_id}{ped_id}.frame(end);

        % time loop of the scene
        for scene_time = scene_start_time : skip_ts : scene_end_time
            % Find the active car and pedestrian tracks for this time step
            activeTracks = find(tracksMetaData{scene_id}.finalFrame >= scene_time &... 
                                tracksMetaData{scene_id}.initialFrame <= scene_time);  
            activeCarTracks = intersect(tracks{scene_id}.car_moving_tracks, activeTracks);
            activePedTracks = intersect(tracks{scene_id}.ped_tracks, activeTracks);

            % when there is atleast one active car in the scene, check if
            % it is the ego-vehicle
            if ~isempty(activeCarTracks) 
                % compile current state of all active cars at this timestep
                currentTSActiveCarData = table(); % initialize
                for car_loop_id = 1: length(activeCarTracks)
                    car_index = activeCarTracks(car_loop_id);
                    car_track_time_step = find(formattedTracksData{scene_id}{car_index}.frame == scene_time); % the tracksMetaData frames start from '0' and are 1 value lower than 'scene_time'.
                    currentTSActiveCarData(car_loop_id,:) = formattedTracksData{scene_id}{car_index}(car_track_time_step, :);
                end

                % Pedestrian data for an observaton window; this is done within the prediction loop as the ego-car can change within the prediction loop           
                if  loop_time > SampFreq % i.e. if there is data for over 1 s           
                    currentTSPedData = currentPedData(loop_time-SampFreq + 1: loop_time, :);
                else
                    currentTSPedData = currentPedData(1: loop_time, :);
                end
                % find the closet vehicle and distances to it
                currentTSPedEgoData = egoCarFunc(currentTSPedData, currentTSActiveCarData, cw, annotatedImage_enhanced);
                currentPedData.isLookingVehicles(loop_time) = currentTSPedEgoData.isLooking;
            end
            
            % update the loop time irrespective if there is an ego-vehicle
            % or not, so that it remains consistent with the track size
            loop_time = loop_time + 1; 
            
        end
         % copy 
         formattedTracksData{scene_id}{ped_id} = currentPedData;
        
        
    end
    
    
    
end
        