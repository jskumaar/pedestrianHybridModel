%% wait time reset
reSampleRate = 5;
N_scenes = 12;
wait_time_steps = -1;
for sceneId = 1:N_scenes
    N_tracks = size(formattedTracksData{sceneId},1);
    for trackId = 1:N_tracks
       
        if strcmp(formattedTracksData{sceneId}{trackId,1}.class{1}, 'pedestrian')   
            wait_time_steps = -1;
            N_instances = size(formattedTracksData{sceneId}{trackId}.frame,1);
            for time_step = 1:N_instances
                if (strcmp(formattedTracksData{sceneId}{trackId}.HybridState(time_step), 'Wait') && wait_time_steps(end)~=-1 )
                    wait_time_steps = [wait_time_steps; wait_time_steps(end)+ reSampleRate];
                elseif (strcmp(formattedTracksData{sceneId}{trackId}.HybridState(time_step), 'Wait') && wait_time_steps(end)==-1 )
                     wait_time_steps = [wait_time_steps; reSampleRate];
                else
                     wait_time_steps =  [wait_time_steps; wait_time_steps(end)];
                end
                % reset wait after the wait has ended
                if (time_step>1 && ~strcmp(formattedTracksData{sceneId}{trackId}.HybridState(time_step-1), 'Wait') && strcmp(formattedTracksData{sceneId}{trackId}.HybridState(time_step), 'Wait'))
                    wait_time_steps = [wait_time_steps; -1];
                end
            end
            
            formattedTracksData{sceneId}{trackId}.waitTimeSteps = wait_time_steps(2:end);
            
        end
        
        
    end


end

