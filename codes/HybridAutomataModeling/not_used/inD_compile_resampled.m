%% This script is the main script needed to compile the data from inD intersection dataset

% function [formattedTracksData] = inD_compile(formattedTracksData, Params, annotatedImageEnhanced, cw, reset)
% %function [allTracksMetaData, N_scenes, annotatedImage_enhanced, cw] = inD_compile(SampFreq, AdjustedSampFreq)


N_scenes = size(formattedTracksData,1);
skip_ts = double(Params.reSampleRate);
orthopxToMeter = Params.orthopxToMeter;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% loop starts
for scene_id = 1:N_scenes
    
    N_tracks = size(formattedTracksData{scene_id},1);

    % inner loop starts
    for track_id = 1:N_tracks


            % a) identify the hybrid state of the pedestrians and distance to closest CW (if the track is a
            % pedestrian track)
            % check if track is pedestrian
            flag.hybridStatePred = false;
            if strcmp(formattedTracksData{scene_id}{track_id,1}.class{1}, 'pedestrian')   
                [formattedTracksData{scene_id}{track_id,1}] = ....
                        hybridStateCopy_v3(formattedTracksData{scene_id}{track_id,1}, cw, flag, annotatedImageEnhanced, Params, 1, resetStates);    

                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % identify the cumulative wait times
%                 N_instances = size(formattedTracksData{scene_id}{track_id}.frame,1);
%                 wait_time_steps  = cumsum(formattedTracksData{scene_id}{track_id}.ProbHybridState(:,2));
%                 if (sum(wait_time_steps~=0)~=0)
%                     formattedTracksData{scene_id}{track_id}.waitTimeSteps = wait_time_steps*skip_ts;
%                 else
%                     formattedTracksData{scene_id}{track_id}.waitTimeSteps = -1*ones(N_instances,1);
%                 end
%                 if isfield(formattedTracksData{scene_id}{track_id}, 'wait_time_steps')
%                     formattedTracksData{scene_id}{track_id} = rmfield(formattedTracksData{scene_id}{track_id},'wait_time_steps');
%                 end
                
                % (b) this method resets the wait time (can later be added
                % to the HybridState function)
                % correct the wait times
                N_instances = size(formattedTracksData{scene_id}{track_id}.frame,1);
                wait_time_steps = -1;
                for time_step = 1:N_instances
                    if (strcmp(formattedTracksData{scene_id}{track_id}.HybridState(time_step), 'Wait') && wait_time_steps(end)~=-1 )
                        wait_time_steps = [wait_time_steps; wait_time_steps(end)+ reSampleRate];
                    elseif (strcmp(formattedTracksData{scene_id}{track_id}.HybridState(time_step), 'Wait') && wait_time_steps(end)==-1 )
                         wait_time_steps = [wait_time_steps; reSampleRate];
                    else
                         wait_time_steps =  [wait_time_steps; wait_time_steps(end)];
                    end
                    % reset wait after the wait has ended
                    if (time_step>1 && ~strcmp(formattedTracksData{scene_id}{track_id}.HybridState(time_step-1), 'Wait') && strcmp(formattedTracksData{sceneId}{trackId}.HybridState(time_step), 'Wait'))
                        wait_time_steps = [wait_time_steps; -1];
                    end
                end
                
                formattedTracksData{sceneId}{trackId}.waitTimeSteps = wait_time_steps(2:end);
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
            end

%             % b) identify the lane and the distance to the closest CW (if the track is a car track)  
%             if strcmp(formattedTracksData{scene_id}{track_id,1}.class{1}, 'car')  
%                 formattedTracksData{scene_id}{track_id}  = carState(formattedTracksData{scene_id}{track_id}, cw, resetStates, Params);
%             end


    end % inner loop ends for the image

% 
end % loop ends for all images

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% end % function end