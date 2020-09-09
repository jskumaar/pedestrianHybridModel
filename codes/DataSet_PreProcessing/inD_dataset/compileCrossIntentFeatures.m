% %% This script compiles the features for crossing intent prediction during the Approach region
% 
% % Steps:
% % 1) Specify an observation window and the rolling window
% % 2) Identify the indices of 'Approach'
% % 3) For each observation, compile the observations for 3s (store it
% % separately) maybe even write in an excel file
% % 4) Features to collect: (i) pedestrian gaze, (ii) pedestrian speed, (iii)
% % pedestrian distance to the curb, (iv) pedestrian distance to the
% % crosswalk, (v) ego-vehicle speed, (vi) ego-vehicle distance
% 
% 
%% load compiled data
% load('tracksData_compiled.mat');

% %% addpath of necessary directories
p1 = genpath('G:\My Drive\Research\Projects\pedestrianHybridModel\codes');
p2 = genpath('G:\My Drive\Research\Projects\pedestrianHybridModel\datasets');

addpath(p1)
addpath(p2)




%% parameters
AdjustedSampFreq = 5;
observationWindow = 3 * AdjustedSampFreq; % 3s, i.e. 75 frames
rollOverWindow = 1 * AdjustedSampFreq; % 1s rolling window
recordingMetaData = readtable(strcat(num2str(18),'_recordingMeta.csv'));
orthopxToMeter = Params.orthopxToMeter;
%orthopxToMeter = recordingMetaData.orthoPxToMeter;
scale_down_factor = 12;

alpha = orthopxToMeter*scale_down_factor;


veh_speed = [];
veh_acc = [];
N_scenes = size(formattedTracksData,2);
obs_index = 1;
CrossIntentData = table();
tracks_updated = tracks;

for scene_id = 1:N_scenes
   
    all_ped_tracks = [tracks_updated{scene_id}.ped_crossing_tracks; tracks_updated{scene_id}.ped_not_crossing_tracks];   
    N_ped_tracks = size(all_ped_tracks, 1);
    
    for track_index = 1:N_ped_tracks       
        ped_id = all_ped_tracks(track_index);
        
        if intersect(tracks_updated{scene_id}.ped_crossing_tracks, ped_id)
            cross_intent = true;
        else
            cross_intent = false;
        end
                  
        N_instances = size(formattedTracksData{scene_id}{ped_id}, 1);      
        
        for time_step = 1:N_instances          
           car_id = formattedTracksData{scene_id}{ped_id}.closeCar_ind(time_step); 
           
           if car_id ~=0 && car_id ~=inf
               ped_ts = formattedTracksData{scene_id}{ped_id}.frame(time_step);
               car_ts = find(formattedTracksData{scene_id}{car_id}.frame == ped_ts);
           end
           
           if ~isempty(car_ts)           
                veh_speed =  [veh_speed; formattedTracksData{scene_id}{car_id}.lonVelocity(car_ts) ];
                veh_acc = [veh_acc; formattedTracksData{scene_id}{car_id}.lonAcceleration(car_ts) ];
           else
                veh_speed =  [veh_speed; inf ];
                veh_acc =  [veh_acc; inf ];
           end
           
           % compile data (all the necessary data have been calculated
           % before)
           if mod(time_step,rollOverWindow)==0 && time_step/observationWindow >= 1
               
               % copy features               
               gaze = formattedTracksData{scene_id}{ped_id}.isLooking(time_step-observationWindow+1 :time_step);
               DTCurb = alpha*double(formattedTracksData{scene_id}{ped_id}.lat_disp_ped_cw(time_step-observationWindow+1 :time_step));
               DTCW = alpha*double(formattedTracksData{scene_id}{ped_id}.long_disp_ped_cw(time_step-observationWindow+1 :time_step));
               ped_speed = formattedTracksData{scene_id}{ped_id}.lonVelocity(time_step-observationWindow+1 :time_step);           
               veh_ped_dist = alpha*double(formattedTracksData{scene_id}{ped_id}.long_disp_ped_car(time_step-observationWindow+1 :time_step));
               isSamedirection = formattedTracksData{scene_id}{ped_id}.isPedSameDirection(time_step-observationWindow+1 :time_step);
               ego_car_ind = find(veh_speed~=inf);
               ped_ind = find(DTCurb~=inf & DTCW ~= inf);
        

            if ~isempty(ped_ind)
               CrossIntentData.mean_ped_speed(obs_index) = mean(ped_speed(ped_ind));
               CrossIntentData.std_ped_speed(obs_index) = std(ped_speed(ped_ind));

               CrossIntentData.mean_DTCurb(obs_index) = mean(DTCurb(ped_ind));
               CrossIntentData.std_DTCurb(obs_index) = std(DTCurb(ped_ind));

               CrossIntentData.mean_DTCW(obs_index) = mean(DTCW(ped_ind));
               CrossIntentData.std_DTCW(obs_index) = std(DTCW(ped_ind));
               
               CrossIntentData.direction(obs_index) = mean(isSamedirection(ped_ind))>0.5;

               if isnan(CrossIntentData.mean_DTCurb(obs_index))
                   x=1;
               end
            end
               
               CrossIntentData.scene_id(obs_index) = scene_id;
               CrossIntentData.track_id(obs_index) = ped_id;

               CrossIntentData.cross_intent(obs_index) = cross_intent;

               
               % compile features
               if ~isempty(ego_car_ind)
               
                   duration_ego_vehicle = length(ego_car_ind)/observationWindow;
                   isNearLane = formattedTracksData{scene_id}{ped_id}.isNearLane(time_step-observationWindow+1 :time_step);
              
                   CrossIntentData.mean_veh_ped_dist(obs_index) = mean(veh_ped_dist(ego_car_ind));
                   CrossIntentData.std_veh_ped_dist(obs_index) = std(veh_ped_dist(ego_car_ind));

                   CrossIntentData.mean_veh_speed(obs_index) = mean(veh_speed(ego_car_ind));
                   CrossIntentData.std_veh_speed(obs_index) = std(veh_speed(ego_car_ind));                             

                   CrossIntentData.mean_veh_acc(obs_index) = mean(veh_acc(ego_car_ind));
                   CrossIntentData.std_veh_acc(obs_index) = std(veh_acc(ego_car_ind));                             

                   CrossIntentData.duration_ego_vehicle(obs_index) = duration_ego_vehicle;
                   CrossIntentData.gaze_ratio(obs_index) = sum(gaze(ego_car_ind))/length(ego_car_ind);

                   CrossIntentData.isNearLane(obs_index) = mean(isNearLane(end-AdjustedSampFreq+1:end))>0.5;

                   obs_index = obs_index + 1;
               else
                   duration_ego_vehicle = 0;
                   CrossIntentData.duration_ego_vehicle(obs_index) = duration_ego_vehicle;
                   CrossIntentData.gaze_ratio(obs_index) = 0;
                   CrossIntentData.isNearLane(obs_index) = false;
                   obs_index = obs_index + 1;
               end
               
               % to update the next set of observation
               veh_speed(1:rollOverWindow) = [] ;
               veh_acc(1:rollOverWindow)  = [];
               
           end
           
           % reset
           
           ped_ts = [];
           car_ts = [];

        end
         % reset
           
           veh_speed = [];
           veh_acc = [];

    end

    
end




% identify jaywalking pedestrians and remove that data
load('inD_trackDescriptives_removed_ped_tracks.mat')
tracks = tracks_updated;
CrossIntentData_NoJayWalk = CrossIntentData;

for ii=1:height(CrossIntentData)
   if (any(tracks{CrossIntentData.scene_id(ii)}.ped_jaywalking_tracks == CrossIntentData.track_id(ii) ))
      CrossIntentData_NoJayWalk(ii,:) = [];
   end    
end

%% separate cross intent data for with vehicles and without vehicles

CrossIntentData_withEgoCar = CrossIntentData;
CrossIntentData_withOutEgoCar = CrossIntentData;

ind_withEgoCar = find(CrossIntentData_withEgoCar.duration_ego_vehicle~=0);
ind_withOutEgoCar = find(CrossIntentData_withEgoCar.duration_ego_vehicle==0);

CrossIntentData_withEgoCar(ind_withEgoCar,:) = [];
CrossIntentData_withOutEgoCar(ind_withOutEgoCar,:) = [];



