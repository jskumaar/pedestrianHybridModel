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

load('inD_trackDescriptives_v3.mat') 
% tracks = tracks_updated;


%% parameters
AdjustedSampFreq = 5;
observationWindow = 3 * AdjustedSampFreq; % 3s, i.e. 75 frames
rollOverWindow = 1 * AdjustedSampFreq; % 1s rolling window
recordingMetaData = readtable(strcat(num2str(18),'_recordingMeta.csv'));
orthopxToMeter = Params.orthopxToMeter;
%orthopxToMeter = recordingMetaData.orthoPxToMeter;
scale_down_factor = 12;
alpha = orthopxToMeter*scale_down_factor;
decZone = Params.decZone;

% initialize
N_scenes = length(formattedTracksData);
obs_index = 1;
CrossIntentData = table();

for scene_id = 1:N_scenes
    % pedestrian tracks
    all_ped_tracks = [tracks{scene_id}.pedCrossingTracks; tracks{scene_id}.pedNotCrossingTracks];   
    N_ped_tracks = size(all_ped_tracks, 1);
    
    % for all pedestrian tracks
    for track_index = 1:N_ped_tracks       
        ped_id = all_ped_tracks(track_index);
        
        % identify ground truth crossing intent
        if intersect(tracks{scene_id}.pedCrossingTracks, ped_id)
            cross_intent = true;
        else
            cross_intent = false;
        end
        %%%%%%%%%%%%%%%%        
        N_instances = size(formattedTracksData{scene_id}{ped_id}.frame, 1);      
        
        % initialize      
        for time_step = 1:N_instances
            
               veh_speed = [];
               veh_acc = [];
               
               car_id = formattedTracksData{scene_id}{ped_id}.closeCar_ind(time_step); 
               current_time = formattedTracksData{scene_id}{ped_id}.frame(time_step);
               % if there is a car closeby
               if car_id ~=0 && car_id ~=inf
                   car_timeStep = find(formattedTracksData{scene_id}{car_id}.frame == current_time);
               else
                   car_timeStep = [];
               end
              
               % minimum observation duration is reached
               if mod(time_step,rollOverWindow)==0 && time_step/observationWindow >= 1

                   % when there is a vehicle
                   if ~isempty(car_timeStep)
                        car_start_index = max(1, car_timeStep-observationWindow+1);

                        veh_speed =  formattedTracksData{scene_id}{car_id}.lonVelocity(car_start_index:car_timeStep);
                        veh_acc = formattedTracksData{scene_id}{car_id}.lonAcceleration(car_start_index:car_timeStep);
                   else
                       x=1;
                   end

                   % compile data (all the necessary data have been calculated
                   % before)
                   % only when pedestrian is close to a crosswalk


                   % copy features
                   if (formattedTracksData{scene_id}{ped_id}.closestCW(time_step)~=0 &&  formattedTracksData{scene_id}{ped_id}.closestCW(time_step)~=inf && abs(formattedTracksData{scene_id}{ped_id}.longDispPedCw(time_step)) < decZone )
                        
                           
                           gaze = formattedTracksData{scene_id}{ped_id}.isLooking(time_step-observationWindow+1 :time_step);
                           if (isfield(formattedTracksData{scene_id}{ped_id},'latDispPedCw'))
                                DTCurb = alpha*double(formattedTracksData{scene_id}{ped_id}.latDispPedCw(time_step-observationWindow+1 :time_step));
                           elseif (isfield(formattedTracksData{scene_id}{ped_id},'lat_disp_ped_cw'))        
                                DTCurb = alpha*double(formattedTracksData{scene_id}{ped_id}.lat_disp_ped_cw(time_step-observationWindow+1 :time_step));
                           end
                           if (isfield(formattedTracksData{scene_id}{ped_id},'longDispPedCw'))
                                DTCW = alpha*double(formattedTracksData{scene_id}{ped_id}.longDispPedCw(time_step-observationWindow+1 :time_step));
                           else
                                DTCW = alpha*double(formattedTracksData{scene_id}{ped_id}.long_disp_ped_cw(time_step-observationWindow+1 :time_step));
                           end

                           ped_speed = formattedTracksData{scene_id}{ped_id}.lonVelocity(time_step-observationWindow+1 :time_step);           
                           veh_ped_dist = alpha*double(formattedTracksData{scene_id}{ped_id}.long_disp_ped_car(time_step-observationWindow+1 :time_step));
                           ego_car_ind = find(veh_speed~=inf);
                           ped_ind = find(DTCurb~=inf & DTCW ~= inf);
                           
                           
                           % pedestrian tracks
                           if (~isempty(ped_ind))

                               CrossIntentData.mean_ped_speed(obs_index) = mean(ped_speed(ped_ind));
                               CrossIntentData.std_ped_speed(obs_index) = std(ped_speed(ped_ind));

                               CrossIntentData.mean_DTCurb(obs_index) = mean(DTCurb(ped_ind));
                               CrossIntentData.std_DTCurb(obs_index) = std(DTCurb(ped_ind));

                               CrossIntentData.mean_DTCW(obs_index) = mean(DTCW(ped_ind));
                               CrossIntentData.std_DTCW(obs_index) = std(DTCW(ped_ind));
                            end

                           CrossIntentData.scene_id(obs_index) = scene_id;
                           CrossIntentData.track_id(obs_index) = ped_id;
                           CrossIntentData.cross_intent(obs_index) = cross_intent;
                           
                           %% debug
                           if isempty(ego_car_ind) &&~isempty(ped_ind)
                               x=1;
                           end

                           % compile features
                           if ~isempty(ego_car_ind)

                               duration_ego_vehicle = length(ego_car_ind)/observationWindow;
                               
                               if isfield(formattedTracksData{scene_id}{ped_id}, 'isNearLane') 
                                    isNearLane = formattedTracksData{scene_id}{ped_id}.isNearLane(time_step-observationWindow+1 :time_step);
                               else
                                    isNearLane = false(observationWindow,1);
                               end
                                   
                               isSamedirection = formattedTracksData{scene_id}{ped_id}.isPedSameDirection(time_step-observationWindow+1 :time_step);

                               CrossIntentData.mean_veh_ped_dist(obs_index) = mean(veh_ped_dist(ego_car_ind));
                               CrossIntentData.std_veh_ped_dist(obs_index) = std(veh_ped_dist(ego_car_ind));

                               CrossIntentData.mean_veh_speed(obs_index) = mean(veh_speed(ego_car_ind));
                               CrossIntentData.std_veh_speed(obs_index) = std(veh_speed(ego_car_ind));                             

                               CrossIntentData.mean_veh_acc(obs_index) = mean(veh_acc(ego_car_ind));
                               CrossIntentData.std_veh_acc(obs_index) = std(veh_acc(ego_car_ind));                             

                               CrossIntentData.duration_ego_vehicle(obs_index) = duration_ego_vehicle;
                               CrossIntentData.gaze_ratio(obs_index) = sum(gaze(ego_car_ind))/length(ego_car_ind);

                               CrossIntentData.isNearLane(obs_index) = mean(isNearLane(end-AdjustedSampFreq+1:end))>0.5;
                               CrossIntentData.isSamedirection(obs_index) = mean(isSamedirection(end-AdjustedSampFreq+1:end))>0.5;

                               obs_index = obs_index + 1;
                           else
                               duration_ego_vehicle = 0;
                               CrossIntentData.duration_ego_vehicle(obs_index) = duration_ego_vehicle;
                               CrossIntentData.gaze_ratio(obs_index) = 0;
                               CrossIntentData.isNearLane(obs_index) = false;
                               obs_index = obs_index + 1;
                           end

                   end
                   
                    
                       
               end

               
           
        end
       
    end

    
end




% identify jaywalking pedestrians and remove that data
% load('inD_trackDescriptives_removed_ped_tracks.mat')
% tracks = tracks;
CrossIntentData_NoJayWalk = CrossIntentData;

for ii=1:height(CrossIntentData)
   if (any(tracks{CrossIntentData.scene_id(ii)}.pedJaywalkingTracks == CrossIntentData.track_id(ii) ))
      CrossIntentData_NoJayWalk(ii,:) = [];
   end    
end

%% separate cross intent data for with vehicles and without vehicles

CrossIntentData_withEgoCar = CrossIntentData;
CrossIntentData_withOutEgoCar = CrossIntentData;

ind_withEgoCar = find(CrossIntentData_withEgoCar.duration_ego_vehicle~=0);
ind_withOutEgoCar = find(CrossIntentData_withEgoCar.duration_ego_vehicle==0);

CrossIntentData_withEgoCar(ind_withOutEgoCar,:) = [];
CrossIntentData_withOutEgoCar(ind_withEgoCar,:) = [];



