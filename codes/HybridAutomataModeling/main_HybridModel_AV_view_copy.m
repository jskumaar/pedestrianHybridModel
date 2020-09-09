%% This is the main file to run the various hybrid automaton models of individual 
% pedestrians interacting with a single automated vehicle

% 1) Discrete State Transitions - various probabilistic SVM models
% 2) Contunous State Transitions - constant velocity with KF and Gaussian
% Noise
% 3) A Bayesian Inference framework is used for predicting and updating the
% states of the hybrid model
% 3a) For the discrete states the predictions occur through (i) the intent
% prediction model and (ii) gap acceptance model
% 3b) For the discrete states the update occurs through the actual hybrid
% state (pre-calculated in this dataset, but in reality will use the guard states)
% 3c) For the continuous states the predictions occur through a constant
% velocity model
% 3d) For the continuous states, the update occurs the trajectory
% observations provided through the dataset

%% Updated: 08/25/20
clearvars -except annotatedImage_enhanced formattedTracksData tracks tracksMetaData tracks cw Prob_CrossIntentModel Prob_GapAcceptanceModel Params

%% setup
% a) addpath of necessary directories
p1 = genpath('G:\My Drive\Research\Projects\pedestrianHybridModel\codes');
p2 = genpath('G:\My Drive\Research\Projects\pedestrianHybridModel\datasets');
addpath(p1)
addpath(p2)

% b)load models
%load the gap acceptance model
% load('GapAcceptance_inD_6Features_noGaze_noTimeGap_FGaussianSVM_2300RejGaps_BootStrapped.mat', 'GapAcceptance_inD_6Features_noGaze_noTimeGap_GaussianSVM');
% GapAcceptanceModel = GapAcceptance_inD_6Features_noGaze_noTimeGap_GaussianSVM.ClassificationSVM;
% Prob_GapAcceptanceModel = fitSVMPosterior(GapAcceptanceModel);
% % load the crossing intent model
% load('CrossIntent_inD_6Features_onlyMean_noVehAcc_GaussianSVM_3s.mat');
% CrossIntentModel = CrossIntent_inD_6Features_onlyMean_noVehAcc_GaussianSVM.ClassificationSVM;
% Prob_CrossIntentModel = fitSVMPosterior(CrossIntentModel);
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
recordingMetaData = readtable(strcat(num2str(29),'_recordingMeta.csv'));
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% %c) parameters
% % for inD dataset
% inD_params;

% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 1a) Run the following if compiling data for the first time, else run 1b)
[formattedTracksData, tracksMetaData, N_scenes, annotatedImage_enhanced] = inD_compile(Params); 
% [tracksMetaData, N_scenes, annotatedImage_enhanced, cw] = inD_compile(Params.reSampleRate); 
% [tracks, ~] = trackDescriptives(formattedTracksData, N_scenes);
load('inD_trackDescriptives_removed_ped_tracks.mat')
tracks = tracks_updated;

for scene_id = 1:N_scenes
    tracksMetaData{scene_id}.ego_veh_gap_hist(1:size(tracksMetaData{scene_id},1)) = {zeros(20,1)};  % for inD dataset the maximum number of gaps for a pedestrian track is '13'
    tracksMetaData{scene_id}.wait_start_hist(1:size(tracksMetaData{scene_id},1)) = inf;  % when pedestrians start to wait
end


% %check for ego-pedestrian and pedestrian gaze for the entire dataset
egoPed_Gaze_HPed;

% save the data file for later reuse
save('tracksData_reSampled_correctDisCW_v4.mat','formattedTracksData','tracksMetaData','-v7.3','-nocompression')
x = 1;

%% 1b) load already compiled tracks data 
% load formatted data
% load('tracksData_reSampled_correctDisCW_v2.mat')
% load('inD_trackDescriptives_removed_ped_tracks.mat')
% tracks = tracks_updated;
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % load the reset/goal locations of the dataset
[reset] = goalsinD();

%% For every time step in a scene for every scene, run the Bayesian predict and Bayesian update loops

% initialize loop variables
pred_ped_index = 0;
flag.pred = false; % this is to run the close CW function ('hybridState') w/o hybrid state update
% loop starts
for scene_id = 1:N_scenes
    %initialize scene variables
    ped_index_within_scene_history = [];
    track_time_step = 1;  % time loop of the scene  
    car_moving_tracks = tracks{scene_id}.car_moving_tracks;
    % assume every moving car track is an ego-AV
    for track_id = 1:length(car_moving_tracks)
        % initialize trak variables
        car_track_id = car_moving_tracks(track_id);
        carData = formattedTracksData{scene_id}{car_track_id};
        car_heading = 0;
        ped_index_in_track_history = inf;

        % track time loop starts
        for track_time_step = 1:height(carData)
            tic   % start timer here; before this data reading, from here the process is similar to how an AV would process its information
            track_time = carData.frame(track_time_step);
            carPosPixels = [carData.xCenterPix(track_time_step), carData.yCenterPix(track_time_step)];
            y_vel = carData.yVelocity(track_time_step);
            if ( abs(y_vel) < Params.moving_threshold )
                y_vel = 0;
            end
            x_vel = carData.xVelocity(track_time_step);
            if ( abs(x_vel) < Params.moving_threshold )
                x_vel = 0;
            end
            % if the car is stopped, maintain the previous heading
            if x_vel~=0 || y_vel~=0 
                car_heading = atan2(y_vel, x_vel)*180/pi;
            end
            
            %% find the agents withing the sensing range of the AV
            activeAgentsWithinRange;
            
            %% run the prediction if there is a pedestrian within the range
            if ~isempty(activePedWithinRange)
                % Update the current data of all active cars
                % 10 - xVelocity, 11- yVelocity, 12- xAcceleration, 13 - yAcceleration, 14-lonVelocity, 16-lonAcceleration, 17 - latAcceleration, 
                % 18 - xCenterPix, 19 - yCenterPix, 21- closestCW,4 23-
                % calcHeading, 24 - car lane
                currentTSActiveCarData = table();
                variablesToCopy = [10, 11, 12, 13, 14, 16, 17, 18, 19, 21, 23, 24];
                % compile all cars' states within the sensing range
                for car_loop_id = 1: length(activeCarWithinRange)
                    car_index = activeCarWithinRange(car_loop_id);
                    if car_loop_id > 1
                        x=1;
                    end
                    car_track_time_step = (track_time - formattedTracksData{scene_id}{car_index}.frame(1))/Params.reSampleRate + 1;                 
                    currentTSActiveCarData(car_loop_id, :) = formattedTracksData{scene_id}{car_index}(car_track_time_step, variablesToCopy);
                end
                currentTSActiveCarData.Turn(1:length(activeCarWithinRange)) = false;
                currentTSActiveCarData.changeLane(1:length(activeCarWithinRange)) = false;
                currentTSActiveCarData.reachGoal(1:length(activeCarWithinRange)) = false;
                
                % For every active pedestrian run the trajectory prediction        
                for ped_loop_id = 1: length(activePedWithinRange)  % for every active pedestrian during this time step

                    % initialize pedestrian variables and time step
                    ped_index_within_scene = activePedWithinRange(ped_loop_id);
                    currentPedData = formattedTracksData{scene_id}{ped_index_within_scene};
                    currentPedMetaData = tracksMetaData{scene_id}(ped_index_within_scene, :);            
                    flag.EgoCar = false;  %flag for the presence of an ego-car; default is false
                    ped_track_time_step = (track_time - currentPedData.frame(1))/Params.reSampleRate + 1;

                    % check if it is an existing pedestrian or a new pedestrian           
                    if ~ismember(ped_index_within_scene, ped_index_in_track_history)
                        ped_index_in_track_history = [ped_index_in_track_history; ped_index_within_scene];
                        pred_ped_index = pred_ped_index + 1;  % this index keeps track of all pedestrians from all scenes               
                        % initialize the pedestrian
                        predicted_ped_traj{pred_ped_index}.scene_id = scene_id;
                        predicted_ped_traj{pred_ped_index}.car_track_id = car_track_id;
                        predicted_ped_traj{pred_ped_index}.ped_track_id = ped_index_within_scene;                        
                    end

                    % 4) Run the prediction framework          
                    [pedPredictions, pred_GapFeatures] = predictStates(currentPedData, currentPedMetaData, currentTSActiveCarData, ped_track_time_step, cw, annotatedImage_enhanced, reset,...
                                                     Prob_GapAcceptanceModel, Prob_CrossIntentModel, Params);
                    GapFeatures_allScenes = [GapFeatures_allScenes; pred_GapFeatures];
                    % 5) Save the predictions
                    predicted_ped_traj{pred_ped_index,1}.data{track_time_step,1} = pedPredictions;
                    
                    
                    % 6) Save the actual values
                    predicted_ped_traj{pred_ped_index,1}.GTdata(track_time_step, :) = [currentPedData.xCenterPix(ped_track_time_step), currentPedData.yCenterPix(ped_track_time_step)];
                    
                end  % end of all pedestrians

            end
        end  % end of time loop for this scene
    
    
    end  

    
end % end of all scenes