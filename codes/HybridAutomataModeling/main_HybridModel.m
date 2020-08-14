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
% 3d) For the continuou states, the update occurs the trajectory
% observations provided through the dataset

%% Updated: 07/28/20

clearvars -except formattedTracksData tracks_updated tracksMetaData tracks cw

% %% addpath of necessary directories
p1 = genpath('G:\My Drive\Research\1.Projects\pedestrianHybridModel\codes');
p2 = genpath('G:\My Drive\Research\1.Projects\pedestrianHybridModel\datasets');

addpath(p1)
addpath(p2)

% load the gap acceptance model
load('GapAcceptance_inD_6Features_noGaze_noTimeGap_FGaussianSVM_2300RejGaps_BootStrapped.mat', 'GapAcceptance_inD_6Features_noGaze_noTimeGap_GaussianSVM');
GapAcceptanceModel = GapAcceptance_inD_6Features_noGaze_noTimeGap_GaussianSVM.ClassificationSVM;
Prob_GapAcceptanceModel = fitSVMPosterior(GapAcceptanceModel);
% Load the crossing intent model
load('CrossIntent_inD_6Features_onlyMean_noVehAcc_GaussianSVM_3s.mat');
CrossIntentModel = CrossIntent_inD_6Features_onlyMean_noVehAcc_GaussianSVM_3s.classificationSVM;
Prob_CrossIntentModel = fitSVMPosterior(CrossIntentModel);
% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % 
%% Step 1: get the individual tracks of all agents for all scenes
% for inD dataset
delta_T = 0.04;   % sampling rate is 25 Hz, 0.04 seconds for each data point
SampFreq = int32(1/delta_T);
AdjustedSampFreq = 5;

reSampleRate = SampFreq/AdjustedSampFreq;
pred_horizon = 3*AdjustedSampFreq;      % 3 s
rollOverWindow = 1*AdjustedSampFreq;  % in time steps; 1s
observationWindow = 3*AdjustedSampFreq; % in time stepsl 3s

% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% [formattedTracksData, tracksMetaData, N_scenes, annotatedImage_enhanced, orthopxToMeter, cw] = inD_compile(SampFreq, AdjustedSampFreq); 
[N_scenes, annotatedImage_enhanced, cw] = inD_compile(SampFreq, AdjustedSampFreq); 
% [tracks, ~] = trackDescriptives(formattedTracksData, N_scenes);
% for scene_id = 1:N_scenes
%     tracksMetaData{scene_id}.ego_veh_gap_hist(1:size(tracksMetaData{scene_id},1)) = {zeros(20,1)};  % for inD dataset the maximum number of gaps for a pedestrian track is '13'
%     tracksMetaData{scene_id}.wait_start_hist(1:size(tracksMetaData{scene_id},1)) = inf;  % when pedestrians start to wait
% end
% 
% % (optional) compile the ego-vehicle as a whole
% formattedTracksData = tracksData;
% egoCar_v2;
% 
% %% compile the gaze
% main_gazeAtVehicles;
% 
% % save the data file for later reuse
% save('tracksData_reSampled_compiled_all_ped_tracks.mat','formattedTracksData','tracksMetaData','-v7.3','-nocompression')

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% load formatted data
% load('tracksData_reSampled_compiled_all_ped_tracks.mat')

% load the reset/goal locations of the dataset
[reset] = goalsinD();
predicted_ped_traj = table();

%% Step 2: For every time step in a scene for every scene, run the Bayesian predict and Bayesian update loops

% initialize variables
pred_ped_index = 0;
% loop starts
for scene_id = 1:N_scenes
    % Find the time instances of the scenes
    scene_start_time = min(tracksMetaData{scene_id}.initialFrame(1));
    scene_end_time = max(tracksMetaData{scene_id}.finalFrame(end));
    % Find the time instances for the resampled scenes
    scene_start_time = scene_start_time + mod(scene_start_time, reSampleRate);
    scene_end_time = scene_end_time + mod(scene_end_time, reSampleRate);
    
    %initialize variables
    ped_index_within_scene_history = [];
    scene_time_step = 1;  % time loop of the scene

    % scene time loop starts
    for scene_time = scene_start_time : reSampleRate : scene_end_time
        % Find the active car and pedestrian tracks for this time step
        activeTracks = find(tracksMetaData{scene_id}.finalFrame >= scene_time &... 
                            tracksMetaData{scene_id}.initialFrame <= scene_time);  
        activeCarTracks = intersect(tracks{scene_id}.car_moving_tracks, activeTracks);
        activePedTracks = intersect(tracks{scene_id}.ped_tracks, activeTracks);
               
        % For every active pedestrian run the trajectory prediction        
        for ped_loop_id = 1: length(activePedTracks)  % for every active pedestrian during this time step
            
            % initialize pedestrian variables and time step
            ped_index_within_scene = activePedTracks(ped_loop_id);
            currentPedData = formattedTracksData{scene_id}{ped_index_within_scene};
            currentPedMetaData = tracksMetaData{scene_id}(ped_index_within_scene, :);            
            flag.EgoCar = false;  %flag for the presence of an ego-car; default is false
            ped_track_time_step = (scene_time - currentPedData.frame(1))/reSampleRate + 1;
            
            % check if it is an existing pedestrian or a new pedestrian           
            if ~ismember(ped_index_within_scene, ped_index_within_scene_history)
                ped_index_within_scene_history = [ped_index_within_scene_history; ped_index_within_scene];
                pred_ped_index = pred_ped_index + 1;  % this index keeps track of all pedestrians from all scenes               
                % initialize the pedestrian
                predicted_ped_traj.scene_id(pred_ped_index) = scene_id;
                predicted_ped_traj.track_id(pred_ped_index) = ped_index_within_scene;
            end
                      
            % Update the current data of all active cars
            % 10 - xVelocity, 11- yVelocity, 14-lonVelocity, 16-lonAcceleration, 18 - xCenterPix, 19 - yCenterPix, 21- closestCW,
            % 23- calcHeading, 24 - car lane
            variablesToCopy = [10, 11, 14, 16, 18, 19, 21, 23, 24];
            for car_loop_id = 1: length(activeCarTracks)
                car_index = activeCarTracks(car_loop_id);
                car_track_time_step = (scene_time - formattedTracksData{scene_id}{car_index}.frame(1))/reSampleRate + 1;                 
                currentTSActiveCarData(car_loop_id,:) = formattedTracksData{scene_id}{car_index}(car_track_time_step, variablesToCopy);
                currentTSActiveCarData.Turn(car_loop_id) = false;
                currentTSActiveCarData.changeLane(car_loop_id) = false;
                currentTSActiveCarData.reachGoal(car_loop_id) = false;
            end

            % 4) Run the prediction framework
            pedPredictions = predictStates(currentPedData, currentPedMetaData, currentTSActiveCarData, ped_track_time_step, pred_horizon, AdjustedSampFreq, cw, annotatedImage_enhanced, reset,...
                                           Prob_GapAcceptanceModel, Prob_CrossingIntentModel);
            
            % 5) Save the predictions
            predicted_ped_traj.data{pred_ped_index}{scene_time_step} = pedPredictions;
       
        end  % end of all pedestrians

        % update scene time step
        scene_time_step = scene_time_step + 1;
    end  % end of time loop for this scene
    
end  % end of all scenes
