%% This is the main file to run the various hybrid automaton models of individual 
% pedestrians interacting with a single automated vehicle

% 1) Discrete State Transitions - various probabilistic SVM models
% 2) Continuous State Transitions - constant velocity with KF and Gaussian
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

%% debug
clearvars -except resetStates annotatedImageEnhanced formattedTracksData tracks tracksMetaData cw Prob_CrossIntentModelCar Prob_CrossIntentModelNoCar Prob_GapAcceptanceModel Params

%% setup
% % a) addpath of necessary directories
% p1 = genpath('G:\My Drive\Research\Projects\pedestrianHybridModel\codes');
% p2 = genpath('G:\My Drive\Research\Projects\pedestrianHybridModel\datasets');
p1 = genpath('E:\pedestrianHybridModel\codes');
p2 = genpath('E:\pedestrianHybridModel\datasets');
addpath(p1)
addpath(p2)

% % b)load models
% % load the gap acceptance model
load('GapAcceptance_inD_9Features_FGaussianSVM_BootStrappedTwice.mat', 'GapAcceptance_inD_9Features_FGaussianSVM_BootStrappedTwice');
GapAcceptanceModel = GapAcceptance_inD_9Features_FGaussianSVM_BootStrappedTwice.ClassificationSVM;
Prob_GapAcceptanceModel = fitSVMPosterior(GapAcceptanceModel);
% load the crossing intent model
load('CrossIntent_inD_9Features_BS2_noDuration_MGaussianSVM_3s.mat');
CrossIntentModelCar = CrossIntent_inD_9Features_BS2_noDuration_MGaussianSVM_3s.ClassificationSVM;
Prob_CrossIntentModelCar = fitSVMPosterior(CrossIntentModelCar);
load('CrossIntent_NoCar_inD_9Features_BS1_noDuration_FGaussianSVM_3s.mat');
CrossIntentModelNoCar = CrossIntent_NoCar_inD_9Features_BS1_noDuration_FGaussianSVM_3s.ClassificationSVM;
Prob_CrossIntentModelNoCar = fitSVMPosterior(CrossIntentModelNoCar);

%c) parameters
dataset="inD";
HPed_params;

% d) Read data or compile data?
flag.dataCompile = false;
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% compile/data tracks data if flag.dataCompile
%     % a) Run the following if compiling data for the first time, else
%     run1b) 
if flag.dataCompile
    [formattedTracksData, allTracksMetaData, N_Scenes, annotatedImageEnhanced] = inD_compile(Params, reset);
    [tracks, ~] = trackDescriptives(formattedTracksData, N_scenes);

    for scene_id = 1:N_scenes
            tracksMetaData{scene_id}.ego_veh_gap_hist(1:size(tracksMetaData{scene_id},1))  = {zeros(20,1)};  % for inD dataset the maximum number of gaps
            tracksMetaData{scene_id}.wait_start_hist(1:size(tracksMetaData{scene_id},1))  = {zeros(20,1)}; 
    end

        % %check for ego-pedestrian and pedestrian gaze for the entire dataset
        egoPed_Gaze_HPed;

        % save the data file for later reuse
        save('tracksData_reSampled_correctDisCW_v2.mat','formattedTracksData','tracksMetaData','-v7.3','-nocompression')
        x = 1;
    
else
    
    % b) load already compiled tracks data
    load('tracksData_reSampled_correctDisCW_v7.mat')
    load('inD_trackDescriptives_removed_ped_tracks_v2.mat') 
    tracks =  tracksUpdated;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% For every time step in a scene for every scene, run the prediction and update loops; 
% the compiled tracks data serve as observations. This way the code can be
% directly reused for any simulations with minimal changes

%% debug
N_Scenes = 1;

% initialize variables
GapFeaturesAllScenes = {};
CrossFeaturesAllScenes = {};
predPedIndex = 0;
flag.pred = false; % this is to run the close CW function ('hybridState') w/o hybrid state update
% loop starts
for sceneId = 1:N_Scenes
    %initialize scene variables
    pedIndexWithinSceneHistory = [];
    trackTimeStep = 1;  % time loop of the scene  
    carMovingTracks = tracks{sceneId}.carMovingTracks;
    % assume every moving car track is an ego-AV
    for trackId = 1:length(carMovingTracks)
%     for trackId = 6:20
        % initialize trak variables
        carTrackId = carMovingTracks(trackId);
        carData = formattedTracksData{sceneId}{carTrackId};
        carHeading = 0;
        pedIndexInTrackHistory = inf;

        % track time loop starts
        for trackTimeStep = 1:height(carData)
            tic   % start timer here; before this data reading, from here the process is similar to how an AV would process its information
            trackTime = carData.frame(trackTimeStep);
            carPosPixels = double([carData.xCenterPix(trackTimeStep), carData.yCenterPix(trackTimeStep)]);
            carHeading = carData.calcHeading(trackTimeStep);
            AVStates.carPosPixels = carPosPixels;
            AVStates.carHeading = carHeading;
            
            %% run prediction of the ego-car is approaching a crosswalk, else run constant velocity predictions
            if (carData.closestCW(trackTimeStep)~=0 || carData.closestCW(trackTimeStep)~=inf)
               flag.EgoCar = true;
            else
               flag.EgoCar = false;
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
                for carLoopId = 1: length(activeCarWithinRange)
                    carIndex = activeCarWithinRange(carLoopId);
                    carTrackTimeStep = (trackTime - formattedTracksData{sceneId}{carIndex}.frame(1))/Params.reSampleRate + 1;                 
                    currentTSActiveCarData(carLoopId, :) = formattedTracksData{sceneId}{carIndex}(carTrackTimeStep, variablesToCopy);
                end
                currentTSActiveCarData.turn(1:length(activeCarWithinRange)) = false;
                currentTSActiveCarData.changeLane(1:length(activeCarWithinRange)) = false;
                currentTSActiveCarData.reachGoal(1:length(activeCarWithinRange)) = false;
                
                % For every active pedestrian run the trajectory prediction        
                for pedLoopId = 1: length(activePedWithinRange)  % for every active pedestrian during this time step

                    % initialize pedestrian variables and time step
                    pedIndexWithinScene = activePedWithinRange(pedLoopId);
                    currentPedData = formattedTracksData{sceneId}{pedIndexWithinScene};
                    currentPedMetaData = tracksMetaData{sceneId}(pedIndexWithinScene, :);            
                    pedTrackTimeStep = (trackTime - currentPedData.frame(1))/Params.reSampleRate + 1;

                    % check if it is an existing pedestrian or a new pedestrian           
                    if ~ismember(pedIndexWithinScene, pedIndexInTrackHistory)
                        pedIndexInTrackHistory = [pedIndexInTrackHistory; pedIndexWithinScene];
                        predPedIndex = predPedIndex + 1;  % this index keeps track of all pedestrians from all scenes               
                        % initialize the pedestrian
                        predictedPedTraj{sceneId}{trackId,1}{pedIndexWithinScene,1}.sceneId = sceneId;
                        predictedPedTraj{sceneId}{trackId,1}{pedIndexWithinScene,1}.carTrackId = carTrackId;
                        predictedPedTraj{sceneId}{trackId,1}{pedIndexWithinScene,1}.pedTrackId = pedIndexWithinScene;  
                        predictedPedTraj{sceneId}{trackId,1}{pedIndexWithinScene,1}.timeStep(1) = inf;
                        predictedPedTraj{sceneId}{trackId,1}{pedIndexWithinScene,1}.data{1} = inf;
                        predictedPedTraj{sceneId}{trackId,1}{pedIndexWithinScene,1}.kfData{1} = inf;
                    end

                    % 4) Run the H-Ped prediction framework if there is an ego
                    % car for the pedestrian, else run a constant velocity
                    % model
                    
                    % initialize the kalman filter
                    kf.x = [currentPedData.xCenter(pedTrackTimeStep);
                            currentPedData.yCenter(pedTrackTimeStep);
                            currentPedData.xVelocity(pedTrackTimeStep);
                            currentPedData.yVelocity(pedTrackTimeStep)];                        
                    kf.P = eye(4)*R(1,1);
                    kf.detP = det(kf.P); % Let's keep track of the noise by detP
                    
                    if flag.EgoCar
                        if (currentPedData.closeCar_ind(pedTrackTimeStep)~=0 && currentPedData.closeCar_ind(pedTrackTimeStep)~=inf)
                            crossCarData = formattedTracksData{sceneId}{currentPedData.closeCar_ind(pedTrackTimeStep)};
                        else
                            crossCarData = [];
                        end
                        [pedPredictions, pedKFpredictions, predGapFeatures, predCrossFeatures] = predictStates(kf, currentPedData, currentPedMetaData, currentTSActiveCarData, crossCarData, AVStates, pedTrackTimeStep, ...
                                                                           cw, annotatedImageEnhanced, resetStates, Prob_GapAcceptanceModel, Prob_CrossIntentModelCar,Prob_CrossIntentModelNoCar, Params, flag);
                        GapFeaturesAllScenes = [GapFeaturesAllScenes; predGapFeatures];
                        CrossFeaturesAllScenes = [CrossFeaturesAllScenes; predCrossFeatures];
                    else
                        [pedPredictions, pedKFpredictions] = HPed_CV(kf, currentPedData, Params, pedTrackTimeStep);
                    end
                    
                    % 5) Save the predictions
                    if pedTrackTimeStep >= Params.AdjustedSampFreq
                        predictedPedTraj{sceneId}{trackId,1}{pedIndexWithinScene,1}.timeStep(end+1,1) = pedTrackTimeStep;
                        predictedPedTraj{sceneId}{trackId,1}{pedIndexWithinScene,1}.data{end+1,1} = pedPredictions;
                        predictedPedTraj{sceneId}{trackId,1}{pedIndexWithinScene,1}.kfData{end+1,1} = pedKFpredictions;
                    end
                    
                    x=1;
                end % end of all pedestrians
                     
            end  

            
        end  % end of time loop for this scene
    
    
    end  

    
end % end of all scenes