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

%% debug
clearvars -except resetStates annotatedImageEnhanced formattedTracksData tracks tracksMetaData cw Prob_CrossIntentModelCar Prob_CrossIntentModelNoCar Prob_GapAcceptanceModel Params

%% setup
% % a) addpath of necessary directories
% p1 = genpath('G:\My Drive\Research\Projects\pedestrianHybridModel\codes');
% p2 = genpath('G:\My Drive\Research\Projects\pedestrianHybridModel\datasets');
% p3 = genpath('G:\My Drive\Research\Projects\pedestrianHybridModel\results');
p1 = genpath('E:\jskumaar\pedestrianHybridModel\codes');
p2 = genpath('E:\jskumaar\pedestrianHybridModel\datasets');
addpath(p1)
addpath(p2)
% addpath(p3)

% % b)load models
% load the gap acceptance model
% load('GapAcceptance_inD_8Features_FGaussianSVM_BootStrappedTwice_v2.mat', 'GapAcceptance_inD_8Features_FGaussianSVM_BootStrappedTwice_v2');
% GapAcceptanceModel = GapAcceptance_inD_8Features_FGaussianSVM_BootStrappedTwice_v2.ClassificationSVM;
% Prob_GapAcceptanceModel = fitSVMPosterior(GapAcceptanceModel);
% load the crossing intent model
% load('CrossIntent_inD_9Features_BS1_noDuration_FGaussianSVM_3s_v2.mat');
% CrossIntentModelCar = CrossIntent_inD_9Features_BS2_noDuration_FGaussianSVM_3s_v2.ClassificationSVM;
% Prob_CrossIntentModelCar = fitSVMPosterior(CrossIntentModelCar);
% load('CrossIntent_NoCar_inD_3Features_BS1_noDuration_FGaussianSVM_v3.mat');
% CrossIntentModelNoCar = CrossIntent_NoCar_inD_3Features_BS1_noDuration_FGaussianSVM_v3.ClassificationSVM;
% Prob_CrossIntentModelNoCar = fitSVMPosterior(CrossIntentModelNoCar);
% read tracks MetaData
for jj=1:12
    sceneId = 17+jj;
    tracksMetaData{jj} = readtable(strcat(num2str(sceneId),'_tracksMeta.csv')) ;
end

%c) parameters
dataset="inD";
HPed_params;
% 
% % d) Read data or compile data?
% flag.dataCompile = false;
% % % flag.dataCompile = true;
% % % %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % compile/data tracks data if flag.dataCompile is set to true
% % a) Run the following if compiling data for the first time, else
% %     run1b) 
% if flag.dataCompile
% %     % for full data compilation
% %     [formattedTracksData, allTracksMetaData, N_Scenes] = inD_compile(Params, resetStates);
% %     [tracks, ~] = trackDescriptives(formattedTracksData, N_scenes);
% %     %%%%%%%%%%%%%%%%
%     % for recompiling the data from earlier resampled data
%     load('tracksData_reSampled_v11.mat')     
%     load('inD_trackDescriptives_v3.mat') 
%     %%%%%%%%%%%%%%%%%
%     % hybrid state
%     inD_compile_resampled;
%     %check for ego-pedestrian and pedestrian gaze for the entire dataset
%     egoPed_Gaze_HPed_v2;
%     %check pedestrian lane
%     tmp_nearLaneCalc;
%     
% %     for scene_id = 1:N_scenes
% %         tracksMetaData{scene_id}.ego_veh_gap_hist(1:size(tracksMetaData{scene_id},1))  = {zeros(20,1)};  % for inD dataset the maximum number of gaps
% %         tracksMetaData{scene_id}.wait_start_hist(1:size(tracksMetaData{scene_id},1))  = {zeros(20,1)}; 
% %     end
% 
%     % save the data file for later reuse
%     save('tracksData_reSampled_v11.mat','formattedTracksData','tracksMetaData','-v7.3','-nocompression')
%     x = 1;
% 
% else
%     
%     % b) load already compiled tracks data
%     load('tracksData_reSampled_v11.mat')
%     load('inD_trackDescriptives_v3.mat') 
% %     tracks =  tracksUpdated;
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% For every time step in a scene for every scene, run the prediction and update loops; 
% the compiled tracks data serve as observations. This way the code can be
% directly reused for any simulations with minimal changes

%% debug
N_Scenes = 12;
% initialize variables
GapFeaturesAllScenes = struct('recordingId',[],'pedcarTrackId',[],'pedTrackTimeStep',[],'egoCarTrack',[],'pedCloseCw',[],'F_pedSpeed',[],'F_pedDistToCW',[],...
                              'F_cumWait',[],'F_pedDistToVeh',[],'F_vehVel',[],'F_pedDistToCurb',[],'F_vehAcc',[],'F_isEgoNearLane',[],...
                              'F_isSameDirection',[],'predDecision',[],'timeStepInHorizon',[]);
CrossFeaturesAllScenes = struct('recordingId',[],'pedcarTrackId',[],'pedTrackTimeStep',[],'timeStepInHorizon',[],'mean_ped_speed',[],...
                                'mean_DTCurb',[],'mean_DTCW',[],'mean_veh_speed',[],'mean_veh_acc',[],'mean_veh_ped_dist',[],...
                                'gaze_ratio',[],'isSameDirection',[], 'isNearLane',[], 'duration_ego_vehicle',[],'closestCW',[]);
predictedPedTraj = cell(12, 211, 613); %maximum sizes of scenes, no. of moving cars, and tracks in scene respectively; pre-allocated for speed
predPedIndex = 0;
GapFeatureId = 1;
CrossFeatureId = 1;
flag.pred = false; % this is to run the close CW function ('hybridState') w/o hybrid state update
% loop starts
for sceneId = 1:N_Scenes
    %initialize scene variables
    pedIndexWithinSceneHistory = [];
    trackTimeStep = 1;  % time loop of the scene  
    carMovingTracks = tracks{sceneId}.carMovingTracks;
    N_tracks = size(formattedTracksData{sceneId},1);
    % assume every moving car track is an ego-AV
    for track_index = 1:length(carMovingTracks)
%     for carTrackId = 6:20
        % initialize track variables
        carTrackId = carMovingTracks(track_index);
        carData = formattedTracksData{sceneId}{carTrackId};
        carHeading = 0;
        pedIndexInTrackHistory = -1*ones(N_tracks,1);
        pedInSceneId = 1;
        %%%%%%%%%%%%%%%%%%%%%%%
        % track time loop starts
        for trackTimeStep = 1:size(carData.recordingId,1)
            tic   % start timer here; before this data reading, from here the process is similar to how an AV would process its information
            cartrackTime = carData.frame(trackTimeStep);
            carPosPixels = double([carData.xCenterPix(trackTimeStep), carData.yCenterPix(trackTimeStep)]);
            carHeading = carData.calcHeading(trackTimeStep);
            AVStates.carPosPixels = carPosPixels;
            AVStates.carHeading = carHeading;
            carTrackCurrentTimeStepInPredictionData = [];
            %%%%%%%%%%%%%%%%%%%%%%%
            % run prediction of the ego-car is approaching a crosswalk, else run constant velocity predictions
            if (carData.closestCW(trackTimeStep)~=0 || carData.closestCW(trackTimeStep)~=inf)
               flag.EgoCar = true;
            else
               flag.EgoCar = false;
            end
            %%%%%%%%%%%%%%%%%%%%%%%
            % find the agents withing the sensing range of the AV
            activeAgentsWithinRange;
            %%%%%%%%%%%%%%%%%%%%%%%
            % run the prediction if there is a pedestrian within the range
            if ~isempty(activePedWithinRange)

                %%%%%%%%%%%%%%%%%%%%%%%
                % For every active pedestrian run the trajectory prediction        
                for pedLoopId = 1: length(activePedWithinRange)  % for every active pedestrian during this time step
                    % initialize pedestrian variables and time step
                    pedIndexWithinScene = activePedWithinRange(pedLoopId);
                    currentPedData = formattedTracksData{sceneId}{pedIndexWithinScene};
                    currentPedMetaData = tracksMetaData{sceneId}(pedIndexWithinScene, :);            
                    pedTrackTimeStep = (cartrackTime - currentPedData.frame(1))/Params.reSampleRate + 1;
                    
                    if trackTimeStep>1 && prevPedTrackTimeStep-prevPedTrackTimeStep~=1
                        x=1;
                    end
                    
                    prevPedTrackTimeStep = pedTrackTimeStep;
                    
                    %%%%%%%%%%%%%%%%%%%%%%%
                    % check if it is an existing pedestrian or a new pedestrian           
                    if ~ismember(pedIndexWithinScene, pedIndexInTrackHistory)
                        pedIndexInTrackHistory(pedInSceneId) = pedIndexWithinScene;
                        pedInSceneId = pedInSceneId + 1;   % this index keeps track of all pedestrians in a scene
                        predPedIndex = predPedIndex + 1;  % this index keeps track of all pedestrians from all scenes               
                        % initialize the pedestrian
                        predictedPedTraj{sceneId}{carTrackId}{pedIndexWithinScene}.sceneId = sceneId;
                        predictedPedTraj{sceneId}{carTrackId}{pedIndexWithinScene}.carTrackId = carTrackId;
                        predictedPedTraj{sceneId}{carTrackId}{pedIndexWithinScene}.pedcarTrackId = pedIndexWithinScene;  
                        predictedPedTraj{sceneId}{carTrackId}{pedIndexWithinScene}.timeStep(1) = inf;
                        predictedPedTraj{sceneId}{carTrackId}{pedIndexWithinScene}.data{1} = inf;
                        predictedPedTraj{sceneId}{carTrackId}{pedIndexWithinScene}.kfData{1} = inf;
                    end
                    %%%%%%%%%%%%%%%%%%%%%%%                    
                    % initialize the kalman filter
                    kf.x = [currentPedData.xCenter(pedTrackTimeStep);
                            currentPedData.yCenter(pedTrackTimeStep);
                            currentPedData.xVelocity(pedTrackTimeStep);
                            currentPedData.yVelocity(pedTrackTimeStep)];                        
                    kf.P = eye(4)*R(1,1);
                    kf.detP = det(kf.P); % Let's keep track of the noise by detP
                    %%%%%%%%%%%%%%%%%%%%%%%
                    
                    [pedPredictions, pedKFpredictions] = HPed_CV(kf, currentPedData, Params, pedTrackTimeStep);
                    
                    %%%%%%%%%%%%%%%%%%%%%%%
                    % Save the predictions
                    if pedTrackTimeStep >= Params.AdjustedSampFreq 
                        predictedPedTraj{sceneId}{carTrackId}{pedIndexWithinScene}.timeStep(end+1,1) = pedTrackTimeStep;
                        predictedPedTraj{sceneId}{carTrackId}{pedIndexWithinScene}.data{end+1,1} = pedPredictions;
                        predictedPedTraj{sceneId}{carTrackId}{pedIndexWithinScene}.kfData{end+1,1} = pedKFpredictions;
                    end
                end % end of all pedestrians
                     
            end  % if there are active pedestrians   
        end  % end of time loop for this scene
    
    end  % end of all cars
  
end % end of all scenes

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

