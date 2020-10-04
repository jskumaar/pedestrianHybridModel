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
p1 = genpath('G:\My Drive\Research\Projects\pedestrianHybridModel\codes');
p2 = genpath('G:\My Drive\Research\Projects\pedestrianHybridModel\datasets');
% p1 = genpath('E:\pedestrianHybridModel\codes');
% p2 = genpath('E:\pedestrianHybridModel\datasets');
addpath(p1)
addpath(p2)

% % % b)load models
% % load the gap acceptance model
% load('GapAcceptance_inD_8Features_FGaussianSVM_BootStrappedTwice.mat', 'GapAcceptance_inD_8Features_FGaussianSVM_BootStrappedTwice');
% GapAcceptanceModel = GapAcceptance_inD_8Features_FGaussianSVM_BootStrappedTwice.ClassificationSVM;
% Prob_GapAcceptanceModel = fitSVMPosterior(GapAcceptanceModel);
% % load the crossing intent model
% load('CrossIntent_inD_9Features_BS1_noDuration_FGaussianSVM_3s_v2.mat');
% CrossIntentModelCar = CrossIntent_inD_9Features_BS2_noDuration_FGaussianSVM_3s_v2.ClassificationSVM;
% Prob_CrossIntentModelCar = fitSVMPosterior(CrossIntentModelCar);
% load('CrossIntent_NoCar_inD_3Features_BS1_noDuration_FGaussianSVM_v2.mat');
% CrossIntentModelNoCar = CrossIntent_NoCar_inD_3Features_BS1_noDuration_FGaussianSVM_v2.ClassificationSVM;
% Prob_CrossIntentModelNoCar = fitSVMPosterior(CrossIntentModelNoCar);
% % read tracks MetaData
% for jj=1:12
%     sceneId = 17+jj;
%     tracksMetaData{jj} = readtable(strcat(num2str(sceneId),'_tracksMeta.csv')) ;
% end

%c) parameters
dataset="inD";
HPed_params;
% 
% d) Read data or compile data?
flag.dataCompile = false;
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % compile/data tracks data if flag.dataCompile
%     % a) Run the following if compiling data for the first time, else
% %     run1b) 
% if flag.dataCompile
%     [formattedTracksData, allTracksMetaData, N_Scenes, annotatedImageEnhanced] = inD_compile(Params, reset);
%     [tracks, ~] = trackDescriptives(formattedTracksData, N_scenes);
% 
%     for scene_id = 1:N_scenes
%         tracksMetaData{scene_id}.ego_veh_gap_hist(1:size(tracksMetaData{scene_id},1))  = {zeros(20,1)};  % for inD dataset the maximum number of gaps
%         tracksMetaData{scene_id}.wait_start_hist(1:size(tracksMetaData{scene_id},1))  = {zeros(20,1)}; 
%     end
% 
%         % %check for ego-pedestrian and pedestrian gaze for the entire dataset
%         egoPed_Gaze_HPed;
% 
%         % save the data file for later reuse
%         save('tracksData_reSampled_correctDisCW_v2.mat','formattedTracksData','tracksMetaData','-v7.3','-nocompression')
%         x = 1;
%     
% else
%     
%     % b) load already compiled tracks data
%     load('tracksData_reSampled_correctDisCW_v8.mat')
%     load('inD_trackDescriptives_removed_ped_tracks_v2.mat') 
%     tracks =  tracksUpdated;
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% For every time step in a scene for every scene, run the prediction and update loops; 
% the compiled tracks data serve as observations. This way the code can be
% directly reused for any simulations with minimal changes

%% debug
N_Scenes = 1;
% initialize variables
GapFeaturesAllScenes = struct('recordingId',[],'pedTrackId',[],'pedTrackTimeStep',[],'egoCarTrack',[],'pedCloseCw',[],'F_pedSpeed',[],'F_pedDistToCW',[],...
                              'F_cumWait',[],'F_pedDistToVeh',[],'F_vehVel',[],'F_pedDistToCurb',[],'F_vehAcc',[],'F_isEgoNearLane',[],...
                              'F_isSameDirection',[],'predDecision',[],'timeStepInHorizon',[]);
CrossFeaturesAllScenes = struct('recordingId',[],'pedTrackId',[],'pedTrackTimeStep',[],'timeStepInHorizon',[],'mean_ped_speed',[],...
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
    for trackId = 1:length(carMovingTracks)
%     for trackId = 6:20
        % initialize track variables
        carTrackId = carMovingTracks(trackId);
        carData = formattedTracksData{sceneId}{carTrackId};
        carHeading = 0;
        pedIndexInTrackHistory = -1*ones(N_tracks,1);
        pedInSceneId = 1;
        %%%%%%%%%%%%%%%%%%%%%%%
        % track time loop starts
        for trackTimeStep = 1:size(carData.recordingId,1)
            tic   % start timer here; before this data reading, from here the process is similar to how an AV would process its information
            trackTime = carData.frame(trackTimeStep);
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
                % Update the current data of all active cars
                % 10 - xVelocity, 11- yVelocity, 12- xAcceleration, 13 - yAcceleration, 14-lonVelocity, 16-lonAcceleration, 17 - latAcceleration, 
                % 18 - xCenterPix, 19 - yCenterPix, 21- closestCW, 23- calcHeading, 24 - car_lane
                variablesToCopy = [10, 11, 12, 13, 14, 16, 17, 18, 19, 21, 23, 24];
                currentTSActiveCarData = cell(length(activeCarWithinRange),1);
                
                actCarNos =  length(activeCarWithinRange);
                tempCarTrackCurrentTimeStep = zeros(1,actCarNos);
                
                %% debug
                if actCarNos==3
                    x=1;
                end
                
               
                % compile all cars' states within the sensing range (for M-1 time steps before the current time step, where M is the observation window and for next N time steps after the current time step, where N is the prediction horizon)
                for carLoopId = 1: actCarNos
                    carIndex = activeCarWithinRange(carLoopId);
%                     trackTime_carIndex = find(formattedTracksData{sceneId}{carIndex}.frame==trackTime,1,'first');
%                     carTrackCurrentTimeStep(carLoopId) = max((trackTime - formattedTracksData{sceneId}{carIndex}.frame(1))/Params.reSampleRate + 1, 1);
%                     carTrackCurrentTimeStep(carLoopId) = Params.observationWindow - (trackTime - formattedTracksData{sceneId}{carIndex}.frame(1))/Params.reSampleRate + 1 - Params.observationWindow);
%                     carTrackCurrentTimeStep(carLoopId) = max((trackTime - formattedTracksData{sceneId}{carIndex}.frame(1))/Params.reSampleRate + 1, 1);
                    tempCarTrackCurrentTimeStep = (trackTime - formattedTracksData{sceneId}{carIndex}.frame(1))/Params.reSampleRate + 1;                    
                    carTrackStartTimeStep = max(tempCarTrackCurrentTimeStep - Params.observationWindow + 1, 1);   % [t-Observation : t+predictionHorizon]              
                    carTrackCurrentTimeStepInPredictionData(carLoopId) = min(tempCarTrackCurrentTimeStep-carTrackStartTimeStep+1, Params.observationWindow);                    
                    carTrackEndTimeStep = min(tempCarTrackCurrentTimeStep + Params.predHorizon-1, length(formattedTracksData{sceneId}{carIndex}.frame) ); % comment these lines out if using only current time step car data and predicting car data
                    carTrackTimeStep = (carTrackStartTimeStep:carTrackEndTimeStep); % comment these lines out if using only current time step car data and predicting car data
                                       
%                     % copy variables (for current time step data only)
%                     currentTSActiveCarData.xCenter = formattedTracksData{sceneId}{carIndex}.xCenter(carTrackTimeStep);
%                     currentTSActiveCarData.yCenter(carLoopId) = formattedTracksData{sceneId}{carIndex}.yCenter(carTrackTimeStep);
%                     currentTSActiveCarData.xVelocity(carLoopId) = formattedTracksData{sceneId}{carIndex}.xVelocity(carTrackTimeStep);
%                     currentTSActiveCarData.yVelocity(carLoopId) = formattedTracksData{sceneId}{carIndex}.yVelocity(carTrackTimeStep);
%                     currentTSActiveCarData.xAcceleration(carLoopId) = formattedTracksData{sceneId}{carIndex}.xAcceleration(carTrackTimeStep);
%                     currentTSActiveCarData.yAcceleration(carLoopId) = formattedTracksData{sceneId}{carIndex}.yAcceleration(carTrackTimeStep);
%                     currentTSActiveCarData.lonVelocity(carLoopId) = formattedTracksData{sceneId}{carIndex}.lonVelocity(carTrackTimeStep);
%                     currentTSActiveCarData.lonAcceleration(carLoopId) = formattedTracksData{sceneId}{carIndex}.lonAcceleration(carTrackTimeStep);
%                     currentTSActiveCarData.latAcceleration(carLoopId) = formattedTracksData{sceneId}{carIndex}.latAcceleration(carTrackTimeStep);
%                     currentTSActiveCarData.closestCW(carLoopId) = formattedTracksData{sceneId}{carIndex}.closestCW(carTrackTimeStep);
%                     currentTSActiveCarData.calcHeading(carLoopId) = formattedTracksData{sceneId}{carIndex}.calcHeading(carTrackTimeStep);
%                     currentTSActiveCarData.car_lane(carLoopId) = formattedTracksData{sceneId}{carIndex}.car_lane(carTrackTimeStep);
%                     currentTSActiveCarData.carTrackId(carLoopId) = formattedTracksData{sceneId}{carIndex}.trackId(carTrackTimeStep);
                    
                    % copy variables 
                    currentTSActiveCarData{carLoopId}.xCenter = formattedTracksData{sceneId}{carIndex}.xCenter(carTrackTimeStep);
                    currentTSActiveCarData{carLoopId}.yCenter = formattedTracksData{sceneId}{carIndex}.yCenter(carTrackTimeStep);
                    currentTSActiveCarData{carLoopId}.xVelocity = formattedTracksData{sceneId}{carIndex}.xVelocity(carTrackTimeStep);
                    currentTSActiveCarData{carLoopId}.yVelocity = formattedTracksData{sceneId}{carIndex}.yVelocity(carTrackTimeStep);
                    currentTSActiveCarData{carLoopId}.xAcceleration = formattedTracksData{sceneId}{carIndex}.xAcceleration(carTrackTimeStep);
                    currentTSActiveCarData{carLoopId}.yAcceleration = formattedTracksData{sceneId}{carIndex}.yAcceleration(carTrackTimeStep);
                    currentTSActiveCarData{carLoopId}.lonVelocity = formattedTracksData{sceneId}{carIndex}.lonVelocity(carTrackTimeStep);
                    currentTSActiveCarData{carLoopId}.lonAcceleration = formattedTracksData{sceneId}{carIndex}.lonAcceleration(carTrackTimeStep);
                    currentTSActiveCarData{carLoopId}.latAcceleration = formattedTracksData{sceneId}{carIndex}.latAcceleration(carTrackTimeStep);
                    currentTSActiveCarData{carLoopId}.closestCW = formattedTracksData{sceneId}{carIndex}.closestCW(carTrackTimeStep);
                    currentTSActiveCarData{carLoopId}.calcHeading = formattedTracksData{sceneId}{carIndex}.calcHeading(carTrackTimeStep);
                    currentTSActiveCarData{carLoopId}.car_lane = formattedTracksData{sceneId}{carIndex}.car_lane(carTrackTimeStep);
                    currentTSActiveCarData{carLoopId}.carTrackId = formattedTracksData{sceneId}{carIndex}.trackId(carTrackTimeStep);
                    
                    currentTSActiveCarData{carLoopId}.turn = false;
                    currentTSActiveCarData{carLoopId}.changeLane = false;
                    currentTSActiveCarData{carLoopId}.reachGoal = false;
                end
                
                %% debug
                if actCarNos==3
                    if carTrackCurrentTimeStepInPredictionData(1)~=carTrackCurrentTimeStepInPredictionData(2)
                        x=1;
                    end
                end

                %%%%%%%%%%%%%%%%%%%%%%%
                % For every active pedestrian run the trajectory prediction        
                for pedLoopId = 1: length(activePedWithinRange)  % for every active pedestrian during this time step
                    % initialize pedestrian variables and time step
                    pedIndexWithinScene = activePedWithinRange(pedLoopId);
                    currentPedData = formattedTracksData{sceneId}{pedIndexWithinScene};
                    currentPedMetaData = tracksMetaData{sceneId}(pedIndexWithinScene, :);            
                    pedTrackTimeStep = (trackTime - currentPedData.frame(1))/Params.reSampleRate + 1;
                    %%%%%%%%%%%%%%%%%%%%%%%
                    % check if it is an existing pedestrian or a new pedestrian           
                    if ~ismember(pedIndexWithinScene, pedIndexInTrackHistory)
                        pedIndexInTrackHistory(pedInSceneId) = pedIndexWithinScene;
                        pedInSceneId = pedInSceneId + 1;   % this index keeps track of all pedestrians in a scene
                        predPedIndex = predPedIndex + 1;  % this index keeps track of all pedestrians from all scenes               
                        % initialize the pedestrian
                        predictedPedTraj{sceneId}{trackId,1}{pedIndexWithinScene,1}.sceneId = sceneId;
                        predictedPedTraj{sceneId}{trackId,1}{pedIndexWithinScene,1}.carTrackId = carTrackId;
                        predictedPedTraj{sceneId}{trackId,1}{pedIndexWithinScene,1}.pedTrackId = pedIndexWithinScene;  
                        predictedPedTraj{sceneId}{trackId,1}{pedIndexWithinScene,1}.timeStep(1) = inf;
                        predictedPedTraj{sceneId}{trackId,1}{pedIndexWithinScene,1}.data{1} = inf;
                        predictedPedTraj{sceneId}{trackId,1}{pedIndexWithinScene,1}.kfData{1} = inf;
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
                    % Run the H-Ped prediction framework if there is an ego
                    % car for the pedestrian, else run a constant velocity
                    % model
                    if flag.EgoCar
%                         if (currentPedData.closeCar_ind(pedTrackTimeStep)~=0 && currentPedData.closeCar_ind(pedTrackTimeStep)~=inf)
%                             crossCarFullData = formattedTracksData{sceneId}{currentPedData.closeCar_ind(pedTrackTimeStep)};
%                             crossCarData.frame = crossCarFullData.frame;
%                             crossCarData.trackLifetime = crossCarFullData.trackLifetime;
%                             crossCarData.xCenter = crossCarFullData.xCenter;
%                             crossCarData.calcHeading = crossCarFullData.frame;
%                             crossCarData.yCenter = crossCarFullData.yCenter;
%                             crossCarData.xVelocity = crossCarFullData.xVelocity;
%                             crossCarData.yVelocity = crossCarFullData.yVelocity;
%                             crossCarData.lonVelocity = crossCarFullData.lonVelocity;
%                             crossCarData.latAcceleration = crossCarFullData.latAcceleration;
%                             crossCarData.lonAcceleration = crossCarFullData.lonAcceleration;
%                             crossCarData.xAcceleration = crossCarFullData.xAcceleration;
%                             crossCarData.yAcceleration = crossCarFullData.yAcceleration;
%                             crossCarData.distCW = crossCarFullData.distCW;
%                             crossCarData.closestCW = crossCarFullData.closestCW;
%                             crossCarData.car_lane = crossCarFullData.car_lane;
% %                             crossCarData.long_disp_ped_car = crossCarFullData.long_disp_ped_car;
% %                             crossCarData.isPedSameDirection = crossCarFullData.isPedSameDirection;
%                         else
%                             crossCarData = [];
%                         end
                        [pedPredictions, pedKFpredictions, predGapFeatures, predCrossFeatures] = predictStates(kf, currentPedData, currentPedMetaData, currentTSActiveCarData, carTrackCurrentTimeStepInPredictionData, AVStates, pedTrackTimeStep, ...
                                                                           cw, annotatedImageEnhanced, resetStates, Prob_GapAcceptanceModel, Prob_CrossIntentModelCar,Prob_CrossIntentModelNoCar, Params, flag);
                        % Gap features
                        N_gaps = size(predGapFeatures,1);
                        if N_gaps >=1
                            predGapFeatures.recordingId = sceneId;
                            predGapFeatures.pedTrackId = pedIndexWithinScene;
                            GapFeaturesAllScenes(GapFeatureId:GapFeatureId+N_gaps-1) =  predGapFeatures;
                            GapFeatureId = GapFeatureId + N_gaps;
                        end
                        % Cross intent features
                        N_cross = size(predCrossFeatures,1);
                        if N_cross >=1
                            predCrossFeatures.recordingId = sceneId;
                            predCrossFeatures.pedTrackId = pedIndexWithinScene;
                            CrossFeaturesAllScenes(CrossFeatureId:CrossFeatureId+N_cross-1) = predCrossFeatures;
                            CrossFeatureId = CrossFeatureId + N_cross;
                        end
                    else
                        [pedPredictions, pedKFpredictions] = HPed_CV(kf, currentPedData, Params, pedTrackTimeStep);
                    end
                    %%%%%%%%%%%%%%%%%%%%%%%
                    % Save the predictions
                    if pedTrackTimeStep >= Params.AdjustedSampFreq 
                        predictedPedTraj{sceneId}{trackId,1}{pedIndexWithinScene,1}.timeStep(end+1,1) = pedTrackTimeStep;
                        predictedPedTraj{sceneId}{trackId,1}{pedIndexWithinScene,1}.data{end+1,1} = pedPredictions;
                        predictedPedTraj{sceneId}{trackId,1}{pedIndexWithinScene,1}.kfData{end+1,1} = pedKFpredictions;
                    end
                end % end of all pedestrians
                     
            end  % if there are active pedestrians   
        end  % end of time loop for this scene
    
    end  % end of all cars
  
end % end of all scenes

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% remove initialized empty data
GapFeaturesAllScenes(GapFeatureId:end,:) =  [];
CrossFeaturesAllScenes(CrossFeatureId:end, :) = [];
