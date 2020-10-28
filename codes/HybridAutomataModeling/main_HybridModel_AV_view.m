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

%% %%%%%%%%%%%%%%%%%%%%  model setup  %%%%%%%%%%%%%%%%%%%%%
% % a) addpath of necessary directories
% p1 = genpath('G:\My Drive\Research\Projects\pedestrianHybridModel\codes');
% p2 = genpath('G:\My Drive\Research\Projects\pedestrianHybridModel\datasets');
% p3 = genpath('G:\My Drive\Research\Projects\pedestrianHybridModel\results');
p1 = genpath('E:\jskumaar\pedestrianHybridModel\codes');
p2 = genpath('E:\jskumaar\pedestrianHybridModel\datasets');
p3 = genpath('E:\jskumaar\pedestrianHybridModel\results');
addpath(p1)
addpath(p2)
addpath(p3)

% % b)load SVM models
% % load the gap acceptance model
% load('GapAcceptance_inD_8Features_FGaussianSVM_BootStrappedTwice_v2.mat', 'GapAcceptance_inD_8Features_FGaussianSVM_BootStrappedTwice_v2');
% GapAcceptanceModel = GapAcceptance_inD_8Features_FGaussianSVM_BootStrappedTwice_v2.ClassificationSVM;
% Prob_GapAcceptanceModel = fitSVMPosterior(GapAcceptanceModel);
% % load the crossing intent model
% load('CrossIntent_inD_9Features_BS1_noDuration_FGaussianSVM_3s_v2.mat');
% CrossIntentModelCar = CrossIntent_inD_9Features_BS2_noDuration_FGaussianSVM_3s_v2.ClassificationSVM;
% Prob_CrossIntentModelCar = fitSVMPosterior(CrossIntentModelCar);
% load('CrossIntent_NoCar_inD_3Features_BS1_noDuration_FGaussianSVM_v3.mat');
% CrossIntentModelNoCar = CrossIntent_NoCar_inD_3Features_BS1_noDuration_FGaussianSVM_v3.ClassificationSVM;
% Prob_CrossIntentModelNoCar = fitSVMPosterior(CrossIntentModelNoCar);

% 
%c) parameters
configure_MHP;
% initialize output variables
predictedPedTraj_MHP = cell(12, 211, 613); %maximum sizes of scenes, no. of moving cars, and tracks in scene respectively; pre-allocated for speed
% predictedPedTraj_HBase = cell(12, 211, 613);
% predictedPedTraj_CV = cell(12, 211, 613);


% % 
% % d) Read data or compile data?
flag.dataCompile = false;
% flag.dataCompile = true;
% [formattedTracksData, tracksMetaData, tracks] = load_compile_data(flag, dataset);

% e) initialize variables
GapFeaturesAllScenes = struct('recordingId',[],'pedcarTrackId',[],'pedTrackTimeStep',[],'egoCarTrack',[],'pedCloseCw',[],'F_pedSpeed',[],'F_pedDistToCW',[],...
                              'F_cumWait',[],'F_pedDistToVeh',[],'F_vehVel',[],'F_pedDistToCurb',[],'F_vehAcc',[],'F_isEgoNearLane',[],...
                              'F_isSameDirection',[],'predDecision',[],'timeStepInHorizon',[]);
CrossFeaturesAllScenes = struct('recordingId',[],'pedcarTrackId',[],'pedTrackTimeStep',[],'timeStepInHorizon',[],'mean_ped_speed',[],...
                                'mean_DTCurb',[],'mean_DTCW',[],'mean_veh_speed',[],'mean_veh_acc',[],'mean_veh_ped_dist',[],...
                                'gaze_ratio',[],'isSameDirection',[], 'isNearLane',[], 'duration_ego_vehicle',[],'closestCW',[],'predDecision',[]);
predPedIndex = 0;
GapFeatureId = 1;
CrossFeatureId = 1;
flag.pred = false; % this is to run the close CW function ('hybridState') w/o hybrid state update
%%%%%%%%%%%%%%%%%%%%%%  setup ends  %%%%%%%%%%%%%%%%%%%%%


%% %%%%%%%%%%%%%%%%%% Prediction Loop for every car  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Every car is considered as the ego-vehicle and has their zone of observation/interaction, defined in 'configure_MHP.m'. 
% For every ego-vehicle at every time step, trajectories are predicted for
% all pedestrians within the the interaction zone.

%%%%%%%%%%%%%%%%%%%%%%%
% prediction loop starts for all cars in the dataset
for sceneId = 1:1
    %initialize scene variables
    pedIndexWithinSceneHistory = [];
    trackTimeStep = 1;  % time loop of the scene  
    carMovingTracks = tracks{sceneId}.carMovingTracks;
    N_tracks = size(formattedTracksData{sceneId},1);
    % assume every moving car track is an ego-vehicle
    for track_index = 1:length(carMovingTracks)
        % initialize track variables
        carTrackId = carMovingTracks(track_index);
        carData = formattedTracksData{sceneId}{carTrackId};
        pedIndexInTrackHistory = -1*ones(N_tracks,1);
        pedInSceneId = 1;
        %%%%%%%%%%%%%%%%%%%%%%%
        % track time loop starts
        for trackTimeStep = 1:size(carData.recordingId,1)
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
                currentTSActiveCarData = cell(length(activeCarWithinRange),1);                
                actCarNos =  length(activeCarWithinRange);
                tempCarTrackCurrentTimeStep = zeros(1,actCarNos);
               
                % compile all cars' states within the sensing range (for M-1 time steps before the current time step, where M is the observation window and for next N time steps after the current time step, where N is the prediction horizon)
                carTrackCurrentTimeStepInPredictionData = -1*ones(actCarNos,1);
                for carLoopId = 1: actCarNos
                    carIndex = activeCarWithinRange(carLoopId);
                    tempCarTrackCurrentTimeStep = (trackTime - formattedTracksData{sceneId}{carIndex}.frame(1))/Params.reSampleRate + 1;                    
                    carTrackStartTimeStep = max(tempCarTrackCurrentTimeStep - Params.observationWindow + 1, 1);   % [t-Observation : t+predictionHorizon]              
                    carTrackCurrentTimeStepInPredictionData(carLoopId) = min(tempCarTrackCurrentTimeStep-carTrackStartTimeStep+1, Params.observationWindow);                    
                    carTrackEndTimeStep = min(tempCarTrackCurrentTimeStep + Params.predHorizon-1, length(formattedTracksData{sceneId}{carIndex}.frame) ); % comment these lines out if using only current time step car data and predicting car data
                    carTrackTimeStep = (carTrackStartTimeStep:carTrackEndTimeStep); % comment these lines out if using only current time step car data and predicting car data
                                       
                    % copy observed data to variables 
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
                    % define new variables
                    currentTSActiveCarData{carLoopId}.turn = false;
                    currentTSActiveCarData{carLoopId}.changeLane = false;
                    currentTSActiveCarData{carLoopId}.reachGoal = false;
                end
            

                %%%%%%%%%%%%%%%%%%%%%%%
                % For every active pedestrian run the trajectory prediction        
                for pedLoopId = 1: length(activePedWithinRange)  % for every active pedestrian during this time step
                    % initialize pedestrian variables and time step
                    pedIndexWithinScene = activePedWithinRange(pedLoopId);
                    currentPedData = formattedTracksData{sceneId}{pedIndexWithinScene};
                    currentPedMetaData = tracksMetaData{sceneId}(pedIndexWithinScene, :);            
                    pedTrackTimeStep = (trackTime - currentPedData.frame(1))/Params.reSampleRate + 1;
                    
                    %% debug
                    if sceneId==1 && track_index==14 && pedIndexWithinScene==54 && pedTrackTimeStep==66
                        x=1;
                    end
                    
                    
                    %%%%%%%%%%%%%%%%%%%%%%%
                    % check if it is an existing pedestrian or a new pedestrian           
                    if ~ismember(pedIndexWithinScene, pedIndexInTrackHistory)
                        pedIndexInTrackHistory(pedInSceneId) = pedIndexWithinScene;
                        pedInSceneId = pedInSceneId + 1;   % this index keeps track of all pedestrians in a scene
                        predPedIndex = predPedIndex + 1;  % this index keeps track of all pedestrians from all scenes               
                        % initialize the pedestrian
                        predictedPedTraj_MHP{sceneId}{track_index,1}{pedIndexWithinScene,1}.sceneId = sceneId;
                        predictedPedTraj_MHP{sceneId}{track_index,1}{pedIndexWithinScene,1}.carTrackId = carTrackId;
                        predictedPedTraj_MHP{sceneId}{track_index,1}{pedIndexWithinScene,1}.pedcarTrackId = pedIndexWithinScene;  
                        predictedPedTraj_MHP{sceneId}{track_index,1}{pedIndexWithinScene,1}.timeStep(1) = inf;
                        predictedPedTraj_MHP{sceneId}{track_index,1}{pedIndexWithinScene,1}.data{1} = inf;
                        predictedPedTraj_MHP{sceneId}{track_index,1}{pedIndexWithinScene,1}.kfData{1} = inf;
                        predictedPedTraj_MHP{sceneId}{track_index,1}{pedIndexWithinScene,1}.predictionModel(1) = 0;
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
                        if strcmp(predictionModel,"MultipleHybridPedestrian")
                            [pedPredictions, pedKFpredictions, predGapFeatures, predCrossFeatures] = predictStates(kf, currentPedData, currentPedMetaData, currentTSActiveCarData, carTrackCurrentTimeStepInPredictionData, AVStates, pedTrackTimeStep, ...
                                                                          cw, annotatedImageEnhanced, resetStates, Prob_GapAcceptanceModel, Prob_CrossIntentModelCar,Prob_CrossIntentModelNoCar, Params, flag);
                             predModelForTimeStep = 2;
                        elseif strcmp(predictionModel,"BaselineHybrid")
                            [pedPredictions, pedKFpredictions, predGapFeatures] = predictStates_baseHybrid_v2(kf, currentPedData, currentPedMetaData, currentTSActiveCarData, carTrackCurrentTimeStepInPredictionData, AVStates, pedTrackTimeStep, ...
                                                                         cw, annotatedImageEnhanced, resetStates, Prob_GapAcceptanceModel, Params, flag);
                             predModelForTimeStep = 1;
                        elseif strcmp(predictionModel,"ConstantVelocity")
                            [pedPredictions, pedKFpredictions] = HPed_MHP(kf, currentPedData, Params, pedTrackTimeStep);
                            predModelForTimeStep = 0;
                        end                                         
                       % Gap features
                       if strcmp(predictionModel,"MultipleHybridPedestrian") || strcmp(predictionModel,"BaselineHybrid")
                            N_gaps = size(predGapFeatures,1);
                            if N_gaps >=1
                                predGapFeatures.recordingId = sceneId;
                                predGapFeatures.pedcarTrackId = pedIndexWithinScene;
                                GapFeaturesAllScenes(GapFeatureId:GapFeatureId+N_gaps-1) =  predGapFeatures;
                                GapFeatureId = GapFeatureId + N_gaps;
                            end
                       end
                        % Cross intent features
                        if strcmp(predictionModel,"MultipleHybridPedestrian")
                            N_cross = size(predCrossFeatures,1);
                            if N_cross >=1
                                predCrossFeatures.recordingId = sceneId;
                                predCrossFeatures.pedcarTrackId = pedIndexWithinScene;
                                CrossFeaturesAllScenes(CrossFeatureId:CrossFeatureId+N_cross-1) = predCrossFeatures;
                                CrossFeatureId = CrossFeatureId + N_cross;
                            end
                        end
                    else
                        [pedPredictions, pedKFpredictions] = HPed_MHP(kf, currentPedData, Params, pedTrackTimeStep);
                        predModelForTimeStep = 0;
                    end
                    %%%%%%%%%%%%%%%%%%%%%%%
                    % Save the predictions
%                     if pedTrackTimeStep >= Params.AdjustedSampFreq 
                        predictedPedTraj_MHP{sceneId}{track_index,1}{pedIndexWithinScene,1}.timeStep(end+1,1) = pedTrackTimeStep;
                        predictedPedTraj_MHP{sceneId}{track_index,1}{pedIndexWithinScene,1}.data{end+1,1} = pedPredictions;
                        predictedPedTraj_MHP{sceneId}{track_index,1}{pedIndexWithinScene,1}.kfData{end+1,1} = pedKFpredictions;
                        predictedPedTraj_MHP{sceneId}{track_index,1}{pedIndexWithinScene,1}.predictionModel(end+1,1) = predModelForTimeStep;
%                         %% debug
%                         prevTimeStep = predictedPedTraj_MHP{sceneId}{track_index}{pedIndexWithinScene}.timeStep(end-1,1);
%                         if pedTrackTimeStep-prevTimeStep~=1 && prevTimeStep~=inf
%                             x=1;
%                         end
%                         % ground truth
%                         GT_trajectory = [formattedTracksData{sceneId}{pedIndexWithinScene}.xCenter(pedTrackTimeStep:end),  formattedTracksData{sceneId}{pedIndexWithinScene}.xCenter(pedTrackTimeStep:end)];
%                         pred_trajectory = predictedPedTraj_MHP{sceneId}{track_index}{pedIndexWithinScene}.data{end}/(Params.orthopxToMeter*Params.scaleFactor);
%                        
%                         
%                         for ii=1:length(currentTSActiveCarData)
%                             carPosPixels = int32([currentTSActiveCarData{ii}.xCenter, currentTSActiveCarData{ii}.yCenter]/(Params.orthopxToMeter*Params.scaleFactor));
%                             for kk=1:length(carPosPixels)
%                                 annotatedImageEnhanced(-carPosPixels(kk,2), carPosPixels(kk,1)) = 150;
%                             end
%                         end
% 
%                         for jj=1:size(pred_trajectory,1)
%                             temp_pred_trajectory = reshape(pred_trajectory(jj,3:end), [(length(pred_trajectory)-2)/2, 2]);
%                             annotatedImageEnhanced(-carPosPixels(kk,2), carPosPixels(kk,1)) = 150;
%                         end
%                         imshow(annotatedImageEnhanced)
%                         x=1;
%                     end
                end % end of all pedestrians
                     
            end  % if there are active pedestrians   
        end  % end of time loop for this scene
    
    end  % end of all cars
  
end % end of all scenes
%%%%%%%%%%%%%%%%%%%% End of Prediction Loop for every car  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% remove initialized empty data
GapFeaturesAllScenes(GapFeatureId:end,:) =  [];
CrossFeaturesAllScenes(CrossFeatureId:end, :) = [];
