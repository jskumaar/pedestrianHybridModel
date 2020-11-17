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
% clearvars -except predictedPedTraj_MHP GapFeaturesAllScenes CrossFeaturesAllScenes predPedIndex GapFeatureId CrossFeatureId flag predictionModel resetStates annotatedImageEnhanced formattedTracksData tracks tracksMetaData cw Prob_CrossIntentModelCar Prob_CrossIntentModelNoCar Prob_GapAcceptanceModel Params
clearvars -except annotatedImageEnhanced formattedTracksData tracks tracksMetaData cw Prob_CrossIntentModelCar Prob_CrossIntentModelNoCar Prob_GapAcceptanceModel Params

%% %%%%%%%%%%%%%%%%%%%%  model setup  %%%%%%%%%%%%%%%%%%%%%
% a) addpath of necessary directories
% p1 = genpath('G:\My Drive\Research\Projects\pedestrianHybridModel\codes');
% p2 = genpath('G:\My Drive\Research\Projects\pedestrianHybridModel\datasets');
% p3 = genpath('G:\My Drive\Research\Projects\pedestrianHybridModel\results');
p1 = genpath('E:\jskumaar\pedestrianHybridModel\codes');
p2 = genpath('E:\jskumaar\pedestrianHybridModel\datasets');
p3 = genpath('E:\jskumaar\pedestrianHybridModel\results');
addpath(p1)
addpath(p2)
addpath(p3)

% b) parameters
script_configureMHP;

% c) set up and initialize the model
script_modelSetup;
%%%%%%%%%%%%%%%%%%%%%%  setup ends  %%%%%%%%%%%%%%%%%%%%%
%% model debug
debugMode = true;
% debugMode = false;

indices_imp = [1	70	231	2	85];

sceneId_interest = indices_imp(1);
car_index_interest = indices_imp(2);
pedTrackId_interest = indices_imp(3);
predictionTimeStep_interest = indices_imp(4);
actualTimeStep_interest = indices_imp(5);
%%%%%%%%%%%%%%%%

%% %%%%%%%%%%%%%%%%%% Prediction Loop for every car  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Every car is considered as the ego-vehicle and has their zone of observation/interaction, defined in 'configure_MHP.m'. 
% For every ego-vehicle at every time step, trajectories are predicted for
% all pedestrians within the the interaction zone.

%%%%%%%%%%%%%%%%%%%%%%%
% prediction loop starts for all cars in the dataset
%% debug
for sceneId = 1:12
% for sceneId = sceneId_interest
    % scene variables
    carMovingTracks = tracks{sceneId}.carMovingTracks;
    % assume every moving car track is an ego-vehicle
    %% debug
    for track_index = 1:length(carMovingTracks)
%     for track_index = car_index_interest
        % initialize track variables
        carTrackId = carMovingTracks(track_index);
%         carTrackId = car_index_interest;
        carData = formattedTracksData{sceneId}{carTrackId};
        pedIndexInTrackHistory = [];
        pedInSceneId = 1;
        %%%%%%%%%%%%%%%%%%%%%%%
        % track time loop starts
        for trackTimeStep = 1:size(carData.recordingId,1)
            trackTime = carData.frame(trackTimeStep);
            carPosPixels = double([carData.xCenterPix(trackTimeStep), carData.yCenterPix(trackTimeStep)]);
            carHeading = carData.calcHeading(trackTimeStep);
            % compile ego car data for prediction
            egoCarEndTimeStep = min(trackTimeStep + Params.predHorizon-1, length(carData.frame)); 
            AVStates.carPosPixels =  double([carData.xCenterPix(trackTimeStep:egoCarEndTimeStep), carData.yCenterPix(trackTimeStep:egoCarEndTimeStep)]);
            AVStates.carHeading = carData.calcHeading(trackTimeStep:egoCarEndTimeStep);
            %%%%%%%%%%%%%%%%%%%%%%%
%             % run prediction of the ego-car is approaching a crosswalk, else run constant velocity predictions
%             if (carData.closestCW(trackTimeStep)~=0 || carData.closestCW(trackTimeStep)~=inf)
%                flag.EgoCar = true;
%             else
%                flag.EgoCar = false;
%             end
            %%%%%%%%%%%%%%%%%%%%%%%
            % find the agents withing the sensing range of the AV
            script_activeAgentsWithinRange;
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
                    activeCarData = formattedTracksData{sceneId}{carIndex};                  
                    % copy observed data to variables                    
                    newActiveCarStruct = struct('xCenter',activeCarData.xCenter(carTrackTimeStep), 'yCenter',activeCarData.yCenter(carTrackTimeStep), 'xVelocity', activeCarData.xVelocity(carTrackTimeStep), ...
                                                'yVelocity', activeCarData.yVelocity(carTrackTimeStep), 'xAcceleration', activeCarData.xAcceleration(carTrackTimeStep), 'yAcceleration', activeCarData.yAcceleration(carTrackTimeStep),...
                                                'lonVelocity', activeCarData.lonVelocity(carTrackTimeStep), 'lonAcceleration', activeCarData.lonAcceleration(carTrackTimeStep), 'latAcceleration', activeCarData.latAcceleration(carTrackTimeStep),...
                                                'closestCW', activeCarData.closestCW(carTrackTimeStep), 'calcHeading', activeCarData.calcHeading(carTrackTimeStep), 'car_lane', activeCarData.car_lane(carTrackTimeStep),...
                                                'carTrackId', activeCarData.trackId(carTrackTimeStep), 'turn', false, 'changeLane', false, 'reachGoal', false);
                    currentTSActiveCarData{carLoopId} = newActiveCarStruct;
                end
            
                %%%%%%%%%%%%%%%%%%%%%%%
                % For every active pedestrian run the trajectory prediction        
                for pedLoopId = 1: length(activePedWithinRange)  % for every active pedestrian during this time step
                    % initialize pedestrian variables and time step
                    pedIndexWithinScene = activePedWithinRange(pedLoopId);
                    %% debug
%                     if pedIndexWithinScene == pedTrackId_interest
                    %% debug
%                     if sceneId==sceneId_interest && track_index==car_index_interest && pedIndexWithinScene==pedTrackId_interest && pedTrackTimeStep==predictionTimeStep_interest
                    if sceneId==sceneId_interest && track_index==car_index_interest && pedIndexWithinScene==pedTrackId_interest
                        x=1;
                    end

                    currentPedData = formattedTracksData{sceneId}{pedIndexWithinScene};
                    currentPedMetaData = tracksMetaData{sceneId}(pedIndexWithinScene, :);            
                    pedTrackTimeStep = (trackTime - currentPedData.frame(1))/Params.reSampleRate + 1;        
                    % initialize the kalman filter
                    kf.x = [currentPedData.xCenter(pedTrackTimeStep); currentPedData.yCenter(pedTrackTimeStep); currentPedData.xVelocity(pedTrackTimeStep); currentPedData.yVelocity(pedTrackTimeStep)];                        
                    kf.P = eye(4)*R(1,1);
                    kf.detP = det(kf.P); % Let's keep track of the noise by detP                    
                    %%%%%%%%%%%%%%%%%%%%%%%
                    % initialize the output if it is a new pedestrian           
                    if ~ismember(pedIndexWithinScene, pedIndexInTrackHistory)
                            pedIndexInTrackHistory(pedInSceneId) = pedIndexWithinScene;
                            pedInSceneId = pedInSceneId + 1;   % this index keeps track of all pedestrians in a scene
                            predPedIndex = predPedIndex + 1;   % this index keeps track of all pedestrians from all scenes               
                            % initialize the pedestrian
                            newPedestrianStruct = struct('sceneId',sceneId,'carTrackId',carTrackId,'pedcarTrackId',pedIndexWithinScene,'timeStep',[],'data',cell(1,1),'kfData',cell(1,1),'predictionModel',[]);                        
                            predictedPedTraj_CV{sceneId}{track_index,1}{pedIndexWithinScene,1} =  newPedestrianStruct;
                    end
                    %%%%%%%%%%%%%%%%%%%%%%%     

                    %% debug
                    if pedTrackTimeStep==actualTimeStep_interest
                        x=1;
                    end
                    %%%%%%%%%%%%%%%%%%%%%%%
                    
                    % Run the appropriate prediction framework if there is an ego car for the pedestrian, else run a constant velocity model
%                     if flag.EgoCar
                            if strcmp(predictionModel,"MultipleHybridPedestrian")
                                [pedPredictions, pedKFpredictions, predGapFeatures, predCrossFeatures] = func_predictStates(kf, currentPedData, currentPedMetaData, currentTSActiveCarData, carTrackCurrentTimeStepInPredictionData, AVStates, pedTrackTimeStep, ...
                                                                              cw, annotatedImageEnhanced, resetStates, Prob_GapAcceptanceModel, Prob_CrossIntentModelCar,Prob_CrossIntentModelNoCar, Params, flag);
                                 predModelForTimeStep = 2;
                            elseif strcmp(predictionModel,"BaselineHybrid")
                                [pedPredictions, pedKFpredictions, predGapFeatures] = func_predictStates_baseHybrid_v2(kf, currentPedData, currentPedMetaData, currentTSActiveCarData, carTrackCurrentTimeStepInPredictionData, AVStates, pedTrackTimeStep, ...
                                                                             cw, annotatedImageEnhanced, resetStates, Prob_GapAcceptanceModel, Params, flag);
                                 predModelForTimeStep = 1;
                            elseif strcmp(predictionModel,"ConstantVelocity")
                                [pedPredictions, pedKFpredictions] = func_HPed_CV(kf, currentPedData, Params, pedTrackTimeStep);
                                predModelForTimeStep = 0;
                            end                                         
                           % compile Gap features
                           if strcmp(predictionModel,"MultipleHybridPedestrian") || strcmp(predictionModel,"BaselineHybrid")
                                N_gaps = size(predGapFeatures,1);
                                if N_gaps >=1
                                    predGapFeatures.recordingId = sceneId;
                                    predGapFeatures.pedcarTrackId = pedIndexWithinScene;
                                    GapFeaturesAllScenes(GapFeatureId:GapFeatureId+N_gaps-1) =  predGapFeatures;
                                    GapFeatureId = GapFeatureId + N_gaps;
                                end
                           end
                            % compile Cross intent features
                            if strcmp(predictionModel,"MultipleHybridPedestrian")
                                N_cross = size(predCrossFeatures,1);
                                if N_cross >=1
                                    predCrossFeatures.recordingId = sceneId;
                                    predCrossFeatures.pedcarTrackId = pedIndexWithinScene;
                                    CrossFeaturesAllScenes(CrossFeatureId:CrossFeatureId+N_cross-1) = predCrossFeatures;
                                    CrossFeatureId = CrossFeatureId + N_cross;
                                end
                            end
%                     else
%                             % if there is no ego-car run a constant velocity prediction irrespective of the prediction model
%                             [pedPredictions, pedKFpredictions] = func_HPed_CV(kf, currentPedData, Params, pedTrackTimeStep);
%                             predModelForTimeStep = 0;
%                     end
                    %%%%%%%%%%%%%%%%%%%%%%%
                    %% debug
                    if pedTrackTimeStep==actualTimeStep_interest
                        x=1;
                    end
                    % Save the predictions
                    predictedPedTraj_CV{sceneId}{track_index,1}{pedIndexWithinScene,1}.timeStep(end+1,1) = pedTrackTimeStep;
                    predictedPedTraj_CV{sceneId}{track_index,1}{pedIndexWithinScene,1}.data{end+1,1} = pedPredictions;
                    predictedPedTraj_CV{sceneId}{track_index,1}{pedIndexWithinScene,1}.kfData{end+1,1} = pedKFpredictions;
                    predictedPedTraj_CV{sceneId}{track_index,1}{pedIndexWithinScene,1}.predictionModel(end+1,1) = predModelForTimeStep;
%                     end % debug end 
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
