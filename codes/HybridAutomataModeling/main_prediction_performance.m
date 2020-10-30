%% This script calculates the performance of the prediction algorithms

% Deterministic Metrics
% 1) Average displacement error
% 2) Final displacement error
% 3) Gap accpetance classification error
% 4) Crossing intent classification error

% Probabilistic Metrics
% 1) Predicted probability

p1 = genpath('G:\My Drive\Research\Projects\pedestrianHybridModel\codes');
p2 = genpath('G:\My Drive\Research\Projects\pedestrianHybridModel\datasets');
p3 = genpath('G:\My Drive\Research\Projects\pedestrianHybridModel\results');
% p1 = genpath('E:\jskumaar\pedestrianHybridModel\codes');
% p2 = genpath('E:\jskumaar\pedestrianHybridModel\datasets');
% p3 = genpath('E:\jskumaar\pedestrianHybridModel\results');
addpath(p1)
addpath(p2)
addpath(p3)

intersection_img = imread('18_background.png');
img_size = size(intersection_img);
 
% % load predictions
% load('trial_7_allScenes_MHP_10_26_clean.mat');
% load('trial4_allScenes_CV.mat');
% load('trial_6_allScenes_HBase_10_25_clean.mat');

% % run scripts
configure_MHP;
temp_crossStart_timesteps;

% parameters
orthopxToMeter = Params.orthopxToMeter;
scaleFactor = Params.scaleFactor;
N_scenes = 12;
event_TS = 30; % how far (in time steps) from the event (crossing or walked past crosswalk) should be considered for performance evaluation
N = 1000;

% initialize variables
ADE_MHP = [];
FDE_MHP = [];
ADE_HBase = [];
FDE_HBase = [];
ADE_CV = [];
FDE_CV = [];

MHP_good_ADE_performance_indices = [];
MHP_good_FDE_performance_indices = [];
MHP_bad_ADE_performance_indices = [];
MHP_bad_FDE_performance_indices = [];

PP_MHP = -1*ones(N,Params.predHorizon);
PP_HBase =  -1*ones(N,Params.predHorizon);
PP_CV =  -1*ones(N,Params.predHorizon);
LL_MHP =  -1*ones(N,Params.predHorizon);
LL_HBase =  -1*ones(N,Params.predHorizon);
LL_CV =  -1*ones(N,Params.predHorizon);
FSR_MHP = -1*ones(N,Params.predHorizon);
FSR_HBase = -1*ones(N,Params.predHorizon);
FSR_CV = -1*ones(N,Params.predHorizon);

% loop for all scenes
index = 1;

indices_imp = [1,108,303,34,88,9.28933333994465];

sceneId_interest = indices_imp(1);
car_index_interest = indices_imp(2);
pedTrackId_interest = indices_imp(3);
predictionTimeStep_interest = indices_imp(4);
actualTimeStep_interest = indices_imp(5);

figure()
% for sceneId = 1:N_Scenes
for sceneId = sceneId_interest
    
    N_car = size(predictedPedTraj_MHP{sceneId},1);
    % for every ego-car
%     for car_index = 1:N_car
    for car_index = car_index_interest
        % if there is a car
        if ~isempty(predictedPedTraj_MHP{sceneId}{car_index})
            % predictions of all pedestrians who interacted with the ego-car for
            % all three kinds of models
            pedPredictionForEachCar_MHP = predictedPedTraj_MHP{sceneId}{car_index};
            pedPredictionForEachCar_HBase = predictedPedTraj_HBase{sceneId}{car_index};
            pedPredictionForEachCar_CV = predictedPedTraj_CV{sceneId}{car_index};
            % no. of pedestrians interacted with the ego-car
            N_ped_pred = size(pedPredictionForEachCar_MHP,1); %if there are no predictions for a pedestrian, that entry is empty
            % for all pedestrians
%             for pedTrackId = 1:N_ped_pred
            for pedTrackId = pedTrackId_interest
                % if there was a crossing event or reached Crosswalk event
                % in that pedestrian trajectory, compare the prediction performance
                if ~isempty(crossStartTimeSteps{sceneId}{pedTrackId}) || ~isempty(crossedCWTimeSteps{sceneId}{pedTrackId})
                
                        % pedestrian ground truth data
                        GTPedData = formattedTracksData{sceneId}{pedTrackId};
                        N_GTTimeSteps = size(GTPedData.frame, 1);
                        % pedestrian prediction data
                        pedPredictions_MHP = pedPredictionForEachCar_MHP{pedTrackId}; % the first data is a place holder, 'inf' value
                        pedPredictions_HBase = pedPredictionForEachCar_HBase{pedTrackId};
                        pedPredictions_CV = pedPredictionForEachCar_CV{pedTrackId};

                        % if the pedestrian was an active pedestrian (only
                        % time step of inactive pedestrians is 'inf')
                        if ~isempty(pedPredictions_MHP) && (pedPredictions_MHP.timeStep(end)~=inf)
                            pedPredictionsData_MHP = pedPredictions_MHP.data(2:end); %the first entry in pedPredictions.data is a dummy entry
                            pedKFPredictionsData_MHP = pedPredictions_MHP.kfData(2:end);

                            pedPredictionsData_HBase = pedPredictions_HBase.data(2:end); %the first entry in pedPredictions.data is a dummy entry
                            pedKFPredictionsData_HBase = pedPredictions_HBase.kfData(2:end);

                            pedPredictionsData_CV = pedPredictions_CV.data(2:end); %the first entry in pedPredictions.data is a dummy entry
                            pedKFPredictionsData_CV = pedPredictions_CV.kfData(2:end);

                            pedPredictionsTime = pedPredictions_MHP.timeStep(2:end);
                            N_timeSteps = size(pedPredictionsData_MHP, 1);
                            % for all prediction time steps; note that this
                            % may not correspond to the actual time steps of the pedestrian track
%                             for predictionTimeStep = 1:N_timeSteps 
                            for predictionTimeStep = predictionTimeStep_interest
                                %check if its a interesting timestep
                                N = find(indices_interest_filtered(:,1)==sceneId & indices_interest_filtered(:,2)==pedTrackId & predictionTimeStep >= indices_interest_filtered(:,3)-event_TS);
                                if ~isempty(N)
                                    eventFlag = 1;
                                else
                                    eventFlag = 0;
                                end

                                % reset variables
                                mostProbablePredictedTrajectory_MHP = [];
                                mostProbablePredictedTrajectory_HBase = [];
                                mostProbablePredictedTrajectory_CV = [];
                                % if there is an event
                                if eventFlag
                                    %%%%%%%%%%%%%%%%%%%%%%%%%%%
                                    % MHP model: get the most probable trajectory
                                    N_futures_MHP = size(pedPredictionsData_MHP{predictionTimeStep},1);  
                                   [~, mostProbablePredictionId_MHP] = max(pedPredictionsData_MHP{predictionTimeStep}(:,2));  %probability of the tracks is in the second column

                                    for predId = 1:N_futures_MHP
                                        temp = pedPredictionsData_MHP{predictionTimeStep}(predId, :);

                                        if mod([length(temp)/2 - 1],1) == 0
                                            temp = reshape(temp(3:end), [2, length(temp)/2 - 1])';
                                            tempIsMHPUsed = true;
                                        else
                                            temp = reshape(temp(2:end), [2, floor(length(temp)/2) - 1])';
                                            tempIsMHPUsed = false;
                                        end
                                        mostProbablePredictedTrajectory_MHP(predId,:,:) = temp([end-Params.predHorizon+1:end], :);
                                    end

                                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                    % Base Hybrid predictions
                                    N_futures_HBase = size(pedPredictionsData_HBase{predictionTimeStep},1);  
                                   [~, mostProbablePredictionId_HBase] = max(pedPredictionsData_HBase{predictionTimeStep}(:,2));  %probability of the tracks is in the second column

                                    for predId = 1:N_futures_HBase
                                        temp = pedPredictionsData_HBase{predictionTimeStep}(predId, :);

                                        if mod([length(temp)/2 - 1],1) == 0
                                            temp = reshape(temp(3:end), [2, length(temp)/2 - 1])';
                                            tempIsMHPUsed = true;
                                        else
                                            temp = reshape(temp(2:end), [2, floor(length(temp)/2) - 1])';
                                            tempIsMHPUsed = false;
                                        end
                                        mostProbablePredictedTrajectory_HBase(predId,:,:) = temp([end-Params.predHorizon+1:end], :);
                                    end

                                    
                                    % constant velocity predictions
                                    mostProbablePredictedTrajectory_CV = pedPredictionsData_CV{predictionTimeStep};
                                    mostProbablePredictedTrajectory_CV = reshape(mostProbablePredictedTrajectory_CV{1}(3:end), [2, Params.predHorizon])';

                                    % ground truth trajectory
                                    startTimeStep = pedPredictionsTime(predictionTimeStep) + 1;
                                    endTimeStep = min(startTimeStep + Params.predHorizon -1, N_GTTimeSteps);                
                                    GTTrajectory = [GTPedData.xCenter(startTimeStep:endTimeStep), GTPedData.yCenter(startTimeStep:endTimeStep)];
                                    N_PredTimeSteps = size(GTTrajectory,1);
                                    % if the MHP was different than
                                    % constant velocity
%                                         N_size = size(mostProbablePredictedTrajectory_MHP);
%                                         if N_size(1)~=1 | (N_size(1)==1 & sum(abs(mostProbablePredictedTrajectory_CV-reshape(mostProbablePredictedTrajectory_MHP(1,:,:),[N_size(2), N_size(3)])))~=0 )
                                     if N_futures_MHP>1  && N_PredTimeSteps >=1

                                        % trajectory error
                                        
                                        tempError_MHP = GTTrajectory - reshape(mostProbablePredictedTrajectory_MHP(mostProbablePredictionId_MHP,1:N_PredTimeSteps,:),[N_PredTimeSteps,2]);
                                        predError_MHP = vecnorm(tempError_MHP, 2, 2);
                                        tempError_HBase = GTTrajectory - reshape(mostProbablePredictedTrajectory_HBase(mostProbablePredictionId_HBase,1:N_PredTimeSteps,:),[N_PredTimeSteps,2]);
                                        predError_HBase = vecnorm(tempError_HBase, 2, 2);
                                        tempError_CV = GTTrajectory - mostProbablePredictedTrajectory_CV(1:N_PredTimeSteps,:);
                                        predError_CV = vecnorm(tempError_CV, 2, 2);
                                        
%                                         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                         % plot interesting trajectories
% % 
%                                         if (predError_MHP(1) > 0.2 || predError_MHP(end) > 6)
%                                                 plot_trajectory.m
%                                         end
%                                         %%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                        
%                 %                        deterministic error metrics for the entire prediction horizon
%                                         ADE_MHP(index, 1:N_PredTimeSteps) = cumsum(predError_MHP)./(1:N_PredTimeSteps)';
%                                         FDE_MHP(index, 1:N_PredTimeSteps) = predError_MHP;
%                                         ADE_HBase(index, 1:N_PredTimeSteps) = cumsum(predError_HBase)./(1:N_PredTimeSteps)';
%                                         FDE_HBase(index, 1:N_PredTimeSteps) = predError_HBase;
%                                         ADE_CV(index, 1:N_PredTimeSteps) = cumsum(predError_CV)./(1:N_PredTimeSteps)';
%                                         FDE_CV(index, 1:N_PredTimeSteps) =  predError_CV;                       
% 
%                                         averageDistanceError_MHP{sceneId}{car_index}{pedTrackId}{predictionTimeStep} = cumsum(predError_MHP)./cumsum(1:N_PredTimeSteps,1)';
%                                         finalDistanceError_MHP{sceneId}{car_index}{pedTrackId}{predictionTimeStep} = predError_MHP;
%                                         averageDistanceError_HBase{sceneId}{car_index}{pedTrackId}{predictionTimeStep} = cumsum(predError_HBase)./cumsum(1:N_PredTimeSteps,1)';
%                                         finalDistanceError_HBase{sceneId}{car_index}{pedTrackId}{predictionTimeStep} = predError_HBase;
%                                         averageDistanceError_CV{sceneId}{car_index}{pedTrackId}{predictionTimeStep} = cumsum(predError_CV)./cumsum(1:N_PredTimeSteps,1)';
%                                         finalDistanceError_CV{sceneId}{car_index}{pedTrackId}{predictionTimeStep} = predError_CV;
%                                         isMHPUsed{sceneId}{car_index}{pedTrackId}(predictionTimeStep,1) = tempIsMHPUsed;
% % 
%                                         round(mean(FDE_MHP,1),1)
%                                         round(mean(FDE_HBase,1),1)
%                                         round(mean(FDE_CV,1),1)
                                        
                                        
%                                         %% note down interesting indices
%                                         % better performance than constant
%                                         % velocity
%                                         if ADE_MHP(index, N_PredTimeSteps) < ADE_CV(index, N_PredTimeSteps)
%                                             perf_difference = ADE_CV(index, N_PredTimeSteps) - ADE_MHP(index, N_PredTimeSteps);
%                                             MHP_good_ADE_performance_indices = [MHP_good_ADE_performance_indices; [sceneId, car_index,pedTrackId, predictionTimeStep, pedPredictionsTime(predictionTimeStep), perf_difference]];
%                                         elseif ADE_MHP(index, N_PredTimeSteps) > 2*ADE_CV(index, N_PredTimeSteps)
%                                             perf_difference = ADE_MHP(index, N_PredTimeSteps) - ADE_CV(index, N_PredTimeSteps);
%                                             MHP_bad_ADE_performance_indices = [MHP_bad_ADE_performance_indices; [sceneId, car_index,pedTrackId, predictionTimeStep, pedPredictionsTime(predictionTimeStep), perf_difference]];
%                                         end
%                                         
%                                         
%                                         if FDE_MHP(index, N_PredTimeSteps) < FDE_CV(index, N_PredTimeSteps)
%                                             perf_difference = FDE_CV(index, N_PredTimeSteps) - FDE_MHP(index, N_PredTimeSteps);
%                                             MHP_good_FDE_performance_indices = [MHP_good_FDE_performance_indices; [sceneId, car_index,pedTrackId, predictionTimeStep, pedPredictionsTime(predictionTimeStep), perf_difference]];
%                                         elseif FDE_MHP(index, N_PredTimeSteps) > 2*FDE_CV(index, N_PredTimeSteps)
%                                             perf_difference = FDE_MHP(index, N_PredTimeSteps) - ADE_MHP(index, N_PredTimeSteps);
%                                             MHP_bad_FDE_performance_indices = [MHP_bad_FDE_performance_indices; [sceneId, car_index,pedTrackId, predictionTimeStep, pedPredictionsTime(predictionTimeStep), perf_difference]];
%                                         end
%                                         
                                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                                    %% probabiistic error metrics
                                    type = 'Not_CV';
                                    [predictedProbability_MHP{sceneId}{car_index}{pedTrackId}{predictionTimeStep}, likelihood_MHP{sceneId}{car_index}{pedTrackId}{predictionTimeStep}, FSR_ratio_MHP{sceneId}{car_index}{pedTrackId}{predictionTimeStep}, prob2D_MHP{sceneId}{car_index}{pedTrackId}{predictionTimeStep}] = probMetrics(GTTrajectory, pedPredictionsData_MHP{predictionTimeStep}, pedKFPredictionsData_MHP{predictionTimeStep}, Params, type);
                                    [predictedProbability_HBase{sceneId}{car_index}{pedTrackId}{predictionTimeStep}, likelihood_HBase{sceneId}{car_index}{pedTrackId}{predictionTimeStep}, FSR_ratio_HBase{sceneId}{car_index}{pedTrackId}{predictionTimeStep}, prob2D_HBase{sceneId}{car_index}{pedTrackId}{predictionTimeStep}] = probMetrics(GTTrajectory, pedPredictionsData_HBase{predictionTimeStep}, pedKFPredictionsData_HBase{predictionTimeStep}, Params, type);
                                    type = 'CV';
                                    [predictedProbability_CV{sceneId}{car_index}{pedTrackId}{predictionTimeStep},  likelihood_CV{sceneId}{car_index}{pedTrackId}{predictionTimeStep}, FSR_ratio_CV{sceneId}{car_index}{pedTrackId}{predictionTimeStep}, prob2D_CV{sceneId}{car_index}{pedTrackId}{predictionTimeStep}] = probMetrics(GTTrajectory, pedPredictionsData_CV{predictionTimeStep}, pedKFPredictionsData_CV{predictionTimeStep}, Params, type);
                                    
                                    % plot predicted probability
                                    plot_trajectory_on_image.m
                                                                        
                                    % predict1`ed probability of ground truth
                                    N = length(predictedProbability_MHP{sceneId}{car_index}{pedTrackId}{predictionTimeStep});
                                    PP_MHP(index, 1:N) = predictedProbability_MHP{sceneId}{car_index}{pedTrackId}{predictionTimeStep};
                                    PP_HBase(index, 1:N) = predictedProbability_HBase{sceneId}{car_index}{pedTrackId}{predictionTimeStep};
                                    PP_CV(index, 1:N) = predictedProbability_CV{sceneId}{car_index}{pedTrackId}{predictionTimeStep};
                                    % likelihood of ground truth being among the
                                    % predictions
                                    LL_MHP(index, 1:N) = [ likelihood_MHP{sceneId}{car_index}{pedTrackId}{predictionTimeStep}];
                                    LL_HBase(index, 1:N) = [ likelihood_HBase{sceneId}{car_index}{pedTrackId}{predictionTimeStep}];
                                    LL_CV(index, 1:N) = [ likelihood_CV{sceneId}{car_index}{pedTrackId}{predictionTimeStep}];
                                    % ratio between non-zero predictions and Forward reachable
                                    % set 
                                    FSR_MHP(index, 1:N) = [ FSR_ratio_MHP{sceneId}{car_index}{pedTrackId}{predictionTimeStep}];
                                    FSR_HBase(index, 1:N) = [ FSR_ratio_HBase{sceneId}{car_index}{pedTrackId}{predictionTimeStep}];
                                    FSR_CV(index, 1:N) = [ FSR_ratio_CV{sceneId}{car_index}{pedTrackId}{predictionTimeStep}];
                                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                     
                                    % update index number
                                        index = index+1;
                                     end

                                end            
                        
                            end  % end of prediction horizon
                    
                  
                        end
                end
            end  % end of all pedestrians tracks
        end
 
    end
    
end
