%% This script calculates the performance of the prediction algorithms
% Note: We calculate both deterministic and probabilistic metrics.
% Deterministic Metrics:
% 1) Average displacement error for a) most probable trajectory, b) closest
% trajectory (best prediction)
% 2) Final displacement error a) most probable trajectory, b) closest
% trajectory (best prediction)
% 3) Gap accpetance classification error
% 4) Crossing intent classification error

% Probabilistic Metrics:
% 1) Predicted probability
% 2) Likelihood of ground truth prediction
% 3) Ratio to Forward Reachable set
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% setup
% path
% p1 = genpath('G:\My Drive\Research\Projects\pedestrianHybridModel\codes');
% p2 = genpath('G:\My Drive\Research\Projects\pedestrianHybridModel\datasets');
% p3 = genpath('G:\My Drive\Research\Projects\pedestrianHybridModel\results');
p1 = genpath('E:\jskumaar\pedestrianHybridModel\codes');
p2 = genpath('E:\jskumaar\pedestrianHybridModel\datasets');
p3 = genpath('E:\jskumaar\pedestrianHybridModel\results');
addpath(p1)
addpath(p2)
addpath(p3)
%%%%%%%%%%%%%%%%%%%%%
% load files
intersection_img = imread('18_background.png');
img_size = size(intersection_img);
% % load predictions of MHP and HBase and CV baselines
% load('trial_9_oneScene_MHP_11_05_clean.mat');
% load('trial_8_oneScene_CV_11_03_clean.mat');
% load('trial_9_oneScene_HBase_11_05_clean.mat');
%%%%%%%%%%%%%%%%%%%%
% configure model parameters
script_configureMHP;
% identify event indices
temp_crossStart_timesteps;
% parameters
orthopxToMeter = Params.orthopxToMeter;
scaleFactor = Params.scaleFactor;
N_scenes = 12;
event_TS = 30; % how far (in time steps) from the event (crossing or walked past crosswalk) should be considered for performance evaluation
N = 1000;
%%%%%%%%%%%%%%%%%%%%%

%% initialize variables
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
MHP_good_bestADE_performance_indices = [];
MHP_bad_bestADE_performance_indices = [];

PP_MHP = -1*ones(N,Params.predHorizon);
PP_HBase =  -1*ones(N,Params.predHorizon);
PP_CV =  -1*ones(N,Params.predHorizon);
LL_MHP =  -1*ones(N,Params.predHorizon);
LL_HBase =  -1*ones(N,Params.predHorizon);
LL_CV =  -1*ones(N,Params.predHorizon);
FSR_MHP = -1*ones(N,Params.predHorizon);
FSR_HBase = -1*ones(N,Params.predHorizon);
FSR_CV = -1*ones(N,Params.predHorizon);


% debug
indices_imp = [1,60,215,3,4,7.30840599107940];

sceneId_interest = indices_imp(1);
car_index_interest = indices_imp(2);
pedTrackId_interest = indices_imp(3);
predictionTimeStep_interest = indices_imp(4);
actualTimeStep_interest = indices_imp(5);

%% loop starts
index = 1;
% figure()
%% debug
for sceneId = 1:1
% for sceneId = sceneId_interest    
    N_car = size(predictedPedTraj_HBase{sceneId},1);
    % for every ego-car
    %% debug
    for car_index = 42:N_car
%     for car_index = car_index_interest
        % if there is a car
        if ~isempty(predictedPedTraj_MHP{sceneId}{car_index})
            % predictions of all pedestrians who interacted with the ego-car for
            % all three kinds of models
            pedPredictionForEachCar_MHP = predictedPedTraj_MHP{sceneId}{car_index+1};
            pedPredictionForEachCar_HBase = predictedPedTraj_HBase{sceneId}{car_index};
            pedPredictionForEachCar_CV = predictedPedTraj_CV{sceneId}{car_index+1};
            % no. of pedestrians interacted with the ego-car
            N_ped_pred = size(pedPredictionForEachCar_MHP,1); %if there are no predictions for a pedestrian, that entry is empty
            % for all pedestrians
            %% debug
            for pedTrackId = 1:N_ped_pred
%             for pedTrackId = pedTrackId_interest
                % if there was a crossing event or reached Crosswalk event
                % in that pedestrian trajectory, compare the prediction performance
                if ~isempty(crossStartTimeSteps{sceneId}{pedTrackId}) || ~isempty(crossedCWTimeSteps{sceneId}{pedTrackId})                
                        % pedestrian ground truth data
                        GTPedData = formattedTracksData{sceneId}{pedTrackId};
                        N_GTTimeSteps = size(GTPedData.frame, 1);
                        % pedestrian prediction data
                        pedPredictions_MHP = pedPredictionForEachCar_MHP{pedTrackId}; 
                        pedPredictions_HBase = pedPredictionForEachCar_HBase{pedTrackId};
                        pedPredictions_CV = pedPredictionForEachCar_CV{pedTrackId};

                        % if the pedestrian was an active pedestrian (only
                        % time step of inactive pedestrians is 'inf')
                        if ~isempty(pedPredictions_MHP) && (pedPredictions_MHP.timeStep(end)~=inf)
                            pedPredictionsData_MHP = pedPredictions_MHP.data; 
                            pedKFPredictionsData_MHP = pedPredictions_MHP.kfData;
% 
                            pedPredictionsData_HBase = pedPredictions_HBase.data; 
                            pedKFPredictionsData_HBase = pedPredictions_HBase.kfData;

                            pedPredictionsData_CV = pedPredictions_CV.data; 
                            pedKFPredictionsData_CV = pedPredictions_CV.kfData;
                            
                            % include non-zero CV predictions
                            for jj = 1:size(pedPredictionsData_MHP,1)
                                temp = pedPredictionsData_CV{jj};
                                temp{1}(1,2) = 0.1;
                                pedPredictionsData_MHP{jj}(:,2) = pedPredictionsData_MHP{jj}(:,2)*0.9 ;                                
                                pedPredictionsData_MHP{jj} = [pedPredictionsData_MHP{jj}; temp{1}];
                                
                                temp = pedKFPredictionsData_CV{jj};
                                pedKFPredictionsData_MHP{jj} = [pedKFPredictionsData_MHP{jj}; temp];
                            end
                            

                            pedPredictionsTime = pedPredictions_MHP.timeStep;
                            if ~isempty(pedPredictionsTime)
                                N_timeSteps = size(pedPredictionsData_MHP, 1);
                            else
                                N_timeSteps = 0;
                            end
                            % for all prediction time steps; note that this
                            % may not correspond to the actual time steps of the pedestrian track
                            %% debug
                            for predictionTimeStep = 1:N_timeSteps 
%                             for predictionTimeStep = predictionTimeStep_interest
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
                                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                    % ground truth trajectory
                                    startTimeStep = pedPredictionsTime(predictionTimeStep) + 1;
                                    endTimeStep = min(startTimeStep + Params.predHorizon -1, N_GTTimeSteps);                
                                    GTTrajectory = [GTPedData.xCenter(startTimeStep:endTimeStep), GTPedData.yCenter(startTimeStep:endTimeStep)];
                                    GT_currentPos = [GTPedData.xCenter(startTimeStep-1), GTPedData.yCenter(startTimeStep-1)];
                                    N_PredTimeSteps = size(GTTrajectory,1);
                                    %%%%%%%%%%%%%%%%%%%%%%%%%%%
                                    if ~isempty(GTTrajectory)
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
                                            mostProbablePredictedTrajectory_MHP(predId,:,:) = temp(1:N_PredTimeSteps, :);
                                        end
                                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                        % MHP model: get the best trajectory
                                        % (based on ADE on last time step)
                                        min_ADE = inf;
                                        for predId = 1:N_futures_MHP
                                            temp = pedPredictionsData_MHP{predictionTimeStep}(predId, :);
    %                                         if mod([length(temp)/2 - 1],1) == 0
                                                temp = reshape(temp(3:end), [2, length(temp)/2 - 1])';
                                                tempIsMHPUsed = true;
    %                                         else
    %                                             temp = reshape(temp(2:end), [2, floor(length(temp)/2) - 1])';
    %                                             tempIsMHPUsed = false;
    %                                         end
                                            tempError_MHP = GTTrajectory - reshape(temp(1:N_PredTimeSteps, :),[N_PredTimeSteps,2]);
                                            temp_predError_MHP = vecnorm(tempError_MHP, 2, 2);
                                            temp_ADE = cumsum(temp_predError_MHP)./(1:N_PredTimeSteps)';
                                            if temp_ADE(end) < min_ADE
                                                bestMHP_id = predId;
                                                min_ADE = temp_ADE(end);
                                            end    
                                        end
                                        temp =  pedPredictionsData_MHP{predictionTimeStep}(bestMHP_id, :);
                                        bestPredictedTrajectory_MHP = reshape(temp(3:2*(N_PredTimeSteps+1)),[2, N_PredTimeSteps])';
                                        bestPredError_MHP = vecnorm(GTTrajectory - bestPredictedTrajectory_MHP, 2, 2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                        % Base Hybrid predictions
                                        N_futures_HBase = size(pedPredictionsData_HBase{predictionTimeStep},1);  
                                       [~, mostProbablePredictionId_HBase] = max(pedPredictionsData_HBase{predictionTimeStep}(:,2));  %probability of the tracks is in the second column
    
                                        for predId = 1:N_futures_HBase
                                            temp = pedPredictionsData_HBase{predictionTimeStep}(predId, :);
    
%                                             if mod([length(temp)/2 - 1],1) == 0
                                                temp = reshape(temp(3:end), [2, length(temp)/2 - 1])';
                                                tempIsMHPUsed = true;
%                                             else
%                                                 temp = reshape(temp(2:end), [2, floor(length(temp)/2) - 1])';
%                                                 tempIsMHPUsed = false;
%                                             end

                                            mostProbablePredictedTrajectory_HBase(predId,:,:) = temp(1:N_PredTimeSteps, :);
                                        end

                                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                        % constant velocity predictions
                                        mostProbablePredictedTrajectory_CV = pedPredictionsData_CV{predictionTimeStep};
                                        mostProbablePredictedTrajectory_CV = reshape(mostProbablePredictedTrajectory_CV{1}(3:2*(N_PredTimeSteps+1)), [2, N_PredTimeSteps])';
                                        % if the MHP was different than
                                        % constant velocity
                                        if N_futures_MHP>1  && N_PredTimeSteps >=1
                                            % trajectory error
                                            tempError_MHP = GTTrajectory - reshape(mostProbablePredictedTrajectory_MHP(mostProbablePredictionId_MHP,1:N_PredTimeSteps,:),[N_PredTimeSteps,2]);
                                            predError_MHP = vecnorm(tempError_MHP, 2, 2);
                                            tempError_HBase = GTTrajectory - reshape(mostProbablePredictedTrajectory_HBase(mostProbablePredictionId_HBase,1:N_PredTimeSteps,:),[N_PredTimeSteps,2]);
                                            predError_HBase = vecnorm(tempError_HBase, 2, 2);
                                            tempError_CV = GTTrajectory - mostProbablePredictedTrajectory_CV(1:N_PredTimeSteps,:);
                                            predError_CV = vecnorm(tempError_CV, 2, 2);
                                            % deterministic error metrics for the entire prediction horizon
                                            ADE_MHP(index, 1:N_PredTimeSteps) = cumsum(predError_MHP)./(1:N_PredTimeSteps)';
                                            FDE_MHP(index, 1:N_PredTimeSteps) = predError_MHP;
                                            ADE_HBase(index, 1:N_PredTimeSteps) = cumsum(predError_HBase)./(1:N_PredTimeSteps)';
                                            FDE_HBase(index, 1:N_PredTimeSteps) = predError_HBase;
                                            ADE_CV(index, 1:N_PredTimeSteps) = cumsum(predError_CV)./(1:N_PredTimeSteps)';
                                            FDE_CV(index, 1:N_PredTimeSteps) =  predError_CV;                       
                                            best_ADE_MHP(index, 1:N_PredTimeSteps) = cumsum(bestPredError_MHP)./(1:N_PredTimeSteps)';

                                            %% note down interesting indices
                                            % better performance than constant
                                            % velocity
                                            if ADE_MHP(index, N_PredTimeSteps) < ADE_CV(index, N_PredTimeSteps)
                                                perf_difference = ADE_CV(index, N_PredTimeSteps) - ADE_MHP(index, N_PredTimeSteps);
                                                MHP_good_ADE_performance_indices = [MHP_good_ADE_performance_indices; [sceneId, car_index,pedTrackId, predictionTimeStep, pedPredictionsTime(predictionTimeStep), perf_difference]];
                                            elseif ADE_MHP(index, N_PredTimeSteps) > 2*ADE_CV(index, N_PredTimeSteps)
                                                perf_difference = ADE_MHP(index, N_PredTimeSteps) - ADE_CV(index, N_PredTimeSteps);
                                                MHP_bad_ADE_performance_indices = [MHP_bad_ADE_performance_indices; [sceneId, car_index,pedTrackId, predictionTimeStep, pedPredictionsTime(predictionTimeStep), perf_difference]];
                                            end
                                            
                                            if best_ADE_MHP(index, N_PredTimeSteps) < ADE_CV(index, N_PredTimeSteps)
                                                perf_difference = ADE_CV(index, N_PredTimeSteps) - best_ADE_MHP(index, N_PredTimeSteps);
                                                MHP_good_bestADE_performance_indices = [MHP_good_bestADE_performance_indices; [sceneId, car_index,pedTrackId, predictionTimeStep, pedPredictionsTime(predictionTimeStep), perf_difference]];
                                            elseif best_ADE_MHP(index, N_PredTimeSteps) > 2*ADE_CV(index, N_PredTimeSteps)
                                                perf_difference = best_ADE_MHP(index, N_PredTimeSteps) - ADE_CV(index, N_PredTimeSteps);
                                                MHP_bad_bestADE_performance_indices = [MHP_bad_bestADE_performance_indices; [sceneId, car_index,pedTrackId, predictionTimeStep, pedPredictionsTime(predictionTimeStep), perf_difference]];
                                            end

                                            if FDE_MHP(index, N_PredTimeSteps) < FDE_CV(index, N_PredTimeSteps)
                                                perf_difference = FDE_CV(index, N_PredTimeSteps) - FDE_MHP(index, N_PredTimeSteps);
                                                MHP_good_FDE_performance_indices = [MHP_good_FDE_performance_indices; [sceneId, car_index,pedTrackId, predictionTimeStep, pedPredictionsTime(predictionTimeStep), perf_difference]];
                                            elseif FDE_MHP(index, N_PredTimeSteps) > 2*FDE_CV(index, N_PredTimeSteps)
                                                perf_difference = FDE_MHP(index, N_PredTimeSteps) - ADE_MHP(index, N_PredTimeSteps);
                                                MHP_bad_FDE_performance_indices = [MHP_bad_FDE_performance_indices; [sceneId, car_index,pedTrackId, predictionTimeStep, pedPredictionsTime(predictionTimeStep), perf_difference]];
                                            end

                                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                                        %% probabiistic error metrics
%                                         [predictedProbability_MHP{sceneId}{car_index}{pedTrackId}{predictionTimeStep}, likelihood_MHP{sceneId}{car_index}{pedTrackId}{predictionTimeStep}, FSR_ratio_MHP{sceneId}{car_index}{pedTrackId}{predictionTimeStep}, prob2D_MHP{sceneId}{car_index}{pedTrackId}{predictionTimeStep}] = func_probMetrics(GT_currentPos, GTTrajectory, pedPredictionsData_MHP{predictionTimeStep}, pedKFPredictionsData_MHP{predictionTimeStep}, Params, type);
%                                   
                                        %% probabiistic error metrics
                                        type = 'Not_CV';    
                                        [predictedProbability_MHP, likelihood_MHP, FSR_ratio_MHP, prob2D_MHP] = func_probMetrics(GT_currentPos, GTTrajectory, pedPredictionsData_MHP{predictionTimeStep}, pedKFPredictionsData_MHP{predictionTimeStep}, Params, type);
                                        [predictedProbability_HBase, likelihood_HBase, FSR_ratio_HBase, prob2D_HBase] = func_probMetrics(GT_currentPos, GTTrajectory, pedPredictionsData_HBase{predictionTimeStep}, pedKFPredictionsData_HBase{predictionTimeStep}, Params, type);
                                        type = 'CV';
                                        [predictedProbability_CV,  likelihood_CV, FSR_ratio_CV, prob2D_CV] = func_probMetrics(GT_currentPos, GTTrajectory, pedPredictionsData_CV{predictionTimeStep}, pedKFPredictionsData_CV{predictionTimeStep}, Params, type);
                                        
    %                                     % plot predicted probability
    %                                     plot_trajectory_on_image.m
    %                                                                         
                                        % predict1`ed probability of ground truth
                                        N = length(predictedProbability_MHP);
                                        PP_MHP(index, 1:N) = predictedProbability_MHP;
                                        PP_HBase(index, 1:N) = predictedProbability_HBase;
                                        PP_CV(index, 1:N) = predictedProbability_CV;
                                        % likelihood of ground truth being among the
                                        % predictions
                                        LL_MHP(index, 1:N) =  likelihood_MHP;
                                        LL_HBase(index, 1:N) =  likelihood_HBase;
                                        LL_CV(index, 1:N) = likelihood_CV;
                                        % ratio between non-zero predictions and Forward reachable
                                        % set 
                                        FSR_MHP(index, 1:N) = FSR_ratio_MHP;
                                        FSR_HBase(index, 1:N) =  FSR_ratio_HBase;
                                        FSR_CV(index, 1:N) =  FSR_ratio_CV;
                                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%                                        % update index number
                                        index = index+1;
                                            
                                        end % if there is a GT reference
                                     end

                                end            
                        
                            end  % end of prediction horizon
                    
                  
                        end
                end
                
               
                
                
            end  % end of all pedestrians tracks
        end
 
    end
    
end


%%%%%%%
[~,idx] = sortrows(MHP_good_ADE_performance_indices(:,6),'descend');
MHP_good_ADE_sorted = MHP_good_ADE_performance_indices(idx,:);
[~,idx2] = sortrows(MHP_good_FDE_performance_indices(:,6),'descend');
MHP_good_FDE_sorted = MHP_good_FDE_performance_indices(idx2,:);

[~,idx] = sortrows(MHP_bad_ADE_performance_indices(:,6),'descend');
MHP_bad_ADE_sorted = MHP_bad_ADE_performance_indices(idx,:);
[~,idx2] = sortrows(MHP_bad_FDE_performance_indices(:,6),'descend');
MHP_bad_FDE_sorted = MHP_bad_FDE_performance_indices(idx2,:);

[~,idx2] = sortrows(MHP_good_bestADE_performance_indices(:,6),'descend');
MHP_good_bestADE_sorted = MHP_good_bestADE_performance_indices(idx2,:);
% [~,idx] = sortrows(MHP_bad_bestADE_performance_indices(:,6),'descend');
% MHP_bad_bestADE_sorted = MHP_bad_bestADE_performance_indices(idx,:);

%%%%%%%%%%%%
MHP_bad_ADE_sorted_filtered = MHP_bad_ADE_sorted;
[~,ind] = unique(MHP_bad_ADE_sorted_filtered(:,[3,5]),'rows');
MHP_bad_ADE_sorted_filtered = MHP_bad_ADE_sorted_filtered(ind,:);
temp = diff(MHP_bad_ADE_sorted_filtered(:,[3,5]));

ind = find(temp(:,1)~=0 | abs(temp(:,2)) > 5);
ind  =ind+1;

MHP_bad_ADE_sorted_filtered = MHP_bad_ADE_sorted_filtered(ind+1,:);
[~,idx] = sortrows(MHP_bad_ADE_sorted_filtered(:,6),'descend');
MHP_bad_ADE_sorted_filtered = MHP_bad_ADE_sorted_filtered(idx,:);
%%%%%%%%%%%%
MHP_bad_bestADE_sorted_filtered = MHP_bad_bestADE_sorted;
[~,ind] = unique(MHP_bad_bestADE_sorted_filtered(:,[3,5]),'rows');
MHP_bad_bestADE_sorted_filtered = MHP_bad_bestADE_sorted_filtered(ind,:);
temp = diff(MHP_bad_bestADE_sorted_filtered(:,[3,5]));

ind = find(temp(:,1)~=0 | abs(temp(:,2)) > 5);
ind  =ind+1;

MHP_bad_bestADE_sorted_filtered = MHP_bad_bestADE_sorted_filtered(ind+1,:);
[~,idx] = sortrows(MHP_bad_bestADE_sorted_filtered(:,6),'descend');
MHP_bad_bestADE_sorted_filtered = MHP_bad_bestADE_sorted_filtered(idx,:);
%%%%%%%%%%5
