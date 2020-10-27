%% This script calculates the performance of the prediction algorithms

% Deterministic Metrics
% 1) Average displacement error
% 2) Final displacement error
% 3) Gap accpetance classification error
% 4) Crossing intent classification error

% Probabilistic Metrics
% 1) Predicted probability

% p1 = genpath('G:\My Drive\Research\Projects\pedestrianHybridModel\codes');
% p2 = genpath('G:\My Drive\Research\Projects\pedestrianHybridModel\datasets');
% p3 = genpath('G:\My Drive\Research\Projects\pedestrianHybridModel\results');
p1 = genpath('E:\jskumaar\pedestrianHybridModel\codes');
p2 = genpath('E:\jskumaar\pedestrianHybridModel\datasets');
p3 = genpath('E:\jskumaar\pedestrianHybridModel\results');
addpath(p1)
addpath(p2)
addpath(p3)
% 
% % load predictions
% load('trial_5_Scene4_MHP.mat');
% predictedPedTraj_MHP = predictedPedTraj;
% load('trial_2_allScenes_CV.mat');
% load('trial_2_allScenes_baseHybrid.mat');
% 
% % load dataset
% load('tracksData_reSampled_v11.mat');


% % run scripts
configure_MHP;
temp_crossStart_timesteps;

orthopxToMeter = Params.orthopxToMeter;
scaleFactor = Params.scaleFactor;
% parameters
N_scenes = 12;
event_TS = 15;

error_compilation = true;
if error_compilation
    
% initialize
ADE_MHP = [];
FDE_MHP = [];
ADE_HBase = [];
FDE_HBase = [];
ADE_CV = [];
FDE_CV = [];

N = 1000;

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
for sceneId = 1:N_scenes
    
    N_car = size(predictedPedTraj_MHP{sceneId},1);
    for car_index = 1:N_car
        % if there is a car
        if ~isempty(predictedPedTraj_MHP{sceneId}{car_index})
            
            pedPredictionForEachCar_MHP = predictedPedTraj_MHP{sceneId}{car_index};
%             pedPredictionForEachCar_HBase = predictedPedTraj_HBase{sceneId}{car_index};
            pedPredictionForEachCar_CV = predictedPedTraj_CV{sceneId}{car_index};
            
            N_ped_pred = size(pedPredictionForEachCar_MHP,1); %if there are no predictions for a pedestrian, that entry is empty
            
            for pedTrackId = 1:N_ped_pred
                % if there was a crossing event or reached Crosswalk event
                % in that pedestrian trajectory, compare the prediction performance
                if ~isempty(crossStartTimeSteps{sceneId}{pedTrackId}) || ~isempty(crossedCWTimeSteps{sceneId}{pedTrackId})
                
                        % ground truth
                        GTPedData = formattedTracksData{sceneId}{pedTrackId};
                        N_GTTimeSteps = size(GTPedData.frame, 1);
                        % prediction
                        pedPredictions_MHP = pedPredictionForEachCar_MHP{pedTrackId}; % the first data is a place holder, 'inf' value
%                         pedPredictions_HBase = pedPredictionForEachCar_HBase{pedTrackId};
                        pedPredictions_CV = pedPredictionForEachCar_CV{pedTrackId};

                        % if the pedestrian was ab active pedestrian
                        if ~isempty(pedPredictions_MHP) && (pedPredictions_MHP.timeStep(end)~=inf)
                            pedPredictionsData_MHP = pedPredictions_MHP.data(2:end); %the first entry in pedPredictions.data is a dummy entry
                            pedKFPredictionsData_MHP = pedPredictions_MHP.kfData(2:end);

%                             pedPredictionsData_HBase = pedPredictions_HBase.data(2:end); %the first entry in pedPredictions.data is a dummy entry
%                             pedKFPredictionsData_HBase = pedPredictions_HBase.kfData(2:end);

                            pedPredictionsData_CV = pedPredictions_CV.data(2:end); %the first entry in pedPredictions.data is a dummy entry
                            pedKFPredictionsData_CV = pedPredictions_CV.kfData(2:end);

                            pedPredictionsTime = pedPredictions_MHP.timeStep(2:end);
                            N_timeSteps = size(pedPredictionsData_MHP, 1);

                            for predictionTimeStep = 1:N_timeSteps 
                                %check if its a interesting timesetp
                                N = size(crossStartTimeSteps{sceneId}{pedTrackId},1);
                                eventFlag = 0;
                                for ii=1:N
                                    if predictionTimeStep < crossStartTimeSteps{sceneId}{pedTrackId}(ii)-event_TS
                                        eventFlag = 1;
                                    end
                                end
                                N = size(crossedCWTimeSteps{sceneId}{pedTrackId},1);
                                for ii=1:N
                                    if predictionTimeStep < crossedCWTimeSteps{sceneId}{pedTrackId}(ii)-event_TS
                                        eventFlag = 1;
                                    end
                                end

                                if eventFlag
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

                                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                        % Base Hybrid predictions
%                                         N_futures_HBase = size(pedPredictionsData_HBase{predictionTimeStep},1);  
%                                        [~, mostProbablePredictionId_HBase] = max(pedPredictionsData_HBase{predictionTimeStep}(:,2));  %probability of the tracks is in the second column

%                                         for predId = 1:N_futures_HBase
%                                             temp = pedPredictionsData_HBase{predictionTimeStep}(predId, :);
% 
%                                             if mod([length(temp)/2 - 1],1) == 0
%                                                 temp = reshape(temp(3:end), [2, length(temp)/2 - 1])';
%                                                 tempIsMHPUsed = true;
%                                             else
%                                                 temp = reshape(temp(2:end), [2, floor(length(temp)/2) - 1])';
%                                                 tempIsMHPUsed = false;
%                                             end
%                                             mostProbablePredictedTrajectory_HBase(predId,:,:) = temp([end-Params.predHorizon+1:end], :);
%                                         end


                                        % constant velocity predictions
                                        mostProbablePredictedTrajectory_CV = pedPredictionsData_CV{predictionTimeStep};
                                        mostProbablePredictedTrajectory_CV = reshape(mostProbablePredictedTrajectory_CV{1}(3:end), [2, Params.predHorizon])';

                                        % ground truth trajectory
                                        startTimeStep = pedPredictionsTime(predictionTimeStep) + 1;
                                        endTimeStep = min(startTimeStep + Params.predHorizon -1, N_GTTimeSteps);                
                                        GTTrajectory = [GTPedData.xCenter(startTimeStep:endTimeStep), GTPedData.yCenter(startTimeStep:endTimeStep)];

                                        % trajectory error
                                        N_PredTimeSteps = size(GTTrajectory,1);
                                        tempError_MHP = GTTrajectory - reshape(mostProbablePredictedTrajectory_MHP(mostProbablePredictionId_MHP,1:N_PredTimeSteps,:),[N_PredTimeSteps,2]);
                                        predError_MHP = vecnorm(tempError_MHP, 2, 2);
%                                         tempError_HBase = GTTrajectory - reshape(mostProbablePredictedTrajectory_HBase(mostProbablePredictionId_HBase,1:N_PredTimeSteps,:),[N_PredTimeSteps,2]);
%                                         predError_HBase = vecnorm(tempError_HBase, 2, 2);
                                        tempError_CV = GTTrajectory - mostProbablePredictedTrajectory_CV(1:N_PredTimeSteps,:);
                                        predError_CV = vecnorm(tempError_CV, 2, 2);

                                        if (predError_MHP(1) > 0.2 || predError_MHP(end) > 6)
                                                % plot
                                                figure()
                                                cw_x = cw.center_x*orthopxToMeter*scaleFactor;
                                                cw_y = cw.center_y*orthopxToMeter*scaleFactor;
                                                plot(cw_x, cw_y, 'ro','MarkerSize', 8);hold on;

                                                plot(GTTrajectory(:,1), GTTrajectory(:,2), 'g*', 'MarkerSize',8); hold on;
                                                shift = 1;
                                                for predId = 1:N_futures_MHP
                                                    plot(mostProbablePredictedTrajectory_MHP(predId,1:N_PredTimeSteps,1), mostProbablePredictedTrajectory_MHP(predId,1:N_PredTimeSteps,2), 'b*', 'MarkerSize',8); hold on;
                                                    text(mostProbablePredictedTrajectory_MHP(predId,N_PredTimeSteps,1), mostProbablePredictedTrajectory_MHP(predId,N_PredTimeSteps,2)-shift,strcat(num2str(predId)));
                                                end
%                                                 for predId = 1:N_futures_HBase
%                                                     plot(mostProbablePredictedTrajectory_HBase(predId,1:N_PredTimeSteps,1), mostProbablePredictedTrajectory_HBase(predId,1:N_PredTimeSteps,2), 'b*', 'MarkerSize',8); hold on;
%                                                     text(mostProbablePredictedTrajectory_HBase(predId,N_PredTimeSteps,1), mostProbablePredictedTrajectory_HBase(predId,N_PredTimeSteps,2)-shift,strcat(num2str(predId)));
%                                                 end
                %                                 plot(mostProbablePredictedTrajectory_HBase(1:N_PredTimeSteps,1), mostProbablePredictedTrajectory_HBase(1:N_PredTimeSteps,2), 'k*', 'MarkerSize',8); hold on;
                                                plot(mostProbablePredictedTrajectory_CV(1:N_PredTimeSteps,1), mostProbablePredictedTrajectory_CV(1:N_PredTimeSteps,2), 'r*', 'MarkerSize',8); hold on;
                                                x=1;
                                                
                                                sceneId
                                                car_index
                                                pedTrackId
                                                startTimeStep-1
                                                x=1;
                                        end


                %                         deterministic error metrics for the entire prediction horizon
                                        N = N_PredTimeSteps;
                                        ADE_MHP(index, 1:N) = [cumsum(predError_MHP)./cumsum(1:N_PredTimeSteps)'];
                                        FDE_MHP(index, 1:N) = [predError_MHP];
%                                         ADE_HBase(index, 1:N) = [cumsum(predError_HBase)./cumsum(1:N_PredTimeSteps)'];
%                                         FDE_HBase(index, 1:N) = [predError_MHP];
                                        ADE_CV(index, 1:N) = [cumsum(predError_CV)./cumsum(1:N_PredTimeSteps)'];
                                        FDE_CV(index, 1:N) = [ predError_CV];                       

                                        averageDistanceError_MHP{sceneId}{car_index}{pedTrackId}{predictionTimeStep} = cumsum(predError_MHP)./cumsum(1:N_PredTimeSteps,1)';
                                        finalDistanceError_MHP{sceneId}{car_index}{pedTrackId}{predictionTimeStep} = predError_MHP;
%                                         averageDistanceError_HBase{sceneId}{car_index}{pedTrackId}{predictionTimeStep} = cumsum(predError_HBase)./cumsum(1:N_PredTimeSteps,1)';
%                                         finalDistanceError_HBase{sceneId}{car_index}{pedTrackId}{predictionTimeStep} = predError_HBase;
                                        averageDistanceError_CV{sceneId}{car_index}{pedTrackId}{predictionTimeStep} = cumsum(predError_CV)./cumsum(1:N_PredTimeSteps,1)';
                                        finalDistanceError_CV{sceneId}{car_index}{pedTrackId}{predictionTimeStep} = predError_CV;
                                        isMHPUsed{sceneId}{car_index}{pedTrackId}(predictionTimeStep,1) = tempIsMHPUsed;

                %                         %% probabiistic error metrics
                %                         type = 'Not_CV';
                %                         [predictedProbability_MHP{sceneId}{car_index}{pedTrackId}{predictionTimeStep}, likelihood_MHP{sceneId}{car_index}{pedTrackId}{predictionTimeStep}, FSR_ratio_MHP{sceneId}{car_index}{pedTrackId}{predictionTimeStep}] = probMetrics(GTTrajectory, pedPredictionsData_MHP{predictionTimeStep}, pedKFPredictionsData_MHP{predictionTimeStep}, Params, type);
                %                         [predictedProbability_HBase{sceneId}{car_index}{pedTrackId}{predictionTimeStep}, likelihood_HBase{sceneId}{car_index}{pedTrackId}{predictionTimeStep}, FSR_ratio_HBase{sceneId}{car_index}{pedTrackId}{predictionTimeStep}] = probMetrics(GTTrajectory, pedPredictionsData_HBase{predictionTimeStep}, pedKFPredictionsData_HBase{predictionTimeStep}, Params, type);
                %                         type = 'CV';
                %                         [predictedProbability_CV{sceneId}{car_index}{pedTrackId}{predictionTimeStep},  likelihood_CV{sceneId}{car_index}{pedTrackId}{predictionTimeStep}, FSR_ratio_CV{sceneId}{car_index}{pedTrackId}{predictionTimeStep}] = probMetrics(GTTrajectory, pedPredictionsData_CV{predictionTimeStep}, pedKFPredictionsData_CV{predictionTimeStep}, Params, type);
                %                         
                %                         % predicted probability of ground truth
                %                         N = length(predictedProbability_MHP{sceneId}{car_index}{pedTrackId}{predictionTimeStep});
                %                         PP_MHP(index, 1:N) = predictedProbability_MHP{sceneId}{car_index}{pedTrackId}{predictionTimeStep};
                %                         PP_HBase(index, 1:N) = predictedProbability_HBase{sceneId}{car_index}{pedTrackId}{predictionTimeStep};
                %                         PP_CV(index, 1:N) = predictedProbability_CV{sceneId}{car_index}{pedTrackId}{predictionTimeStep};
                %                         % likelihood of ground truth being among the
                %                         % predictions
                %                         LL_MHP(index, 1:N) = [ likelihood_MHP{sceneId}{car_index}{pedTrackId}{predictionTimeStep}];
                %                         LL_HBase(index, 1:N) = [ likelihood_HBase{sceneId}{car_index}{pedTrackId}{predictionTimeStep}];
                %                         LL_CV(index, 1:N) = [ likelihood_CV{sceneId}{car_index}{pedTrackId}{predictionTimeStep}];
                %                         % ratio between non-zero predictions and Forward reachable
                %                         % set 
                %                         FSR_MHP(index, 1:N) = [ FSR_ratio_MHP{sceneId}{car_index}{pedTrackId}{predictionTimeStep}];
                %                         FSR_HBase(index, 1:N) = [ FSR_ratio_HBase{sceneId}{car_index}{pedTrackId}{predictionTimeStep}];
                %                         FSR_CV(index, 1:N) = [ FSR_ratio_CV{sceneId}{car_index}{pedTrackId}{predictionTimeStep}];

                                        % 
                                        index = index+1;

                        
                        
                                end            
                        
                            end
                    
                  
                        end
                end
            end
        end
 
    end
    
end

%%%%%%%%%%%%%%%%%%%%%%%
else
    N = size(ADE_CV,1);
    sum_ADE_CV = zeros(Params.predHorizon,1);  
    for index = 1:N
       ind =  find(ADE_CV(index,:)==0,1,'first');
       if ~isempty(ind)
           sum_ADE_CV(1:index) = sum_ADE_CV(1:index) + ADE_CV(1:index);
       else
           sum_ADE_CV = sum_ADE_CV + ADE_CV;
       end
    end
    
end