%% This script calculates the performance of the prediction algorithms

% Deterministic Metrics
% 1) Average displacement error
% 2) Final displacement error
% 3) Gap accpetance classification error
% 4) Crossing intent classification error

% Probabilistic Metrics
% 1) Predicted probability

% parameters
N_scenes = 1;

ADE_HPed = [];
FDE_HPed = [];
ADE_HBase = [];
FDE_HBase = [];
ADE_CV = [];
FDE_CV = [];
PP_HPed = [];
PP_HBase = [];
PP_CV = [];



% load predictions
% load('trajectory_oneScenePartial_HPed.mat');
% predictedPedTraj_HPed = predictedPedTraj;
% predictedPedTraj_HBase = predictedPedTraj;


for sceneId = 1:N_scenes
    
    N_car = size(predictedPedTraj_HPed{sceneId},1);
    for car_index = 1:N_car
        % if there is a car
        if ~isempty(predictedPedTraj_HPed{sceneId}{car_index})
            
            pedPredictionForEachCar_HPed = predictedPedTraj_HPed{sceneId}{car_index};
            pedPredictionForEachCar_HBase = predictedPedTraj_HBase{sceneId}{car_index};
            pedPredictionForEachCar_CV = predictedPedTraj_CV{sceneId}{car_index};
            
            N_ped_pred = size(pedPredictionForEachCar_HPed,1); %if there are no predictions for a pedestrian, that entry is empty
            
            for pedTrackId = 1:N_ped_pred
                % ground truth
                GTPedData = formattedTracksData{sceneId}{pedTrackId};
                N_GTTimeSteps = size(GTPedData, 1);
                % prediction
                pedPredictions_HPed = pedPredictionForEachCar_HPed{pedTrackId}; % the first data is a place holder, 'inf' value
                pedPredictions_HBase = pedPredictionForEachCar_HBase{pedTrackId};
                pedPredictions_CV = pedPredictionForEachCar_CV{pedTrackId};
                
                % if the pedestrian was ab active pedestrian
                if ~isempty(pedPredictions_HPed)
                    pedPredictionsData_HPed = pedPredictions_HPed.data(2:end); %the first entry in pedPredictions.data is a dummy entry
                    pedKFPredictionsData_HPed = pedPredictions_HPed.kfData(2:end);
                    
                    pedPredictionsData_HBase = pedPredictions_HBase.data(2:end); %the first entry in pedPredictions.data is a dummy entry
                    pedKFPredictionsData_HBase = pedPredictions_HBase.kfData(2:end);
                    
                    pedPredictionsData_CV = pedPredictions_CV.data(2:end); %the first entry in pedPredictions.data is a dummy entry
                    pedKFPredictionsData_CV = pedPredictions_CV.kfData(2:end);
                    
                    pedPredictionsTime = pedPredictions_HPed.timeStep(2:end);
                    N_timeSteps = size(pedPredictionsData_HPed, 1);

                    for predictionTimeStep = 1:N_timeSteps 
                        % get the most probable trajectory
                        N_futures = size(pedPredictionsData_HPed{N_timeSteps},1);       
                        [~, mostProbablePredictionId] = max(pedPredictionsData_HPed{predictionTimeStep}(:,2));  %probability of the tracks is in the second column
                        mostProbablePredictedTrajectory_HPed = pedPredictionsData_HPed{predictionTimeStep}(mostProbablePredictionId, :);
                        
                        if mod([length(mostProbablePredictedTrajectory_HPed)/2 - 1],1) == 0
                            mostProbablePredictedTrajectory_HPed = reshape(mostProbablePredictedTrajectory_HPed(3:end), [2, length(mostProbablePredictedTrajectory_HPed)/2 - 1])';
                            tempIsHPedUsed = true;
                        else
                            mostProbablePredictedTrajectory_HPed = reshape(mostProbablePredictedTrajectory_HPed(2:end), [2, floor(length(mostProbablePredictedTrajectory_HPed)/2) - 1])';
                            tempIsHPedUsed = false;
                        end
                        
                        mostProbablePredictedTrajectory_HPed = mostProbablePredictedTrajectory_HPed([end-Params.predHorizon+1:end], :);
                        
                        mostProbablePredictedTrajectory_HBase = pedPredictionsData_HBase{predictionTimeStep};
                        if mod([length(mostProbablePredictedTrajectory_HBase)/2 - 1],1) == 0
                            mostProbablePredictedTrajectory_HBase = reshape(mostProbablePredictedTrajectory_HBase(3:end), [2, length(mostProbablePredictedTrajectory_HBase)/2 - 1])';
                        else
                            mostProbablePredictedTrajectory_HBase = reshape(mostProbablePredictedTrajectory_HBase(2:end), [2, floor(length(mostProbablePredictedTrajectory_HBase)/2) - 1])';
                            
                        end
                        
                        mostProbablePredictedTrajectory_HBase = mostProbablePredictedTrajectory_HBase([end-Params.predHorizon+1:end], :);
                        
                        mostProbablePredictedTrajectory_CV = pedPredictionsData_CV{predictionTimeStep};
                        mostProbablePredictedTrajectory_CV = reshape(mostProbablePredictedTrajectory_CV{1}(2:end), [2, Params.predHorizon])';

                        % ground truth trajectory
                        startTimeStep = pedPredictionsTime(predictionTimeStep) + 1;
                        endTimeStep = min(startTimeStep + Params.predHorizon -1, N_GTTimeSteps);                
                        GTTrajectory = [GTPedData.xCenter(startTimeStep:endTimeStep), GTPedData.yCenter(startTimeStep:endTimeStep)];

                        % trajectory error
                        N_PredTimeSteps = size(GTTrajectory,1);
                        tempError_HPed = GTTrajectory - mostProbablePredictedTrajectory_HPed(1:N_PredTimeSteps,:);
                        predError_HPed = vecnorm(tempError_HPed, 2, 2);
                        tempError_HBase = GTTrajectory - mostProbablePredictedTrajectory_HBase(1:N_PredTimeSteps,:);;
                        predError_HBase = vecnorm(tempError_HBase, 2, 2);
                        tempError_CV = GTTrajectory - mostProbablePredictedTrajectory_CV(1:N_PredTimeSteps,:);;
                        predError_CV = vecnorm(tempError_CV, 2, 2);

                        % deterministic error metrics for the entire prediction horizon
                        ADE_HPed = [ADE_HPed, cumsum(predError_HPed)./cumsum(1:N_PredTimeSteps)'];
                        FDE_HPed = [FDE_HPed, predError_HPed];
                        ADE_HBase = [ADE_HBase, cumsum(predError_HBase)./cumsum(1:N_PredTimeSteps)'];
                        FDE_HBase = [FDE_HBase, predError_HPed];
                        ADE_CV = [ADE_CV, cumsum(predError_CV)./cumsum(1:N_PredTimeSteps)'];
                        FDE_CV = [FDE_CV, predError_CV];                       
                        
                        averageDistanceError_HPed{sceneId}{car_index}{pedTrackId}{predictionTimeStep} = cumsum(predError_HPed)./cumsum(1:N_PredTimeSteps,1);
                        finalDistanceError_HPed{sceneId}{car_index}{pedTrackId}{predictionTimeStep} = predError_HPed;
                        averageDistanceError_HBase{sceneId}{car_index}{pedTrackId}{predictionTimeStep} = cumsum(predError_HBase)./cumsum(1:N_PredTimeSteps,1);
                        finalDistanceError_HBase{sceneId}{car_index}{pedTrackId}{predictionTimeStep} = predError_HBase;
                        averageDistanceError_CV{sceneId}{car_index}{pedTrackId}{predictionTimeStep} = cumsum(predError_CV)./cumsum(1:N_PredTimeSteps,1);
                        finalDistanceError_CV{sceneId}{car_index}{pedTrackId}{predictionTimeStep} = predError_CV;
                        isHPedUsed{sceneId}{car_index}{pedTrackId}(predictionTimeStep,1) = tempIsHPedUsed;
                        
                        % probabiistic error metrics
                        % a) predicted probability
                        predictedProbability_HPed{sceneId}{car_index}{pedTrackId}{predictionTimeStep} = calculatePredictedProbability(GTTrajectory, pedPredictionsData_HPed{predictionTimeStep}, pedKFPredictionsData_HPed{predictionTimeStep}, Params);
                        predictedProbability_HBase{sceneId}{car_index}{pedTrackId}{predictionTimeStep} = calculatePredictedProbability(GTTrajectory, pedPredictionsData_HBase{predictionTimeStep}, pedKFPredictionsData_HBase{predictionTimeStep}, Params);
                        predictedProbability_CV{sceneId}{car_index}{pedTrackId}{predictionTimeStep} = calculatePredictedProbability(GTTrajectory, pedPredictionsData_CV{predictionTimeStep}, pedKFPredictionsData_CV{predictionTimeStep}, Params);

                        PP_HPed = [PP_HPed; predictedProbability_HPed{sceneId}{car_index}{pedTrackId}{predictionTimeStep}];
                        PP_HBase = [PP_HBase; predictedProbability_HBase{sceneId}{car_index}{pedTrackId}{predictionTimeStep}];
                        PP_CV = [PP_CV; predictedProbability_CV{sceneId}{car_index}{pedTrackId}{predictionTimeStep}];
                        
                                              
                        
                    end
                    
                  
                end
            end
        end
        

        
    end
    

end

