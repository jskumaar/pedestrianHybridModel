%% This script calculates the performance of the prediction algorithms

% Deterministic Metrics
% 1) Average displacement error
% 2) Final displacement error
% 3) Gap accpetance classification error
% 4) Crossing intent classification error

% Probabilistic Metrics
% 1) Predicted probability

% parameters
N_scenes = 2;

for sceneId = 1:N_scenes
    
    N_car = size(predictedPedTraj{sceneId},1);
    for car_index = 1:N_car
        % if there is a car
        if ~isempty(predictedPedTraj{sceneId}{car_index})
            
            pedPredictionForEachCar = predictedPedTraj{sceneId}{car_index};
            N_ped_pred = size(pedPredictionForEachCar,1); %if there are no predictions for a pedestrian, that entry is empty
            
            for pedTrackId = 1:N_ped_pred
                % ground truth
                GTPedData = formattedTracksData{sceneId}{pedTrackId};
                N_GTTimeSteps = size(GTPedData, 1);
                % prediction
                pedPredictions = pedPredictionForEachCar{pedTrackId}; % the first data is a place holder, 'inf' value
                
                % if the pedestrian was ab active pedestrian
                if ~isempty(pedPredictions)
                    pedPredictionsData = pedPredictions.data(2:end); %the first entry in pedPredictions.data is a dummy entry
                    pedPredictionsTime = pedPredictions.timeStep(2:end);
                    N_timeSteps = size(pedPredictionsData, 1);

                    for predictionTimeStep = 1:N_timeSteps 
                        % get the most probable trajectory
                        N_futures = size(pedPredictionsData{N_timeSteps},1);       
                        [~, mostProbablePredictionId] = max(pedPredictionsData{predictionTimeStep}(:,2));  %probability of the tracks is in the second column
                        mostProbablePredictedTrajectory = pedPredictionsData{predictionTimeStep}(mostProbablePredictionId, :);
                        mostProbablePredictedTrajectory = reshape(mostProbablePredictedTrajectory(3:end), [length(mostProbablePredictedTrajectory)/2 - 1], 2);
                        mostProbablePredictedTrajectory = mostProbablePredictedTrajectory([end-Params.predHorizon+1:end], :);
                        % ground truth trajectory
                        startTimeStep = pedPredictionsTime(predictionTimeStep) + 1;
                        endTimeStep = min(startTimeStep + Params.predHorizon -1, N_GTTimeSteps);                
                        GTTrajectory = [GTPedData.xCenter(startTimeStep:endTimeStep), GTPedData.yCenter(startTimeStep:endTimeStep)];

                        % trajectory error
                        N_PredTimeSteps = size(GTTrajectory,1);
                        tempError = GTTrajectory - mostProbablePredictedTrajectory;
                        predError = vecnorm(tempError, 2, 2);

                        % error metrics for the entire prediction horizon
                        averageDistanceError{sceneId}{car_index}{pedTrackId}{predictionTimeStep} = cumsum(predError)./cumsum(1:N_PredTimeSteps);
                        finalDistanceError{sceneId}{car_index}{pedTrackId}{predictionTimeStep} = predError;
                    end
                end
            end
        end
        

        
    end
    

end

