%% This script performs the prediction of the pedestrian

function [predictionTrajectory, predictionKFtrajectory, predGapFeatures] = predictStates(kf, currentPedData, currentPedMetaData, currentTSActiveCarData, crossCarData, AVStates, pedTrackTimeStep, cw, annotatedImageEnhanced, reset, Prob_GapAcceptanceModel, Prob_CrossIntentModelCar, Prob_CrossIntentModelNoCar, Params, flag)
    
    % Note: the currentPedData can have multiple futures
    
    % load the exponential distribution of cross times after
    % load('crossDelayExpDist.mat') 
    
    % parameters
    predHorizon = Params.predHorizon;
    AdjustedSampFreq = Params.AdjustedSampFreq;

    

    % initialize variables
    predGapFeatures = {};
    probCrossingIntent = 0;     % initialize default crossing intent to be zero
    currentPedMetaData.egoVehGapHist = -1;  % dummy value for initialization
    flag.sampleWaitTime = false;
    flag.startCross = false;
    flag.GapStart = false;
    flag.finishedCrossing = false;
    flag.pred = true;
    probGapAccept = 0;
    
    %% Intialize data from the actual data at current time step
    % 1 - recordingID, 2 - trackID, 4-  trackLifetime, 5, xCenter, 6 -yCenter, 10 - xVelocity, 11- yVelocity, 18 - xCenterPix, 19 - yCenterPix, 21- closestCW, 
    % 23 - HybridState,  24 - ProbhybridState, 26- calcHeading, 27 - closeCar_ind, 30 - isLooking, 31 - isPedSameDirection, 34 - wait_time_steps, 36 - longDispPedCw, 
    
    % if size(currentPedData,2) > 30
        parametersToCopy = [1, 2, 4, 5, 6, 10, 11, 18, 19, 21, 23, 26, 27, 30, 31, 34, 36];
        % copy actual pedestrian data
        if pedTrackTimeStep > AdjustedSampFreq
            startInd = pedTrackTimeStep - AdjustedSampFreq + 1;
        else
            startInd = 1;
        end
        endInd = pedTrackTimeStep;   
        
        % copy variables 
        currentPed_TS_ActualData = table();
        currentPed_TS_ActualData.recordingId = currentPedData.recordingId(startInd:endInd, :);
        currentPed_TS_ActualData.trackId = currentPedData.trackId(startInd:endInd, :);
        currentPed_TS_ActualData.trackLifetime = currentPedData.trackLifetime(startInd:endInd, :);
        currentPed_TS_ActualData.xCenter = currentPedData.xCenter(startInd:endInd, :);
        currentPed_TS_ActualData.yCenter = currentPedData.yCenter(startInd:endInd, :);
        currentPed_TS_ActualData.xVelocity = currentPedData.xVelocity(startInd:endInd, :);
        currentPed_TS_ActualData.yVelocity = currentPedData.yVelocity(startInd:endInd, :);
        currentPed_TS_ActualData.xCenterPix = currentPedData.xCenterPix(startInd:endInd, :);
        currentPed_TS_ActualData.yCenterPix = currentPedData.yCenterPix(startInd:endInd, :);
        currentPed_TS_ActualData.closestCW = currentPedData.closestCW(startInd:endInd, :);
        currentPed_TS_ActualData.HybridState = currentPedData.HybridState(startInd:endInd, :);
        currentPed_TS_ActualData.ProbHybridState = currentPedData.ProbHybridState(startInd:endInd, :);
        currentPed_TS_ActualData.calcHeading = currentPedData.calcHeading(startInd:endInd, :);
        currentPed_TS_ActualData.closeCar_ind = currentPedData.closeCar_ind(startInd:endInd, :);
        currentPed_TS_ActualData.isLooking = currentPedData.isLooking(startInd:endInd, :);
        currentPed_TS_ActualData.isPedSameDirection = currentPedData.isPedSameDirection(startInd:endInd, :);
        if sum(strcmp('waitTimeSteps',currentPedData.Properties.VariableNames))~=0
            currentPed_TS_ActualData.waitTimeSteps = currentPedData.waitTimeSteps(startInd:endInd, :);
        else
            currentPed_TS_ActualData.waitTimeSteps = currentPedData.wait_time_steps(startInd:endInd, :);
        end
        currentPed_TS_ActualData.longDispPedCw = currentPedData.longDispPedCw(startInd:endInd, :);

       
    % end
    
    % parametersToCopy = [1, 2, 4, 5, 6, 10, 11, 18, 19, 21, 24, 27, 28];
    
    % run prediction if there is an ego-vehicle approaching the pedestrian
    % in the current time step.
    
    % 1a)Initialize the prediction tracklet! 
    node_no = 1;
    predictionTracklet.data{1} = currentPed_TS_ActualData;  % this is track when the ped does not have intent to cross
    predictionTracklet.kfData{1} = [kf.x', diag(kf.P)'];     % Kalman filter states and variances
    predictionTracklet.probability(1) = 1; 
    predictionTracklet.startNode(1) = node_no; 
    predictionTracklet.isActive(1) = true; 
    predictionTracklet.eventFlag(1) = false;
    predictionTracklet.endNode(1) = -1;  % dummy value
    predictionTracklet.Goal(1, :) = 'NA';
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
%     % 2)Update the crossing intent of the pedestrian if the pedestrian is approaching a crosswalk and it reaches the
%        % rolling window (which is 1s currently)
%     if ( mod(ped_track_time_step, AdjustedSampFreq)==0 && strcmp(currentPedData.HybridState(end), 'Approach')  )     
%         flag.checkIntent = true;  
%         prob_crossing_intent = crossingIntentCheck(currentPedData, ped_track_time_step, Params, Prob_CrossIntentModel);              
%     else
%         flag.checkIntent = false;
%     end
    
    % size of current pedestrian actual data
    N_ts = size(currentPed_TS_ActualData, 1);
    %disp(N_ts)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Prediction loop starts
    annotatedImageEnhanced_w_tracks = annotatedImageEnhanced;
    % initialize KF tracklet
    predictionKFtracklet{1} = kf;
    
    for timeStep = 1:predHorizon       
        predTimeStep = N_ts + timeStep;        
        % update the number of tracklets
        N_tracklets = size(predictionTracklet.data,1);      
        
        % for each tracklet predict the trajectory at this pred time step
        for trackletNo = 1:N_tracklets                    
            %update the future only if this is an active tracklet
            if predictionTracklet.isActive(trackletNo)
                    % Step 3: Find the ego-car in the current time step and save the car track index
                    currentTSPedEgoData = egoCarFunc(predictionTracklet.data{trackletNo}, currentTSActiveCarData, cw, annotatedImageEnhanced,  Params);                                       
                    predictionTracklet.data{trackletNo}.isLooking(end) = currentTSPedEgoData.isLooking;                   
                    predictionTracklet.data{trackletNo}.isPedSameDirection(end) = currentTSPedEgoData.isPedSameDirection;
                    predictionTracklet.data{trackletNo}.longDispPedCw(end) = currentTSPedEgoData.longDispPedCw;
                    predictionTracklet.data{trackletNo}.closeCar_ind(end) = currentTSPedEgoData.closeCar_ind;                    

                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % Step 4: Check for events that happens at the end of this
                    % pred time step
                    % Step 4a: (Gap acceptance) if there is an ego car, find if gap starts and the
                    % corresponding gap features. A gap can start when the
                    % pedestrian has reached the decision zone
                    if currentTSPedEgoData.closeCar_ind~=inf 
                        flag.EgoCar = true;
                        egoActiveCarInd = currentTSPedEgoData.closeCar_ind; %the index within the list of active cars; not the actual car track index
                        currentTSEgoCarData = currentTSActiveCarData(egoActiveCarInd, :);
                        %if pred_time_step > 2
                            % Step 3a: Check if a gap has started and gather the features of the gap acceptance model
                              if size(predictionTracklet.data{trackletNo}, 1) >= AdjustedSampFreq
                                    [GapFeatures, currentPedMetaData, flag] = compileGapFeatures(predictionTracklet.data{trackletNo}, currentTSPedEgoData, currentTSEgoCarData, currentPedMetaData, pedTrackTimeStep, Params, flag);  
                                    if ~isempty(GapFeatures)
                                        predGapFeatures{end+1,1} = GapFeatures;
                                    end
                              end  
                        %end %end of if loop for gap checking
                    end   %end of if loop for the presence of an ego-vehicle                  
                    % Predict the gap acceptance probability
                    if flag.GapStart
                        % Note: use table format, it avoids any confusion in the order of the
                        % features
                        F_cumWait = GapFeatures.F_cumWait;
                        F_pedDistToCW = GapFeatures.F_pedDistToCW; 
                        F_pedDistToCurb = GapFeatures.F_pedDistToCurb;
                        F_pedDistToVeh = GapFeatures.F_pedDistToVeh; 
                        F_pedSpeed = GapFeatures.F_pedSpeed; 
                        F_vehVel = GapFeatures.F_vehVel;
                        F_gazeRatio = GapFeatures.F_gazeRatio;                  
                                           
                        GapSVMFeatures = table(F_cumWait, F_pedDistToCW, F_pedDistToCurb, ...
                                               F_pedDistToVeh, F_pedSpeed, F_vehVel);
                        [pred_gap_class, prob_GA_outputs] = predict(Prob_GapAcceptanceModel, GapSVMFeatures);
                        probGapAccept = prob_GA_outputs(2);
                    end                   
                    % Sample wait time if a gap has non-zero probability
                    if ( probGapAccept && ~flag.sampleWaitTime )
                       % ideally a wait time has to be sampled and checked with the
                       % wait start of this pedestrian. For now, add a fixed time step
                       GapCheckTime = predTimeStep;
                       %time_start_cross = int32(exprnd(crossDelayExpDist.mu)/dt);
                       timeStartCross = AdjustedSampFreq;                 % 1 second delay
                       flag.sampleWaitTime = true;   
                    else
                       flag.sampleWaitTime = false;  
                    end
                    % Check if sampled wait time is reached
                    if flag.sampleWaitTime
                        if predTimeStep == GapCheckTime + timeStartCross
                            flag.startCross = true;
                        else
                            flag.startCross = false;
                        end
                    end                                      
                    % Step 4b: (Walkaway) Check if the pedestrian has
                    % reached walkaway state after crossing
                    if predTimeStep > 2
                        if ( strcmp(predictionTracklet.data{trackletNo}.HybridState(end),'Walkaway') ...
                           && strcmp(predictionTracklet.data{trackletNo}.HybridState(end-1),'Cross') )              
                            flag.finishedCrossing = true;
                        else
                            flag.finishedCrossing = false;
                        end         
                    end
                    % Step 4c: (Reach Crosswalk) Check if the pedestrian
                    % has reached the crosswalk
                    if ( strcmp(predictionTracklet.data{trackletNo}.HybridState(end),'Approach') && abs(currentTSPedEgoData.longDispPedCw) < 0.3)                       
                       flag.reachCrosswalk = true; 
                       flag.checkIntent = true;
                       
                       % compile cross intent check features and check
                       % probability based on whether there is an ego-car
                       % or not
                       car_id = currentPedData.closeCar_ind(pedTrackTimeStep);
                       
                       if (car_id ~=0 && car_id ~=inf)
                           [CrossFeatures] = compileCrossFeatures(currentPedData, pedTrackTimeStep, Params, crossCarData);
                           mean_veh_speed = CrossFeatures.mean_veh_speed;
                           mean_DTCurb = CrossFeatures.mean_DTCurb;
                           mean_veh_ped_dist = CrossFeatures.mean_veh_ped_dist;
                           mean_ped_speed = CrossFeatures.mean_ped_speed;
                           gaze_ratio = CrossFeatures.gaze_ratio;
                           mean_DTCW = CrossFeatures.mean_DTCW;
                           direction = CrossFeatures.direction;
                           
                           
                           CrossSVMFeatures = table(mean_veh_speed, mean_DTCurb , mean_veh_ped_dist,...
                                                    mean_ped_speed, gaze_ratio, mean_DTCW, direction);
                           [pred_calss, prob_CI_outputs] = predict(Prob_CrossIntentModelCar, CrossSVMFeatures);
                            probCrossingIntent = prob_CI_outputs(2);
                       else
%                            [CrossFeatures] = compileCrossFeatures(currentPedData, ped_track_time_step, Params, crossCarData);
%                            CrossSVMFeatures = table(CrossFeatures.mean_DTCurb , CrossFeatures.mean_ped_speed, CrossFeatures.mean_DTCW);
%                            prob_crossing_intent = predict(Prob_CrossIntentModelNoCar, CrossSVMFeatures);
                             probCrossingIntent = 0.9;
                       end         
                    else
                       flag.reachCrosswalk = false;                         
                    end
                    

                    
                    % Step 5: Has any event occured and it is not a new
                    % tracklet? In New tracklets, the same event gets
                    % replicated recursively
                    if  ((flag.startCross || flag.finishedCrossing || flag.reachCrosswalk || predTimeStep==N_ts + predHorizon) ...
                          && height(predictionTracklet.data{trackletNo}) > 1)
                        
                        %update event flag
                        flag.pred = true;  %is hybrid state predicted using SVM models?
                        predictionTracklet.eventFlag(trackletNo) = true;                    
                        %note the end node for the current tracklet
                        node_no = node_no + 1;
                        predictionTracklet.endNode(trackletNo) = node_no;
                        predictionTracklet.isActive(trackletNo) = false;                                      
                        %create a new tracklet based on the event
                        % 1) if a gap was accepted and pedestrian starting
                        % to cross
                        if flag.startCross
                           % create a tracklet for starting to cross
                           predictionTracklet = newTracklet(predictionTracklet, node_no, probGapAccept);
                           predictionTracklet.data{end}.HybridState(end,:) = 'Cross'; %HybridState is a cell variable
                           predictionKFtracklet{end+1} = predictionKFtracklet{end};
                           % if non-zero gap rejection probability
                           if (probGapAccept~=1)
                                % create a tracklet for starting to wait
                                predictionTracklet = newTracklet(predictionTracklet, node_no, 1-probGapAccept);
                                predictionTracklet.data{end}.HybridState(end,:) = 'Wait'; %HybridState is a cell variable
                                predictionKFtracklet{end+1} = predictionKFtracklet{end};
                           end                     
                        end
                        
                        % 2) if pedestrian finished crossing
                        if flag.finishedCrossing
                           % create a tracklet for turning left
                           predictionTracklet = newTracklet(predictionTracklet, node_no, 0.5);
                           predictionTracklet.data{end}.HybridState(end,:) = 'Approach'; %HybridState is a cell variable
                           predictionTracklet.Goal(end) = 'Approach'; %This tracklet approaches the next crosswalk
                           predictionKFtracklet{end+1} = predictionKFtracklet{end};
                           
                           % create a tracklet for turning right 
                           predictionTracklet = newTracklet(predictionTracklet, node_no, 0.5);
                           predictionTracklet.data{end}.HybridState(end,:) = 'Walkaway'; %HybridState is a cell variable
                           predictionTracklet.Goal(end) = 'Walkaway'; %This tracklet walks away
                           predictionKFtracklet{end+1} = predictionKFtracklet{end};
                        end
                        
                        % 3) if pedestrian reached the crosswalk
                        if flag.reachCrosswalk
                           % create a tracklet for having a crossing intent
                           if probCrossingIntent
                                predictionTracklet = newTracklet(predictionTracklet, node_no, probCrossingIntent);
                                predictionTracklet.data{end}.HybridState(end,:) = 'Wait';
                                predictionKFtracklet{end+1} = predictionKFtracklet{end};
                           end
                           
                           % create a tracklet for not having a crossing
                           % intent
                           if probCrossingIntent~=1
                                predictionTracklet = newTracklet(predictionTracklet, node_no, 1-probCrossingIntent);
                                predictionTracklet.data{end}.HybridState(end,:) = 'Approach'; %HybridState is a cell variable
                                predictionKFtracklet{end+1} = predictionKFtracklet{end};
                           end                           
                        end
                        
                        % 4) if end of prediction horizon
                        if predTimeStep==N_ts + predHorizon
                            predictionTracklet.data{trackletNo}.HybridState(end+1,:) = strings;
                            flag.pred = false;
                        end
                    else
                        predictionTracklet.eventFlag(trackletNo) = false;                         
                    end     % end of event check
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    
                    % update discrete state if there is no new event
                    if ~predictionTracklet.eventFlag(trackletNo)                     
%                         % check guard conditions for discrete state
%                         tempData = hybridState(predictionTracklet.data{tracklet_no}(pred_time_step-1,:),cw, flag, annotatedImage_enhanced, Params);
                        predictionTracklet.data{trackletNo}.HybridState(end+1,:) = strings; 
                        flag.pred = false;
                    end                 

                    % update the continuous state of pedestrian
                    kf = predictionKFtracklet{trackletNo};
                    [tempData, kf] = updatePedContStates(kf, predictionTracklet.data{trackletNo}, cw, Params, reset, predictionTracklet.Goal(trackletNo,:));
                    predictionTracklet.data{trackletNo} = tempData; 
                    predictionTracklet.kfData{trackletNo}(end+1, :) = [kf.x', diag(kf.P)']; 
                    predictionKFtracklet{trackletNo} = kf;
                    
                    
                    % update other parameters of pedestrian for the last
                    % time step
                    tempData2 = hybridState(predictionTracklet.data{trackletNo}(end,:), cw, flag, annotatedImageEnhanced, Params);
                    predictionTracklet.data{trackletNo}(end,:) = tempData2;
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    
                    % predict the state of all the active cars
                    if ~isempty(currentTSActiveCarData)
                        currentTSActiveCarData = updateCarState(currentTSActiveCarData, AVStates, Params, reset, cw);
                    end  
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    
                    % plot predicted vehicle and pedestrian states
                    for ii=1:size(currentTSActiveCarData,1)
                        carPos = [currentTSActiveCarData.xCenterPix(ii), currentTSActiveCarData.yCenterPix(ii)];
                        annotatedImageEnhanced_w_tracks(-carPos(2), carPos(1)) = 150;
                    end
                    annotatedImageEnhanced_w_tracks(int32(-predictionTracklet.data{trackletNo}.yCenterPix(end)), int32(predictionTracklet.data{trackletNo}.xCenterPix(end))) = 75;
                    %imshow(annotatedImage_enhanced_w_tracks);
                    
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                  
            end % end of current prediction tracklet

        end % end of loop for all prediction tracklets       

    end  % end of loop for prediction horizon

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% compile predictions
    % compile the start and end nodes for each prediction tracklet
    startNodes = [];
    endNodes = [];
    for trackletId = 1:size(predictionTracklet,1)
        startNodes  = [startNodes; predictionTracklet.startNode(trackletId)];
        endNodes    = [endNodes; predictionTracklet.endNode(trackletId)];
    end
    % initialize the first prediction future
    predictionTrajectory(1,1) = predictionTracklet.endNode(1);
    predictionTrajectory(1,2) = predictionTracklet.probability(1);
    predictionTrajectory = [predictionTrajectory, predictionTracklet.data{1}.xCenter', predictionTracklet.data{1}.yCenter'];
    predictionKFtrajectory = reshape(predictionTracklet.kfData{1}, [size(predictionTracklet.kfData{1},1)*size(predictionTracklet.kfData{1},2)  ,1])';
    
    % compile all the different prediction futures
    N_new = 1;
    predCopying = true;
    while(predCopying)
          for ii=1:size(predictionTrajectory,1)
                nextTracklet = find(startNodes==predictionTrajectory(ii,1));
                if (~isempty(nextTracklet))
                    temp_predcopying(ii) = 1;
                    for jj = length(nextTracklet):-1:1 %the first tracklet gets added to the first prediction
                       trackletId = nextTracklet(jj);
                       if jj~=1
                            predictionTrajectory(ii+N_new, :) = predictionTrajectory(ii, :);
                            predictionTrajectory(ii+N_new, :) = [predictionTrajectory(ii+N_new,:), predictionTracklet.data{trackletId}.xCenterPix', predictionTracklet.data{trackletId}.yCenterPix'];
                            predictionTrajectory(ii+N_new,1) = predictionTracklet.endNode(trackletId);
                            predictionTrajectory(ii+N_new,2) = predictionTracklet.probability(trackletId);                          
                            predictionKFtrajectory(ii+N_new, :) = [predictionKFtrajectory(ii+N_new, :), reshape(predictionTracklet.kfData{1}, [size(predictionTracklet.kfData{trackletId},1)*size(predictionTracklet.kfData{trackletId},2)  ,1])'];
                            N_new = N_new + 1;
                       else
                            predictionTrajectory(ii,:) = [predictionTrajectory(ii,:), predictionTracklet.data{trackletId}.xCenterPix', predictionTracklet.data{trackletId}.yCenterPix'];
                            predictionTrajectory(ii,1) = predictionTracklet.endNode(trackletId);
                            predictionTrajectory(ii,2) = predictionTrajectory(ii,2) * predictionTracklet.probability(trackletId);
                            predictionKFtrajectory(ii, :) = [predictionKFtrajectory(ii, :), predictionTracklet.kfData{trackletId}];
                       end

                    end
                else
                    temp_predcopying(ii) = 0;
                end
                % check if there are no more tracklets for any of the paths
                if sum(temp_predcopying)==0
                    predCopying = false;
                end

          end 
    end
    

end % end of the function