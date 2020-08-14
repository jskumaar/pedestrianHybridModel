%% This script performs the prediction of the pedestrian

function predictionData = predictStates(currentPedData, currentPedMetaData, currentTSActiveCarData, ped_track_time_step, pred_horizon, AdjustedSampFreq, cw, annotatedImage_enhanced, reset, Prob_GapAcceptanceModel, Prob_CrossingIntentModel)
    
    % Note: the currentPedData can have multiple futures
    
    % load the exponential distribution of cross times after
    % load('crossDelayExpDist.mat')  
    recordingMetaData = readtable(strcat(num2str(18),'_recordingMeta.csv'));
    orthopxToMeter = recordingMetaData.orthoPxToMeter;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% initialize variables
    if ped_track_time_step~=1
        prob_crossing_intent = currentPedData.prob_crossing_intent(ped_track_time_step - 1);
    else
        % initialize default crossing intent to be zero
        prob_crossing_intent = 0;
    end
    currentPedMetaData.ego_veh_gap_hist = -1;  % dummy value for initialization
    flag.sampleWaitTime = false;
    flag.startCross = false;
    flag.GapStart = false;
    prob_GapAccept = 0;
    del_t = 1 /AdjustedSampFreq;
    
    %% Intialize data from the actual data at current time step
    % 10 - xVelocity, 11- yVelocity, 18 - xCenterPix, 19 - yCenterPix, 21- closestCW, 24 - HybridState, 25 - ProbhybridState
    % 27- calcHeading, 28 - closeCar_ind
    parametersToCopy = [10, 11, 18, 19, 21, 24, 27, 28];   
    if ped_track_time_step > AdjustedSampFreq
        currentPed_TS_actualData = currentPedData(ped_track_time_step-AdjustedSampFreq + 1 : ped_track_time_step, parametersToCopy);
    else
        currentPed_TS_actualData = currentPedData(1:ped_track_time_step, parametersToCopy);
    end
    
    % 1a)Initialize the prediction tracklet! 
    node_no = 1;
    predTracklet.data{1} = currentPed_TS_actualData;  % this is track when the ped does not have intent to cross
    predTracklet.probability(1) = 1; 
    predTracklet.startNode = node_no; 
    predTracklet.isActive = true; 
    predTracklet.eventFlag = false;
    predTracklet.endNode = -1;  % dummy value
    predTracklet.Goal = 'NA';
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % 2)Update the crossing intent of the pedestrian if the pedestrian is approaching a crosswalk and it reaches the
       % rolling window (which is 1s currently)
    if ( mod(ped_track_time_step, AdjustedSampFreq)==0 && strcmp(currentPedData.HybridState(end), 'Approach')  )     
        flag.checkIntent = true;  
        prob_crossing_intent = crossingIntentCheck(currentPedData, ped_track_time_step, AdjustedSampFreq, Prob_CrossingIntentModel);              
    else
        flag.checkIntent = false;
    end
    
    % size of current pedestrian actual data
    N_ts = size(currentPed_TS_actualData, 1);
    disp(N_ts)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Prediction loop starts
    for time_step = 1:pred_horizon       
        pred_time_step = N_ts + time_step;        
        % update the number of tracklets
        N_tracklets = size(predTracklet,1);      
        
        % for each tracklet predict the trajectory at this pred time step
        for tracklet_no = 1:N_tracklets                    
            %update the future only if this is an active tracklet
            if predTracklet.isActive(tracklet_no)
                    % Step 3: Find the ego-car in the current time step and save the car track index
                    currentTSPedEgoData = egoCarFunc(predTracklet.data{tracklet_no}, currentTSActiveCarData, cw, annotatedImage_enhanced);                    
                    % Step 3b: Update the pedestrian states
                    predTracklet.data{tracklet_no}.closeCar_ind(pred_time_step-1) = currentTSPedEgoData.closeCar_ind;                    
                    % angle between pedestrian and the crosswalks
                    pixel_pos = [predTracklet.data{tracklet_no}.xCenterPix(end), predTracklet.data{tracklet_no}.yCenterPix(end)];
                    ped_cw_angle(1) = atan2(double([cw.center_y(1) - pixel_pos(2)]), double([cw.center_x(1) - pixel_pos(1)]))*180/pi;
                    ped_cw_angle(2) = atan2(double([cw.center_y(2) - pixel_pos(2)]), double([cw.center_x(2) - pixel_pos(1)]))*180/pi;  
                    ped_cw_angle(3) = atan2(double([cw.center_y(3) - pixel_pos(2)]), double([cw.center_x(3) - pixel_pos(1)]))*180/pi;  
                    ped_cw_angle(4) = atan2(double([cw.center_y(4) - pixel_pos(2)]), double([cw.center_x(4) - pixel_pos(1)]))*180/pi; 
                                        
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % Step 4: Check for events that happens at the end of this
                    % pred time step
                    % Step 4a: (Gap acceptance) if there is an ego car, find if gap starts and the
                    % corresponding gap features
                    if currentTSPedEgoData.closeCar_ind~=inf 
                        flag.EgoCar = true;
                        ego_activeCar_ind = currentTSPedEgoData.closeCar_ind; %the index within the list of active cars; not the actual car track index
                        currentTSEgoCarData = currentTSActiveCarData(ego_activeCar_ind, :);
                        if pred_time_step > 2
                            % Step 3a: Check if a gap has started and gather the features of the gap acceptance model
                              if size(predTracklet{tracklet_no}.data, 1) >= AdjustedSampFreq
                                    [GapFeatures, currentPedMetaData, flag] = compileGapFeatures(predTracklet{tracklet_no}.data, currentTSEgoCarData, currentTSPedEgoData, currentPedMetaData, delta_T, pred_time_step, flag);
                              end  
                        end %end of if loop for gap checking
                    end   %end of if loop for the presence of an ego-vehicle                  
                    % Predict the gap acceptance probability
                    if flag.GapStart
                        F_cumWait = GapFeatures.F_cumWait;
                        F_pedDistToCW = GapFeatures.F_pedDistToCW;
                        F_pedDistToCurb = GapFeatures.F_pedDistToCurb;
                        F_pedDistToVeh = GapFeatures.F_pedDistToVeh;
                        F_pedSpeed = GapFeatures.F_pedSpeed;
                        F_vehVel = GapFeatures.F_vehVel;
                        % Note: use table format, it avoids any confusion in the order of the
                        % features
                        SVMFeatures = table(F_cumWait, F_pedDistToCW, F_pedDistToCurb, ...
                                       F_pedDistToVeh, F_pedSpeed, F_vehVel);
                        prob_GapAccept = predict(Prob_GapAcceptanceModel, SVMFeatures);
                    end                   
                    % Sample wait time if a gap has non-zero probability
                    if ( prob_GapAccept && ~flag.sampleWaitTime )
                       % ideally a wait time has to be sampled and checked with the
                       % wait start of this pedestrian. For now, add a fixed time step
                       GapCheckTime = pred_time_step;
                       %time_start_cross = int32(exprnd(crossDelayExpDist.mu)/dt);
                       time_start_cross = AdjustedSampFreq;                 % 1 second delay
                       flag.sampleWaitTime = true;   
                    else
                       flag.sampleWaitTime = false;  
                    end
                    % Check if sampled wait time is reached
                    if flag.sampleWaitTime
                        if pred_time_step == GapCheckTime + time_start_cross
                            flag.startCross = true;
                        else
                            flag.startCross = flase;
                        end
                    end                                      
                    % Step 4b: (Walkaway) Check if the pedestrian has
                    % reached walkaway state after crossing
                    if ( strcmp(predTracklet.data{tracklet_no}.HybridState(end),'Walkaway') ...
                       && strcmp(predTracklet.data{tracklet_no}.HybridState(end),'Cross') )              
                        flag.finishedCrossing = true;
                    else
                        flag.finishedCrossing = false;
                    end                                        
                    % Step 4c: (Reach Crosswalk) Check if the pedestrian
                    % has reached the center of the crosswalk
                    if ( strcmp(predTracklet.data{tracklet_no}.HybridState(end),'Approach') && abs(currentTSPedEgoData.long_disp_ped_cw_pixels) < 0.2 )                       
                       flag.reachCrosswalk = true; 
                    else
                       flag.reachCrosswalk = false;                         
                    end
                    
                    % Step 5: Has any event occured?
                    if (flag.startCross || flag.finishedCrossing || flag.reachCrosswalk || pred_time_step==N_ts + pred_horizon)
                        
                        %update event flag
                        predTracklet.eventFlag(tracklet_no) = true;                    
                        %note the end node for the current tracklet
                        node_no = node_no + 1;
                        predTracklet.endNode(tracklet_no) = node_no;
                        predTracklet.isActive(tracklet_no) = false;                                      
                        %create a new tracklet based on the event
                        % 1) if a gap was accepted and pedestrian starting
                        % to cross
                        if flag.startCross
                           % create a tracklet for starting to cross
                           predTracklet = newTracklet(predTracklet, node_no, prob_GapAccept);
                           predTracklet.data{end}.HybridState{2} = 'Cross'; %HybridState is a cell variable
                           % if non-zero gap rejection probability
                           if (prob_GapAccept~=1)
                                % create a tracklet for starting to wait
                                predTracklet = newTracklet(predTracklet, node_no, 1-prob_GapAccept);
                                predTracklet.data{end}.HybridState{2} = 'Wait'; %HybridState is a cell variable
                           end                     
                        end
                        
                        % 2) if pedestrian finished crossing
                        if flag.finishedCrossing
                           % create a tracklet for turning left
                           predTracklet = newTracklet(predTracklet, node_no, 0.5);
                           predTracklet.data{end}.HybridState{2} = 'Approach'; %HybridState is a cell variable
                           predTracklet.Goal(end) = 'Approach'; %This tracklet approaches the next crosswalk
                                
                           % create a tracklet for turning right 
                           predTracklet = newTracklet(predTracklet, node_no, 0.5);
                           predTracklet.data{end}.HybridState{2} = 'Walkaway'; %HybridState is a cell variable
                           predTracklet.Goal(end) = 'Walkaway'; %This tracklet walks away
                        end
                        
                        % 3) if pedestrian reached the crosswalk
                        if flag.reachCrosswalk
                           % create a tracklet for having a crossing intent
                           if prob_crossing_intent
                                predTracklet = newTracklet(predTracklet, node_no, prob_crossing_intent);
                                predTracklet.data{end}.HybridState{2} = 'Wait';
                           end
                           
                           % create a tracklet for not having a crossing
                           % intent
                           if prob_crossing_intent~=1
                                predTracklet = newTracklet(predTracklet, node_no, 1-prob_crossing_intent);
                                predTracklet.data{end}.HybridState{2} = 'Approach'; %HybridState is a cell variable
                           end                           
                        end
                    end     % end of event check
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    
                    % update discrete state if there is no new event
                    if ~predTracklet.eventFlag(tracklet_no)                     
                        % check guard conditions for discrete state
                        tempData = hybridState(predTracklet.data{tracklet_no}(pred_time_step-1,:),cw, flag, orthopxToMeter, annotatedImage_enhanced);
                        predTracklet.data{tracklet_no}.HybridState{pred_time_step} = tempData.HybridState;           
                   end
                                       
                    % update the continuous state of pedestrian
                    tempData = updatePedContStates(predTracklet.data{tracklet_no}, ped_cw_angle, reset, predTracklet.Goal(tracklet_no), del_t);
                    predTracklet.data{tracklet_no} = tempData;
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    
                    % predict the state of all the active cars
                    currentTSActiveCarData = updateCarState(currentTSActiveCarData, del_t, reset, cw);
                       
                   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                   
            end % end of current prediction tracklet

        end % end of loop for all prediction tracklets       

    end  % end of loop for prediction horizon

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% compile predictions
    % compile the start and end nodes for each prediction tracklet
    startNodes = [];
    endNodes = [];
    for tracklet_id = 1:size(predTracklet,1)
        startNodes  = [startNodes; predTracklet{tracklet_id}.startNode];
        endNodes    = [endNodes; predTracklet{tracklet_id}.endNode];
    end
    % initialize the first prediction future
    if tracklet_id==1
        predictionData{1}(1,1) = pedTracklet{1}.endNode;
        predictionData{1} = [predictionData{1}, pedTracklet.data{1}.xCenterPix', pedTracklet.data{1}.yCenterPix'];
    end
    % compile all the different prediction futures
    N_new = 1;
    predcopying = true;
    while(predCopying)
          for ii=1:size(predictionData,2)
                nextTracklet = find(startNodes==predictionData{ii}(1,1));
                if (~empty(nextTracklet))
                    temp_predcopying(ii) = 1;
                    for jj = length(nextTracklet):-1:1 %the first tracklet gets added to the first prediction
                       tracklet_id = nextTracklet(jj);
                       if jj~=1
                            predictionData{ii+N_new} = predictionData{ii};
                            predictionData{ii+N_new} = [predictionData{ii+N_new}, pedTracklet.data{tracklet_id}.xCenterPix', pedTracklet.data{tracklet_id}.yCenterPix'];
                            predictionData{ii+N_new}(1,1) = pedTracklet.data{tracklet_id}.endNode;
                            N_new = N_new + 1;
                       else
                            predictionData{ii} = [predictionData{ii}, pedTracklet.data{tracklet_id}.xCenterPix', pedTracklet.data{tracklet_id}.yCenterPix'];
                            predictionData{ii}(1,1) = pedTracklet.data{tracklet_id}.endNode;
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