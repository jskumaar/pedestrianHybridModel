%% This script performs the prediction of the pedestrian
% Note: the currentPedData can have multiple futures;
% run prediction if there is an ego-vehicle approaching the pedestrian
% in the current time step.

% function [predictionTrajectoryMatrix, predictionKFtrajectory, predGapFeatures, predCrossFeatures] = predictStates(kf, currentPedData, ~, currentTSActiveCarData, crossCarData, AVStates, pedTrackTimeStep, cw, annotatedImageEnhanced, reset, Prob_GapAcceptanceModel, Prob_CrossIntentModelCar, Prob_CrossIntentModelNoCar, Params, flag)
function [predictionTrajectoryMatrix, predictionKFtrajectory, predGapFeatures, predCrossFeatures] = predictStates(kf, currentPedData, ~, currentTSActiveCarData, carTrackCurrentTimeStep, AVStates, pedTrackTimeStep, cw, annotatedImageEnhanced, reset, Prob_GapAcceptanceModel, Prob_CrossIntentModelCar, Prob_CrossIntentModelNoCar, Params, flag)


%% 1) setup
% load the exponential distribution of cross times after
% load('crossDelayExpDist.mat') 
%%%%%%%%%%%%%%%%%%%    
% parameters
predHorizon = Params.predHorizon;
AdjustedSampFreq = Params.AdjustedSampFreq;
scaleFactor = Params.scaleFactor;
orthopxToMeter = Params.orthopxToMeter;
reSampleRate = Params.reSampleRate;
%%%%%%%%%%%%%%%%%%% 
% initialize variables
% probCrossingIntent = 0.7;     % initialize default crossing intent based on prior from the data
egoVehGapHist = -1;  % dummy value for initialization
flag.sampleWaitTime = false;
flag.startCross = false;
flag.GapStart = false;
flag.finishedCrossing = false;
flag.hybridStatePred = true;
flag.predHorizonEnd = false;
flag.atCrosswalk = false;
flag.outOfPlay = false;
flag.outOfRange = false;
flag.startingFromWait = false;
flag.checkIntentWOEgo = false;
flag.reachGoal = false;
gapId = 1;
crossId = 1;
newTrackletId = 1;
GapCheckTimeInHorizon = -1*ones(10,1);
predGapFeatures = cell(10,1);
predCrossFeatures = cell(10,1);
predictionKFtracklet = cell(10,1);
%%%%%%%%%%%%%%%%%%%  
% Intialize data from the dataset at current time step

% time instances to copy
if pedTrackTimeStep > AdjustedSampFreq
    startInd = pedTrackTimeStep - AdjustedSampFreq + 1;
else
    startInd = 1;
end
endInd = pedTrackTimeStep;
N_obsTimeSteps = endInd-startInd+1;
%%%%%%%%%%
% copy variables to struct format for faster computation
currentPed_TS_ActualData = struct; % creates an empty scalar struct needed for downstream processes
currentPed_TS_ActualData.trackLifetime(:,1) = currentPedData.trackLifetime(startInd:endInd);
currentPed_TS_ActualData.xCenter(:,1) = currentPedData.xCenter(startInd:endInd);
currentPed_TS_ActualData.yCenter(:,1) = currentPedData.yCenter(startInd:endInd);
currentPed_TS_ActualData.xVelocity(:,1) = currentPedData.xVelocity(startInd:endInd);
currentPed_TS_ActualData.yVelocity(:,1) = currentPedData.yVelocity(startInd:endInd);
currentPed_TS_ActualData.lonVelocity(:,1) = currentPedData.lonVelocity(startInd:endInd);
currentPed_TS_ActualData.closestCW(:,1) = currentPedData.closestCW(startInd:endInd);
currentPed_TS_ActualData.HybridState(:,1) = currentPedData.HybridState(startInd:endInd);
currentPed_TS_ActualData.calcHeading(:,1) = currentPedData.calcHeading(startInd:endInd);
currentPed_TS_ActualData.closeCar_ind(:,1) = currentPedData.closeCar_ind(startInd:endInd);
currentPed_TS_ActualData.isLooking(:,1) = currentPedData.isLooking(startInd:endInd);
currentPed_TS_ActualData.isPedSameDirection(:,1) = currentPedData.isPedSameDirection(startInd:endInd);
currentPed_TS_ActualData.long_disp_ped_car(:,1) = currentPedData.long_disp_ped_car(startInd:endInd);
currentPed_TS_ActualData.goalDisp(:,1) = inf*ones(N_obsTimeSteps,1);
currentPed_TS_ActualData.activeCar_ind(:,1) =  inf*ones(N_obsTimeSteps,1);
currentPed_TS_ActualData.Lane(:,1) = currentPedData.Lane(startInd:endInd,:);
currentPed_TS_ActualData.swInd(:,1) = currentPedData.swInd(startInd:endInd);
currentPed_TS_ActualData.goalPositionPixels = inf*ones(N_obsTimeSteps,2);
currentPed_TS_ActualData.waitTimeSteps(:,1) = currentPedData.waitTimeSteps(startInd:endInd);
currentPed_TS_ActualData.longDispPedCw = currentPedData.longDispPedCw(startInd:endInd);
currentPed_TS_ActualData.latDispPedCw = currentPedData.latDispPedCw(startInd:endInd);
currentPed_TS_ActualData.isNearLane(:,1) = currentPedData.isNearLane(startInd:endInd);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 2) Pedestrian Prediction: 
% Initialize the prediction tracklet! 
node_no = 1;
parentTracklet(1) = 1;
trackletData{newTrackletId} = currentPed_TS_ActualData;  % this is track when the ped does not have intent to cross
trackletData{newTrackletId}.probGapAccept = 0;
trackletData{newTrackletId}.probCrossingIntent = 0;
trackletKfData{newTrackletId} = [kf.x', diag(kf.P)'];     % Kalman filter states and variances
trackletProbability(newTrackletId) = 1; 
trackletStartNode(newTrackletId) = node_no; 
trackletEndNode(newTrackletId) = -1;  % dummy value
trackletIsActive(newTrackletId) = true; 
trackletEventFlag(newTrackletId) = false;
trackletGoal(newTrackletId, 1) = "NA";
predictionKFtracklet{newTrackletId} = kf; % different from kfData; this has the entire KF struct
% image
annotatedImageEnhanced_w_tracks = annotatedImageEnhanced;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Starting from Wait?
if strcmp(currentPed_TS_ActualData.HybridState(:,end),'Wait')
    flag.startingFromWait = true;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
% initialize ego car
% Find the ego-car in the current time step and save the car track index
tempData = egoCarFunc(trackletData{1}, currentTSActiveCarData, carTrackCurrentTimeStep, cw, annotatedImageEnhanced,  Params, 1, reset);                                       
trackletData{1} = tempData;




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                       
                    
%% 3) Prediction loop 
for timeStep = 1:predHorizon       
%         predTimeStep = N_ts + timeStep;        
    % update the current number of tracklets (new tracklets can be formed
    % within the inner for loop)
    N_tracklets = size(trackletData,1);  
    
    %% check active vehicles
    % if there is no observed car data aavailable for this prediction time step, remove that car from the list of active cars
    removeId = [];
    if ~isempty(currentTSActiveCarData)
        for carId = 1:size(currentTSActiveCarData,1)
            if timeStep + carTrackCurrentTimeStep(carId) - 1 > size(currentTSActiveCarData{carId}.car_lane,1)
                removeId = [removeId; carId];
                for trackletNo = 1:N_tracklets  
                    if isempty(currentTSActiveCarData)
                        trackletData{trackletNo}.activeCar_ind(end) = inf; % there is no active car anymore. No data available or out of observation
                    elseif trackletData{trackletNo}.activeCar_ind(end) > 1
                        trackletData{trackletNo}.activeCar_ind(end) = trackletData{trackletNo}.activeCar_ind(end)-1;
                        
                    end
                end
            end
        end
    end
    currentTSActiveCarData(removeId) = [];
    carTrackCurrentTimeStep(removeId) = [];
    
    %% debug
    if length(trackletData{1}.activeCar_ind)>1
        if trackletData{1}.activeCar_ind(end)==inf && trackletData{1}.activeCar_ind(end-1)~=inf && trackletData{1}.closeCar_ind(end)~=inf 
            x=1;
        end
    end
    
    if N_tracklets>3
        x=1;
    end
    if N_tracklets>5
        x=1;
    end
    if N_tracklets>7
        x=1;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % for each tracklet predict the trajectory at this pred time step
    for trackletNo = 1:N_tracklets                    
        %update the future only if this is an active tracklet
        if trackletIsActive(trackletNo)
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %% do not consider the pedestrians if they are out of range of the ego-vehicle or out of range of the environment
                pedPosPixels = [trackletData{trackletNo}.xCenter(end),  trackletData{trackletNo}.yCenter(end)]/(orthopxToMeter*scaleFactor);
                pedHeading = trackletData{trackletNo}.calcHeading(end);
                AVHeading = AVStates.carHeading;
                AVPosPixels  = AVStates.carPosPixels;
                distPedEgo = norm(pedPosPixels-double(AVPosPixels));   
                % check if the pedestrian is out of range of the
                % ego-vehicle or out of the observation region of the
                % dataset
                if (( distPedEgo > Params.sensingRange || abs(AVHeading-pedHeading) > 90 ))       
                    flag.outOfRange = true;
                else
                    flag.outOfRange = false;
                end                    
                if ( pedPosPixels(1)<=100 || pedPosPixels(1)>=950 || pedPosPixels(2)<=-560 || pedPosPixels(2)>=-100 )
                    flag.outOfPlay = true;
%                     break
                else
                    flag.outOfPlay = false;
                end
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%             
                %% 4) Check for events that happens at the end of this
                % pred time step
                %%%%%%%%%%%%%%%%%%%%%%%%%             
                % Step 4a: (Gap acceptance) if there is an ego car, find if a gap starts and the
                % corresponding gap features. 
                if ((trackletData{trackletNo}.closeCar_ind(end)~=inf && trackletData{trackletNo}.closeCar_ind(end)~=0) && ~isempty(currentTSActiveCarData))
                    flag.EgoCar(trackletNo) = true;
                    egoActiveCarInd = trackletData{trackletNo}.activeCar_ind(end); %the index within the list of active cars; not the actual car track index
                    currentTSEgoCarData = currentTSActiveCarData{egoActiveCarInd};
                    % Check if a gap has started and gather the features of
                    % the gap acceptance model. A gap can start when the
                    % pedestrian has reached the decision zone or when the
                    % pedestrians' observation (1st time step starts from
                    % 'Wait'.
                      if size(trackletData{trackletNo}.trackLifetime, 1) >= AdjustedSampFreq
                            [GapFeatures, egoVehGapHist, flag] = compileGapFeatures(trackletData{trackletNo}, currentTSEgoCarData, carTrackCurrentTimeStep(egoActiveCarInd), egoVehGapHist, pedTrackTimeStep, Params, flag, trackletNo, timeStep);  
                      end  
                else
                    flag.EgoCar(trackletNo) = false;
                end   %end of if loop for the presence of an ego-vehicle                  
                %%%%%%%%%%%%%%%%%%%%%%%%%
                % Predict the gap acceptance probability when there is
                % a new gap.  A gap can also start when the pedestrian starts from 'Wait' state.
                if ( ( flag.GapStart(trackletNo) && flag.EgoCar(trackletNo) ) )                     
                    % Note: use table format, it avoids any confusion in the order of the
                    % features
                            F_cumWait = GapFeatures.F_cumWait;
                            F_pedDistToCW = GapFeatures.F_pedDistToCW; 
                            F_pedDistToCurb = GapFeatures.F_pedDistToCurb;
                            F_pedDistToVeh = GapFeatures.F_pedDistToVeh; 
                            F_pedSpeed = GapFeatures.F_pedSpeed; 
                            F_vehVel = GapFeatures.F_vehVel;
        %                     F_gazeRatio = GapFeatures.F_gazeRatio; 
                            F_isSameDirection = GapFeatures.F_isSameDirection; 
                            F_isEgoNearLane = GapFeatures.F_isEgoNearLane; 

                            GapSVMFeatures = table(F_cumWait, F_pedDistToCW, F_pedDistToCurb, ...
                                                   F_pedDistToVeh, F_pedSpeed, F_vehVel,...
                                                   F_isSameDirection, F_isEgoNearLane);
                            [~, prob_GA_outputs] = predict(Prob_GapAcceptanceModel, GapSVMFeatures);
                            trackletData{trackletNo}.probGapAccept = prob_GA_outputs(2);
                            % Time for gap check within the prediction horizon
                            % (this is relative time and not the absolute time)
                            GapCheckTimeInHorizon(gapId) = timeStep;
                            %%%%%%%%%%%%%%%%%%%%%%%%%
                            % save gap check variables
                            GapFeatures.predDecision = trackletData{trackletNo}.probGapAccept;
                            GapFeatures.timeStepInHorizon = timeStep;
                            predGapFeatures{gapId} = GapFeatures;           
                            % update gap id
                            gapId = gapId + 1;
                            %% debug
                            if abs(GapFeatures.F_pedDistToCurb) > 6
                                x=1;
                            end
                            %%%%%%%%%%%%%%%%%%%
                    % if gap starts from wait and there is no close
                    % vehicle, accept the gap
                elseif flag.startingFromWait(trackletNo) && ~flag.EgoCar(trackletNo)
                            trackletData{trackletNo}.probGapAccept = 1;
                            GapFeatures.timeStepInHorizon = timeStep;
                            gapId = gapId + 1;
                            
                end
                %%%%%%%%%%%%%%%%%%%%%%%%%
                % Sample wait time if a gap has non-zero probability
                if ( trackletData{trackletNo}.probGapAccept && ~flag.sampleWaitTime(trackletNo) )
                   % ideally a wait time has to be sampled and checked with the
                   % wait start of this pedestrian. For now, add a fixed time step

                   %time_start_cross = int32(exprnd(crossDelayExpDist.mu)/dt);
                   timeStartCross = 0.4*AdjustedSampFreq;                 % 0.4 s second fixed delay; mean value is 0.4741 s
                   flag.sampleWaitTime(trackletNo) = true;   
                end
                %%%%%%%%%%%%%%%%%%%%%%%%%
                % Check if sampled wait time is reached
                if flag.sampleWaitTime(trackletNo) && ~flag.startCross(trackletNo)
                    % >= to allow for the pedestrian to reach the crosswalk
                    % if they are still approaching
                    if timeStep >= GapCheckTimeInHorizon(gapId-1) + timeStartCross
                        flag.startCross(trackletNo) = true;
                        if trackletNo>1
                            x=1;
                        end
                    else
                        flag.startCross(trackletNo) = false;
                    end
                end  
                %%%%%%%%%%%%%%%%%%%%%%%%%
                % Step 4b: (Walkaway) Check if the pedestrian has
                % reached walkaway state after crossing
                if size(trackletData{trackletNo}.trackLifetime,1) >= 2
                    if ( ( strcmp(trackletData{trackletNo}.HybridState(end),'Walkaway') && strcmp(trackletData{trackletNo}.HybridState(end-1),'Crossing') ) ||...
                         ( strcmp(trackletData{trackletNo}.HybridState(end),'Approach') && strcmp(trackletData{trackletNo}.HybridState(end-1),'Crossing') && trackletData{trackletNo}.closestCW(end)~=trackletData{trackletNo}.closestCW(end-1)) )
                        flag.finishedCrossing(trackletNo) = true;
                    else
                        flag.finishedCrossing(trackletNo) = false;
                    end         
                end
                %%%%%%%%%%%%%%%%%%%%%%%%%
                % Step 4c: (Reach Crosswalk) Check if the pedestrian
                % has reached the crosswalk - (a) Either in Approach or Wait State, (b) Goal Disp (dist to sidewalk) is small, (c) Was not at crosswalk before, (d) was approaching the same crosswalk                
                if ( size(trackletData{trackletNo}.trackLifetime,1) >= 2 &&...
                    (strcmp(trackletData{trackletNo}.HybridState(end),'Approach') || strcmp(trackletData{trackletNo}.HybridState(end),'Wait')) &&...
                     trackletData{trackletNo}.goalDisp(end) < 2 && ~flag.atCrosswalk(trackletNo) && ...
                     trackletData{trackletNo}.closestCW(end)==trackletData{trackletNo}.closestCW(end-1) ) 
                 
                   flag.reachCrosswalk(trackletNo) = true; 
                   flag.checkIntent(trackletNo) = true;
                   flag.atCrosswalk(trackletNo) = true;
                   % compile cross intent check features and check
                   % probability based on whether there is an ego-car
                   % or not
                   ped_CloseCar_id = trackletData{trackletNo}.closeCar_ind(end);

                   if (ped_CloseCar_id ~=0 && ped_CloseCar_id ~=inf && ~isempty(carTrackCurrentTimeStep))
                       % compile features for estimating crossing intent
                       [CrossFeatures, flag] = compileCrossFeatures(currentPedData, trackletData, trackletNo, trackletStartNode, trackletEndNode, pedTrackTimeStep, timeStep, Params, flag, currentTSEgoCarData, carTrackCurrentTimeStep(egoActiveCarInd));
                       
                       % flag gets updated if there are no close vehicles,
                       % when veh-ped dist is inf
                       if ~flag.checkIntentWOEgo(trackletNo)
                           CrossFeatures.pedTrackTimeStep = pedTrackTimeStep;
                           CrossFeatures.timeStepInHorizon = timeStep;
                           mean_veh_speed = CrossFeatures.mean_veh_speed;
                           mean_veh_acc = CrossFeatures.mean_veh_acc;
                           mean_DTCurb = CrossFeatures.mean_DTCurb;
                           mean_veh_ped_dist = CrossFeatures.mean_veh_ped_dist;
                           mean_ped_speed = CrossFeatures.mean_ped_speed;
                           gaze_ratio = CrossFeatures.gaze_ratio;
                           mean_DTCW = CrossFeatures.mean_DTCW;
                           isSamedirection = CrossFeatures.isSameDirection;
                           isNearLane = CrossFeatures.isNearLane;
                           duration_ego_vehicle = CrossFeatures.duration_ego_vehicle;
                           % predict
                           CrossSVMFeatures = table(mean_veh_speed, mean_veh_acc, mean_DTCurb , mean_veh_ped_dist, duration_ego_vehicle,...
                                                    mean_ped_speed, gaze_ratio, mean_DTCW, isSamedirection, isNearLane);
                           [~, prob_CI_outputs] = predict(Prob_CrossIntentModelCar, CrossSVMFeatures);
                            trackletData{trackletNo}.probCrossingIntent = prob_CI_outputs(2);
                       end
                   else
                       flag.checkIntentWOEgo(trackletNo) = true;
                   end
                   % check crossIntent 
                   if flag.checkIntentWOEgo(trackletNo)
                       [CrossFeatures, flag] = compileCrossFeatures(currentPedData, trackletData, trackletNo, trackletStartNode, trackletEndNode, pedTrackTimeStep, timeStep, Params, flag);
                       mean_DTCurb = CrossFeatures.mean_DTCurb;
                       mean_ped_speed = CrossFeatures.mean_ped_speed;
                       mean_DTCW = CrossFeatures.mean_DTCW;
                       % predict
                       CrossSVMFeatures = table(mean_DTCurb , mean_ped_speed, mean_DTCW);
                       [~, prob_CI_outputs] = predict(Prob_CrossIntentModelNoCar, CrossSVMFeatures);
                       trackletData{trackletNo}.probCrossingIntent = prob_CI_outputs(2);
                   end
                   
                   
                   % save cross intent features
                   CrossFeatures.pedTrackTimeStep = pedTrackTimeStep;
                   CrossFeatures.timeStepInHorizon = timeStep;
                   CrossFeatures.closestCW = currentPedData.closestCW(pedTrackTimeStep);
                   predCrossFeatures{crossId} = CrossFeatures;
                   crossId = crossId + 1;
                   
                else
                    flag.reachCrosswalk(trackletNo) = false;                         
                end

                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %% 5) Has any event occured and it is not a new
                % tracklet?
%                 if  ((flag.startCross(trackletNo) || flag.finishedCrossing(trackletNo) || flag.reachCrosswalk(trackletNo) || timeStep == predHorizon) ...
%                       && size(trackletData{trackletNo}.trackLifetime,1) > 1)
                if ( (timeStep == predHorizon) || ...
                     (flag.startCross(trackletNo) && ( flag.atCrosswalk(trackletNo)||flag.startingFromWait(trackletNo) )  && ~flag.predHorizonEnd(trackletNo)) ||...
                     (flag.reachCrosswalk(trackletNo) && ~flag.predHorizonEnd(trackletNo)) || ...
                     (flag.finishedCrossing(trackletNo) && ~flag.predHorizonEnd(trackletNo)) )
                           
                    %update event flag
                    trackletEventFlag(trackletNo) = true;  
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%
%                     % update the current tracklet
%                     kf = predictionKFtracklet{newTrackletId};
%                     [tempData, kf] = updatePedContStates(kf, trackletData{end}, AVStates, cw, Params, reset, trackletGoal(end,:), timeStep);
%                     trackletData{end} = tempData;
%                     trackletKfData{trackletNo}(end+1, :) = [kf.x', diag(kf.P)']; 
%                     predictionKFtracklet{trackletNo} = kf;
%                     tempData2 = hybridState_v2(trackletData{trackletNo}(end,:), cw, flag, annotatedImageEnhanced, Params, trackletNo);
%                     trackletData{trackletNo}(end,:) = tempData2;
%                     currentTSPedEgoData = egoCarFunc(trackletData{trackletNo}, currentTSActiveCarData, cw, annotatedImageEnhanced,  Params, timeStep);                                       
%                     trackletData{trackletNo}.longDispPedCw(end,1) = currentTSPedEgoData.longDispPedCw;
%                     trackletData{trackletNo}.latDispPedCw(end,1) = currentTSPedEgoData.latDispPedCw;
%                     trackletData{trackletNo}.closeCar_ind(end,1) = currentTSPedEgoData.closeCar_ind; 
%                     trackletData{trackletNo}.activeCar_ind(end,1) = currentTSPedEgoData.activeCar_ind; 
%                     trackletData{trackletNo}.lonVelocity(end,1) = currentTSPedEgoData.lonVelocity; 
%                     trackletData{trackletNo}.long_disp_ped_car(end,1) = currentTSPedEgoData.long_disp_ped_car; 
%                     %%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %note the end node for the current tracklet
                    node_no = node_no + 1;
                    trackletEndNode(trackletNo) = node_no;
                                                          
                    %%%%%%%%%%%%%%%%%%%%%%%%%
                    %create a new tracklet based on the event
                    % 1) if end of prediction horizon (no more
                    % tracklets should be created in this case)
                    if timeStep == predHorizon
                            % current tracklet becomes inactive
                            trackletIsActive(trackletNo) = false;
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            trackletData{trackletNo}.HybridState(end+1,:) = strings;
                            %%%%%%%%%%%%%%%%%%%%%%%%%
                            % update states and other parameters of
                            % pedestrian for the tracklet's last step
                            kf = predictionKFtracklet{trackletNo};
                            [tempData, kf] = updatePedContStates(kf, trackletData{trackletNo}, trackletNo, AVStates, cw, Params, reset, trackletGoal(trackletNo,:), timeStep, flag);
                            trackletData{trackletNo} = tempData; 
                            trackletKfData{trackletNo}(end+1, :) = [kf.x', diag(kf.P)']; 
                            predictionKFtracklet{trackletNo} = kf;
                            % Update the hybrid state and Close CW
                            tempData2 = hybridState_v2(trackletData{trackletNo}, cw, flag, annotatedImageEnhanced, Params, trackletNo, reset);
                            trackletData{trackletNo} = tempData2;
                            % Find the ego-car (note that timeStep is incremented by '1' as ego-car for current time step is needed) and update the ped-car states
                            tempData = egoCarFunc(trackletData{trackletNo}, currentTSActiveCarData, carTrackCurrentTimeStep, cw, annotatedImageEnhanced,  Params, timeStep, reset);                                       
                            trackletData{trackletNo} = tempData;
                            %%%%%%%%%%%%%%%%%%%%%%%%%
                            % update flag
                            flag.predHorizonEnd(trackletNo) = true;
                            %%%%%%%%%%%%%%%%%%%%%%%
                    % 2) if (a) gap was accepted and sampled wait time is reached, and (b) pedestrian reached
                    % crosswalk (or at crosswalk) or started from wait, and
                    % (c) not the end of prediction horizon
                    elseif (flag.startCross(trackletNo) && ( flag.atCrosswalk(trackletNo)||flag.startingFromWait(trackletNo) )  && ~flag.predHorizonEnd(trackletNo))
                            % current tracklet becomes inactive
                            trackletIsActive(trackletNo) = false;
                            %%%%%%%%%%%%%%%%%%%%%%%%%
                            % 2a) create a tracklet for 'wait' (if non-zero gap
                            % rejection probability)
                            if (trackletData{trackletNo}.probGapAccept~=1)
                                % create a tracklet for starting to wait
                                [trackletData, trackletKfData, trackletProbability, trackletStartNode, trackletEndNode, trackletIsActive, trackletEventFlag, trackletGoal, newTrackletId, flag] = newTracklet(trackletData, trackletKfData, trackletProbability, trackletStartNode, trackletEndNode, trackletIsActive, trackletEventFlag, trackletGoal, node_no, 1-trackletData{trackletNo}.probGapAccept, newTrackletId, trackletNo, flag);
                                parentTracklet(newTrackletId) = trackletNo;
                                predictionKFtracklet{newTrackletId} = predictionKFtracklet{newTrackletId-1};
                                % update the hybrid state
                                trackletData{end}.HybridState(end,:) = 'Wait'; 
                                %%%%%%%%%%%%%%%%%%%%%%%%%
%                                 % update states and other parameters of
%                                 % pedestrian for the first tracklet time step
                                kf = predictionKFtracklet{newTrackletId};
                                [tempData, kf] = waitReset(trackletData{newTrackletId}, kf, trackletNo, Params, cw, reset); 
                                tempData.waitTimeSteps(end) = tempData.waitTimeSteps(end) + reSampleRate;
                                trackletData{newTrackletId} = tempData; 
                                trackletKfData{newTrackletId}(1, :) = [kf.x', diag(kf.P)']; 
                                predictionKFtracklet{newTrackletId} = kf;                               
                                % Find the ego-car (note that timeStep is incremented by '1' as ego-car for current time step is needed) and update the ped-car states
                                tempData = egoCarFunc(trackletData{newTrackletId}, currentTSActiveCarData, carTrackCurrentTimeStep, cw, annotatedImageEnhanced,  Params, timeStep, reset);                                       
                                trackletData{newTrackletId} = tempData;
                                %%%%%%%%%%%%%%%%%%%%%%%%%
                                % update the tracklets flags
                                flag.hybridStatePred(newTrackletId) = true;
                                %%%%%%%%%%%%%%%%%%%%%%%%%
                            end
                            %%%%%%%%%%%%%%%%%%%%%%%%%
                            % 2b) create a tracklet for 'Crossing' (if non-zero gap
                            % acceptance probability)
                            if (trackletData{trackletNo}.probGapAccept~=0)
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                % create a tracklet for starting to cross
                                [trackletData, trackletKfData, trackletProbability, trackletStartNode, trackletEndNode, trackletIsActive, trackletEventFlag, trackletGoal, newTrackletId, flag] = newTracklet(trackletData, trackletKfData, trackletProbability, trackletStartNode, trackletEndNode, trackletIsActive, trackletEventFlag, trackletGoal, node_no, trackletData{trackletNo}.probGapAccept, newTrackletId, trackletNo, flag);
                                parentTracklet(newTrackletId) = trackletNo;
                                trackletData{newTrackletId}.HybridState(end,:) = 'Crossing'; 
                                predictionKFtracklet{newTrackletId} = predictionKFtracklet{newTrackletId-1};                       
                                %%%%%%%%%%%%%%%%%%%%%%%%%
                                % update states and other parameters of
                                % pedestrian for the first tracklet time step
                                kf = predictionKFtracklet{newTrackletId};
                                [tempData, kf] = updatePedContStates(kf, trackletData{newTrackletId}, trackletNo, AVStates, cw, Params, reset, trackletGoal(newTrackletId,:), timeStep, flag);
                                trackletData{end} = tempData; 
                                trackletData{newTrackletId}.trackLifetime(end,:) = trackletData{newTrackletId-1}.trackLifetime(end,:) ; % reset track life time (otherwise 5 frames gets added)
                                trackletKfData{newTrackletId}(1, :) = [kf.x', diag(kf.P)']; 
                                predictionKFtracklet{newTrackletId} = kf;
                                %%%%%%%%%%%%%%%%%%%%%%%%%
                                % update other parameters of pedestrian for the last
                                % time step
                                flag.hybridStatePred(end) = true;
                                tempData2 = hybridState_v2(trackletData{newTrackletId}, cw, flag, annotatedImageEnhanced, Params, newTrackletId, reset);
                                trackletData{newTrackletId} = tempData2;
                                % Find the ego-car (note that timeStep is incremented by '1' as ego-car for current time step is needed) and update the ped-car states
                                tempData = egoCarFunc(trackletData{newTrackletId}, currentTSActiveCarData, carTrackCurrentTimeStep, cw, annotatedImageEnhanced,  Params, timeStep, reset);                                       
                                trackletData{newTrackletId} = tempData;
                                %%%%%%%%%%%%%%%%%%%%%%%%%
                                % update flags
                                flag.hybridStatePred(end) = true;
                                %%%%%%%%%%%%%%%%%%%%%%%%%
                            end
                            %%%%%%%%%%%%%%%%%%%%%%%%%
                           % reset flags for gap acceptance event
%                            flag.startCross(trackletNo) = false; % so that whether is reached is not checked again for the parent tracklet
                           % reset probabilities
                           trackletData{trackletNo}.probGapAccept = 0;
                    %%%%%%%%%%%%%%%%%%%%%%%%%
                    % 3) if pedestrian reached the crosswalk
                    elseif (flag.reachCrosswalk(trackletNo) && ~flag.predHorizonEnd(trackletNo))
                            % current tracklet becomes inactive
                            trackletIsActive(trackletNo) = false;
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
                            % 3a) create a tracklet for not having a crossing
                            % intent
                            if trackletData{trackletNo}.probCrossingIntent~=1
                                [trackletData, trackletKfData, trackletProbability, trackletStartNode, trackletEndNode, trackletIsActive, trackletEventFlag, trackletGoal, newTrackletId, flag] = newTracklet(trackletData, trackletKfData, trackletProbability, trackletStartNode, trackletEndNode, trackletIsActive, trackletEventFlag, trackletGoal, node_no, 1-trackletData{trackletNo}.probCrossingIntent, newTrackletId, newTrackletId, flag);
                                parentTracklet(newTrackletId) = trackletNo;
                                trackletData{end}.HybridState(end,:) = 'Approach'; 
                                predictionKFtracklet{newTrackletId} = predictionKFtracklet{newTrackletId-1};                          
                                %%%%%%%%%%%%%%%%%%%%%%%%%
                                % update the heading for the approach
                                % tracklet based on the next goal location
                                kf = predictionKFtracklet{newTrackletId};
                                [tempData, kf] = approachReset(trackletData{newTrackletId}, kf, Params, cw, reset, flag.reachCrosswalk(trackletNo)); 
                            %                                 [tempData, kf] = updatePedContStates(kf, trackletData{end}, AVStates, cw, Params, reset, trackletGoal(end,:), timeStep);
                                trackletData{newTrackletId} = tempData; 
                                trackletKfData{newTrackletId}(1, :) = [kf.x', diag(kf.P)']; 
                                predictionKFtracklet{newTrackletId} = kf;
                                % update close CW
                                tempData2 = hybridState_v2(trackletData{newTrackletId}, cw, flag, annotatedImageEnhanced, Params, newTrackletId, reset);
                                trackletData{newTrackletId} = tempData2;
                                trackletData{end}.HybridState(end,:) = 'Approach';  % in case it was altered during hybrid state update
                                % Find the ego-car (note that timeStep is incremented by '1' as ego-car for current time step is needed) and update the ped-car states
                                tempData = egoCarFunc(trackletData{newTrackletId}, currentTSActiveCarData, carTrackCurrentTimeStep, cw, annotatedImageEnhanced,  Params, timeStep, reset);                                       
                                trackletData{newTrackletId} = tempData;
                                %%%%%%%%%%%%%%%%%%%%%%%%%
                                % update the tracklets flags
                                flag.hybridStatePred(newTrackletId) = true;
                                flag.atCrosswalk(newTrackletId) = true;                              
                            end
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            % 3b) create a tracklet for having a crossing intent
                            if trackletData{trackletNo}.probCrossingIntent
                               [trackletData, trackletKfData, trackletProbability, trackletStartNode, trackletEndNode, trackletIsActive, trackletEventFlag, trackletGoal, newTrackletId, flag] = newTracklet(trackletData, trackletKfData, trackletProbability, trackletStartNode, trackletEndNode, trackletIsActive, trackletEventFlag, trackletGoal, node_no, trackletData{trackletNo}.probCrossingIntent, newTrackletId, newTrackletId-1, flag);
                               parentTracklet(newTrackletId) = trackletNo; 
                               trackletData{newTrackletId}.HybridState(end,:) = 'Wait';  %instantaneous wait
                            %                                 WaitStartTimeInHorizon = timeStep;
                                predictionKFtracklet{newTrackletId} = predictionKFtracklet{newTrackletId-1};
                                % update the gap acceptance probability of
                                % the parent tracklet
                                trackletData{newTrackletId}.probGapAccept = trackletData{parentTracklet(newTrackletId)}.probGapAccept;                            
                                %%%%%%%%%%%%%%%%%%%%%%%%%
                                % update the heading for the approach
                                % tracklet based on the next goal
                                % location
                                kf = predictionKFtracklet{newTrackletId};
                                [tempData, kf] = waitReset(trackletData{newTrackletId}, kf, trackletNo, Params, cw, reset); 
                            %                             [tempData, kf] = updatePedContStates(kf, trackletData{end}, AVStates, cw, Params, reset, trackletGoal(end,:), timeStep);
                                tempData.waitTimeSteps(end) =  tempData.waitTimeSteps(end) + reSampleRate;
                                trackletData{newTrackletId} = tempData; 
                                trackletKfData{newTrackletId}(1, :) = [kf.x', diag(kf.P)']; 
                                predictionKFtracklet{newTrackletId} = kf;
                                % Find the ego-car (note that timeStep is incremented by '1' as ego-car for current time step is needed) and update the ped-car states
                                tempData = egoCarFunc(trackletData{newTrackletId}, currentTSActiveCarData, carTrackCurrentTimeStep, cw, annotatedImageEnhanced,  Params, timeStep, reset);                                       
                                trackletData{newTrackletId} = tempData;
                                %%%%%%%%%%%%%%%%%%%%%%%%%
                                % update the tracklets flags
                                flag.GapStart(end) = flag.GapStart(trackletNo); % update the gap check flag for the new tracklet (only for 'Wait' tracklet) with the current gap start flag.
                                flag.atCrosswalk(end) = true;
                                flag.hybridStatePred(end) = true;
                                flag.startingFromWait(end) = true;
                            end
                            %reset probabilities
                            trackletData{trackletNo}.probCrossingIntent = 0;
                            %%%%%%%%%%%%%%%%%%%%%%%%%

                    %%%%%%%%%%%%%%%%%%%%%%%%%
                    % 4) if pedestrian finished crossing
                    elseif (flag.finishedCrossing(trackletNo) && ~flag.predHorizonEnd(trackletNo))
                            % current tracklet becomes inactive
                            trackletIsActive(trackletNo) = false;
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                            % create a tracklet for turning and approching another
                            % crosswalk at the intersection
                            [trackletData, trackletKfData, trackletProbability, trackletStartNode, trackletEndNode, trackletIsActive, trackletEventFlag, trackletGoal, newTrackletId, flag] = newTracklet(trackletData, trackletKfData, trackletProbability, trackletStartNode, trackletEndNode, trackletIsActive, trackletEventFlag, trackletGoal, node_no, 0.5, newTrackletId, trackletNo, flag);
                            parentTracklet(newTrackletId) = trackletNo;
                            trackletData{end}.HybridState(end,:) = 'Approach';
                            trackletGoal(end) = "Approach"; %This tracklet approaches the next crosswalk
                            predictionKFtracklet{newTrackletId} = predictionKFtracklet{newTrackletId-1};
                            %%%%%%%%%%%%%%%%%%%%%%%%%
                            % update states and other parameters of
                            % pedestrian for the first tracklet time step
                            kf = predictionKFtracklet{newTrackletId};
                            [tempData, kf] = updatePedContStates(kf, trackletData{end}, trackletNo, AVStates, cw, Params, reset, trackletGoal(end,:), timeStep, flag);
                            trackletData{newTrackletId} = tempData; 
                            trackletKfData{newTrackletId}(1, :) = [kf.x', diag(kf.P)']; 
                            predictionKFtracklet{newTrackletId} = kf;
                            % update hybrid state and close CW
                            tempData2 = hybridState_v2(trackletData{newTrackletId}, cw, flag, annotatedImageEnhanced, Params, newTrackletId, reset);
                            trackletData{newTrackletId} = tempData2;                            
                            % Find the ego-car (note that timeStep is incremented by '1' as ego-car for current time step is needed) and update the ped-car states
                            tempData = egoCarFunc(trackletData{newTrackletId}, currentTSActiveCarData, carTrackCurrentTimeStep, cw, annotatedImageEnhanced,  Params, timeStep, reset);                                       
                            trackletData{newTrackletId} = tempData;
                            %%%%%%%%%%%%%%%%%%%%%%%%%
                            % update flag
                            flag.hybridStatePred = true;

                            %%%%%%%%%%%%%%%%%%%%%%%%%
                            % create a tracklet for turning and walking away
                            % from the crosswalk
                            [trackletData, trackletKfData, trackletProbability, trackletStartNode, trackletEndNode, trackletIsActive, trackletEventFlag, trackletGoal, newTrackletId, flag] = newTracklet(trackletData, trackletKfData, trackletProbability, trackletStartNode, trackletEndNode, trackletIsActive, trackletEventFlag, trackletGoal, node_no, 0.5, newTrackletId, trackletNo, flag);
                            parentTracklet(newTrackletId) = trackletNo;
                            trackletData{end}.HybridState(end,:) = 'Walkaway'; 
                            trackletGoal(end,:) = 'Walkaway'; %This tracklet walks away
                            % for walkaway tracklet, update the crosswalk to the crosswalk they
                            % crossed, not the one they are going to
                            % approach
                            trackletData{end}.closestCW(end) =  trackletData{trackletNo}.closestCW(end-1); 
                            if trackletData{trackletNo}.closestCW(end-1)==3 && trackletData{trackletNo}.closestCW(end) ==1
                                x=1;
                            end
                            %%%%%%%%%%%%%%%%%%%%%%%%%
                            % update states and other parameters of
                            % pedestrian for the first tracklet time step
                            predictionKFtracklet{newTrackletId} = predictionKFtracklet{newTrackletId-1};
                            kf = predictionKFtracklet{newTrackletId};
                            [tempData, kf] = updatePedContStates(kf, trackletData{end}, trackletNo, AVStates, cw, Params, reset, trackletGoal(end,:), timeStep, flag);
                            trackletData{end} = tempData; 
                            trackletKfData{newTrackletId}(1, :) = [kf.x', diag(kf.P)']; 
                            predictionKFtracklet{newTrackletId} = kf;
                            % update hybrid state and close CW
                            tempData2 = hybridState_v2(trackletData{newTrackletId}, cw, flag, annotatedImageEnhanced, Params, newTrackletId, reset);
                            trackletData{newTrackletId} = tempData2;                            
                            % Find the ego-car (note that timeStep is incremented by '1' as ego-car for current time step is needed) and update the ped-car states
                            tempData = egoCarFunc(trackletData{newTrackletId}, currentTSActiveCarData, carTrackCurrentTimeStep, cw, annotatedImageEnhanced,  Params, timeStep, reset);                                       
                            trackletData{newTrackletId} = tempData;
                            %%%%%%%%%%%%%%%%%%%%%%%%%
                            % update the tracklets flags
                            flag.hybridStatePred(end) = true; % it was true anyway
                            %%%%%%%%%%%%%%%%%%%%%%%%%
                    end
                else
                        %%%%%%%%%%%%%%%%%%%%%%%%%
                        % no event
                        trackletEventFlag(trackletNo) = false;  
%                           flag.newTracklet(trackletNo) = false;
                end     % end of event check
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                %% update discrete state if there is no new event 
                if ~trackletEventFlag(trackletNo)                   
%                         % check guard conditions for discrete state
%                         tempData = hybridState_v2(predictionTracklet.data{tracklet_no}(pred_time_step-1,:),cw, flag, annotatedImage_enhanced, Params);
                    % cross after instantaneous wait if there was no gap check in the parent tracklet 
%                     if (~flag.startCross(parentTracklet(trackletNo)) && size(trackletData{trackletNo}.trackLifetime, 1)==1 && strcmp(trackletData{trackletNo}.HybridState(end,:), 'Wait') )
%                         trackletData{trackletNo}.HybridState(end+1,:) = 'Crossing';
%                         flag.hybridStatePred(trackletNo) = true;
%                     else
                        trackletData{trackletNo}.HybridState(end+1,:) = trackletData{trackletNo}.HybridState(end,:); 
                        flag.hybridStatePred(trackletNo) = false;
%                     end
                end                 
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %% update the continuous state of pedestrian if the tracklet is still active
                if trackletIsActive(trackletNo)
                    % update close CW first
%                     flag.hybridStatePred(trackletNo) = true; % to avoid hybrid state calculation but only perform close CW calculation
%                     tempData2 = hybridState_v2(trackletData{trackletNo}(end,:), cw, flag, annotatedImageEnhanced, Params, trackletNo);
%                     trackletData{trackletNo}(end,:) = tempData2;
                    % continuous state updates
                    kf = predictionKFtracklet{trackletNo};
                    [tempData, kf] = updatePedContStates(kf, trackletData{trackletNo}, trackletNo, AVStates, cw, Params, reset, trackletGoal(trackletNo,:), timeStep, flag);
                    trackletData{trackletNo} = tempData; 
                    trackletKfData{trackletNo}(end+1, :) = [kf.x', diag(kf.P)']; 
                    predictionKFtracklet{trackletNo} = kf;
                    %%%%%%%%%%%%%%%%%%%%%%%%%
                    % update other parameters of pedestrian for the last
                    % time step
                    tempData2 = hybridState_v2(trackletData{trackletNo}(end,:), cw, flag, annotatedImageEnhanced, Params, trackletNo, reset);
                    trackletData{trackletNo}(end,:) = tempData2;
                    % Find the ego-car in the current time step and save the car track index
                    tempData = egoCarFunc(trackletData{trackletNo}, currentTSActiveCarData, carTrackCurrentTimeStep, cw, annotatedImageEnhanced,  Params, timeStep, reset);                                       
                    trackletData{trackletNo} = tempData;
                    %%%%%%%%%%%%%%%%%%%%%%%%%
                end
        end % end of current prediction tracklet
    end % end of loop for all prediction tracklets  
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% predict the state of all the active cars
%     if ~isempty(currentTSActiveCarData)
%         currentTSActiveCarData = updateCarState(currentTSActiveCarData, AVStates, Params, reset, cw);
%     end  
    %%%%%%%%%%%%%%%%%%%%%%%%%
%     plot predicted vehicle and pedestrian states
    if (pedPosPixels(1)>=100 && pedPosPixels(1)<=950 && pedPosPixels(2)>=-560 && pedPosPixels(2)<=-100)
        for ii=1:size(currentTSActiveCarData,1)
            carPosPixels = int32([currentTSActiveCarData{ii}.xCenter(timeStep), currentTSActiveCarData{ii}.yCenter(timeStep)]/(orthopxToMeter*scaleFactor));
            annotatedImageEnhanced(-carPosPixels(2), carPosPixels(1)) = 150;
        end
        
        for jj=1:size(trackletData,1)
            annotatedImageEnhanced(int32(-trackletData{jj}.yCenter(end)/(orthopxToMeter*scaleFactor)), int32(trackletData{jj}.xCenter(end)/(orthopxToMeter*scaleFactor))) = 75;
        end
    end
    imshow(annotatedImageEnhanced);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

     
end  % end of loop for prediction horizon
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% compile predictions
% compile the start and end nodes for each prediction tracklet
N_tracklets = size(trackletData,1);

%% debug
if N_tracklets==7
    x=1;
end


%%%%%%%%%%%%%%%%%%%%%%%%%
% initialize the first prediction future
predictionTrajectory{1,1}(1,1) = trackletEndNode(1);
predictionTrajectory{1,1}(1,2) = trackletProbability(1);

tempTraj = [trackletData{1}.xCenter(N_obsTimeSteps+1:end), trackletData{1}.yCenter(N_obsTimeSteps+1:end)]';
predictionTrajectory{1,1} = [predictionTrajectory{1,1}, tempTraj(:)'];
tempTrajKF = trackletKfData{1}';
predictionKFtrajectory{1,1} = tempTrajKF(:)';
%%%%%%%%%%%%%%%%%%%%%%%%%
% compile all the different prediction futures
N_new = 1;
temp_predcopying = zeros(N_tracklets,1);
predCopying = true;
while(predCopying)
      for ii=1:size(predictionTrajectory,1)
            nextTracklet = find(trackletStartNode==predictionTrajectory{ii}(1,1));
            if (~isempty(nextTracklet))
                temp_predcopying(ii) = 1;
                for jj = length(nextTracklet):-1:1 %the first tracklet gets added to the first prediction
                   trackletId = nextTracklet(jj);
                   if jj~=1
                        N_new = N_new + 1;
                        predictionTrajectory{N_new,1} = predictionTrajectory{ii};
                        tempTraj = [trackletData{trackletId}.xCenter(1:end), trackletData{trackletId}.yCenter(1:end)]';
                        predictionTrajectory{N_new} = [predictionTrajectory{N_new}, tempTraj(:)']; %the first entry is the end of previous tracklet and is already accounted for
                        predictionTrajectory{N_new}(1,1) = trackletEndNode(trackletId);
                        predictionTrajectory{N_new}(1,2) = trackletProbability(trackletId);                          
                        predictionKFtrajectory{N_new,1} = predictionKFtrajectory{ii};
                        tempTraj =  trackletKfData{trackletId}(1:end,:)';
                        predictionKFtrajectory{N_new} = [predictionKFtrajectory{N_new},tempTraj(:)'];                        
                        temp_predcopying(N_new) = 1;
                   else
                        tempTraj = [trackletData{trackletId}.xCenter(1:end), trackletData{trackletId}.yCenter(1:end)]';
                        predictionTrajectory{ii} = [predictionTrajectory{ii}, tempTraj(:)'];
                        predictionTrajectory{ii}(1,1) = trackletEndNode(trackletId);
                        predictionTrajectory{ii}(1,2) = predictionTrajectory{ii}(1,2) * trackletProbability(trackletId);
                        tempTraj = trackletKfData{trackletId}(1:end,:)';
                        predictionKFtrajectory{ii} = [predictionKFtrajectory{ii}, tempTraj(:)'];
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




% copy the cell to a matrix
predictionTrajectoryMatrix = double(cell2mat(predictionTrajectory));
% remove empty data
ind = 1;
for ii = 1:10
    if ~isempty(predGapFeatures{ind})
        ind = ind+1;
    end 
end
if ind > 1
    predGapFeatures = predGapFeatures{1:ind-1}; 
else
    predGapFeatures = [];
end
ind = 1;
for ii = 1:10
    if ~isempty(predCrossFeatures{ind})
        ind = ind+1;
    end 
end
if ind > 1
    predCrossFeatures = predCrossFeatures{1:ind-1}; 
else
    predCrossFeatures = [];
end

%     %% debug: plot predicted trajectories
%     figure()
%     for ii=1:size(predictionTrajectoryMatrix,1)
%         tempPredMatrix = reshape(predictionTrajectoryMatrix(ii,end-2*Params.predHorizon+1:end),[2, Params.predHorizon])';
%         plot(tempPredMatrix(:,1), tempPredMatrix(:,2), '*', 'MarkerSize', 10); hold on;
%     end
%     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
end % end of the function