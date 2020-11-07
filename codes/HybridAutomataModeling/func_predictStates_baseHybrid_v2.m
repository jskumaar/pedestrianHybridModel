%% This script performs the prediction of the pedestrian
% Note: the currentPedData can have multiple futures;
% run prediction if there is an ego-vehicle approaching the pedestrian
% in the current time step.

% function [predictionTrajectoryMatrix, predictionKFtrajectory, predGapFeatures, predCrossFeatures] = predictStates(kf, currentPedData, ~, currentTSActiveCarData, crossCarData, AVStates, pedTrackTimeStep, cw, annotatedImageEnhanced, resetStates, Prob_GapAcceptanceModel, Prob_CrossIntentModelCar, Prob_CrossIntentModelNoCar, Params, flag)
function [predictionTrajectoryMatrix, predictionKFtrajectory, predGapFeatures, predCrossFeatures] = func_predictStates_baseHybrid_v2(kf, currentPedData, ~, currentTSActiveCarData, carTrackCurrentTimeStep, AVStates, pedTrackTimeStep, cw, annotatedImageEnhanced, resetStates, Prob_GapAcceptanceModel, Params, flag)

%% 1) setup
% load the exponential distribution of wait times
load('waitTimeWithEgo_expDist.mat') 
waitDistribution = waitTimeWithEgo_expDist;
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
flag.startToCross = false;
flag.startedCrossing = false;
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
flag.finishedCrossingDelayReached = false;
gapId = 1;
crossId = 1;
newTrackletId = 1;
GapCheckTimeInHorizon = zeros(1,20);
predGapFeatures = cell(10,1);
predCrossFeatures = cell(10,1);
predictionKFtracklet = cell(10,1);
walkawayCW = 0;
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
%%%%%%%%%%%%%%%%%%%%%%%
% Flags
% Starting from Wait?
if strcmp(trackletData{1}.HybridState(:,end),'Wait')
    flag.startingFromWait = true;
end
resetFlag.approachReset = false;
resetFlag.walkawayReset = false;
resetFlag.check_goal = false;
resetFlag.sample_goal = false;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
% Find the ego-car in the current time step get the vehicle-pedestrian
% parameters
tempData = func_egoCarFunc(trackletData{1}, currentTSActiveCarData, carTrackCurrentTimeStep, cw, annotatedImageEnhanced,  Params, 1, resetStates);                                       
trackletData{1} = tempData;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                       
%% debug
currentTSActiveCarData_copy = currentTSActiveCarData;
% a) ground truth
endTimeStep = min(length(currentPedData.xCenter), pedTrackTimeStep+predHorizon);
trajectory_GT = [currentPedData.xCenter(pedTrackTimeStep+1:endTimeStep), currentPedData.yCenter(pedTrackTimeStep+1:endTimeStep)];
trajectory_CV = [currentPedData.xCenter(pedTrackTimeStep), currentPedData.yCenter(pedTrackTimeStep)];
vel = [currentPedData.xVelocity(pedTrackTimeStep), currentPedData.yVelocity(pedTrackTimeStep)];
for yy = 1:predHorizon
    trajectory_CV(end+1,:) = trajectory_CV(end,:) + vel*Params.delta_T;
end
trajectory_CV = trajectory_CV(2:end,:);
%%%%%%%%

%% 3) Prediction loop 
for timeStep = 1:predHorizon             
    % update the current number of tracklets (new tracklets can be formed
    % within the inner for loop)
    N_tracklets = size(trackletData,1);  
    
    % check active vehicles: if there is no observed car data aavailable for this prediction time step, remove that car from the list of active cars
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
    
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % for each tracklet predict the trajectory at this pred time step
    for trackletNo = 1:N_tracklets                    
        %update the future only if this is an active tracklet
        if trackletIsActive(trackletNo)
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %% do not consider the pedestrians if they are out of range of the ego-vehicle or out of range of the environment at the prediction time step
                pedPosPixels = [trackletData{trackletNo}.xCenter(end),  trackletData{trackletNo}.yCenter(end)]/(orthopxToMeter*scaleFactor);              
                if timeStep <=length(AVStates.carHeading)
                    AVHeading = AVStates.carHeading(timeStep);
                    AVPosPixels  = AVStates.carPosPixels(timeStep,:);
                    dispPedAV = pedPosPixels-AVPosPixels;
                    distancePedAV = norm(dispPedAV); 
                    AVPedangle = atan2(dispPedAV(2), dispPedAV(1))*180/pi;
                else
                    AVHeading = [];
                    AVPosPixels = [];
                end
                % check if the pedestrian is out of range of the
                % ego-vehicle or out of the observation region of the
                % dataset or if the ego car is out of the observation
                % region of the dataset
                if ( ~isempty(AVHeading) && ( distancePedAV > Params.sensingRange || (abs(AVHeading-AVPedangle) > 90 && abs(AVHeading-AVPedangle) < 270) ) )       
                    flag.outOfRange(trackletNo) = true;
                else
                    flag.outOfRange(trackletNo) = false;
                end                    
                if  ( ~isempty(AVPosPixels) && ( pedPosPixels(1)<=100 || pedPosPixels(1)>=950 || pedPosPixels(2)<=-560 || pedPosPixels(2)>=-100 ) )
                    flag.outOfPlay(trackletNo) = true;
%                     break
                else
                    flag.outOfPlay(trackletNo) = false;
                end
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%             
                %% 4) Check for events that happens at the end of this pred time step
                
                % reset flag at crosswalk when there is a change in the
                % closest crosswalk
                if ( size(trackletData{trackletNo}.closestCW,1)>1 && trackletData{trackletNo}.closestCW(end)~=trackletData{trackletNo}.closestCW(end-1) )
                    flag.atCrosswalk(trackletNo) = false;
                end                
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
%                       if size(trackletData{trackletNo}.trackLifetime, 1) >= AdjustedSampFreq
                            [GapFeatures, egoVehGapHist, flag] = func_compileGapFeatures(trackletData{trackletNo}, currentTSEgoCarData, carTrackCurrentTimeStep(egoActiveCarInd), egoVehGapHist, pedTrackTimeStep, Params, flag, trackletNo, timeStep);  
%                       end                          
                else
                    flag.EgoCar(trackletNo) = false;
                end   %end of if loop for the presence of an ego-vehicle                  
                %%%%%%%%%%%%%%%%%%%%%%%%%
                % Predict the gap acceptance probability when there is
                % a new gap.  A gap can also start when the pedestrian starts from 'Wait' state.
                if ( ( flag.GapStart(trackletNo) && flag.EgoCar(trackletNo) && ~isempty(GapFeatures) && trackletData{trackletNo}.probGapAccept==0) )                     
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
                            GapCheckTimeInHorizon(gapId,trackletNo) = timeStep;
                            %%%%%%%%%%%%%%%%%%%%%%%%%
                            % save gap check variables
                            GapFeatures.predDecision = trackletData{trackletNo}.probGapAccept;
                            GapFeatures.timeStepInHorizon = timeStep;
                            predGapFeatures{gapId} = GapFeatures;           
                            % update gap id
                            gapId = gapId + 1;
                            %%%%%%%%%%%%%%%%%%%
                    % if gap starts from wait and there is no close
                    % vehicle, accept the gap
                elseif flag.startingFromWait(trackletNo) && ~flag.EgoCar(trackletNo) && ~flag.sampleWaitTime(trackletNo)
                            trackletData{trackletNo}.probGapAccept = 1;
                            GapCheckTimeInHorizon(gapId,trackletNo) = timeStep;
                            GapFeatures.timeStepInHorizon = timeStep;
                            gapId = gapId + 1;   
                end
                %%%%%%%%%%%%%%%%%%%%%%%%%
                % Sample wait time if a gap has non-zero probability
                if ( trackletData{trackletNo}.probGapAccept && ~flag.sampleWaitTime(trackletNo) )
                   % ideally a wait time has to be sampled and checked with the
                   % wait start of this pedestrian. For now, add a fixed time step
                   %time_start_cross = int32(exprnd(crossDelayExpDist.mu)/dt);
%                    timestartToCross = 0.4*AdjustedSampFreq;                 % 0.4 s second fixed delay; mean value is 0.4741 s
                   if flag.EgoCar(trackletNo)
                       timestartToCross(trackletNo) = floor((exprnd(waitDistribution.mu)/Params.delta_T));                     
                   else
                       timestartToCross(trackletNo) = 1;
                   end
                   flag.sampleWaitTime(trackletNo) = true;
                end
                %%%%%%%%%%%%%%%%%%%%%%%%%
                % Check if sampled wait time is reached
                if flag.sampleWaitTime(trackletNo) && ~flag.startToCross(trackletNo)
                    % >= to allow for the pedestrian to reach the crosswalk
                    % if they are still approaching
                    if timeStep >= GapCheckTimeInHorizon(gapId-1,trackletNo) + timestartToCross(trackletNo)
                        flag.startToCross(trackletNo) = true;
                    else
                        flag.startToCross(trackletNo) = false;
                    end
                end  
                %%%%%%%%%%%%%%%%%%%%%%%%%
                % Step 4b: (Walkaway) Check if the pedestrian has
                % reached walkaway state after crossing
                if size(trackletData{trackletNo}.trackLifetime,1) >= 2
%                     if ( ( strcmp(trackletData{trackletNo}.HybridState(end),'Walk_away') && strcmp(trackletData{trackletNo}.HybridState(end-1),'Crossing') ) ||...
%                          ( strcmp(trackletData{trackletNo}.HybridState(end),'Approach') && strcmp(trackletData{trackletNo}.HybridState(end-1),'Crossing') && trackletData{trackletNo}.closestCW(end)~=trackletData{trackletNo}.closestCW(end-1)) )
                    if ( ( strcmp(trackletData{trackletNo}.HybridState(end),'Walk_away') && strcmp(trackletData{trackletNo}.HybridState(end-1),'Crossing') ) ||...
                         ( strcmp(trackletData{trackletNo}.HybridState(end),'Approach') && strcmp(trackletData{trackletNo}.HybridState(end-1),'Crossing') ) )                    
                        flag.finishedCrossing(trackletNo) = true;
                        finishedCrossTime(trackletNo) = timeStep;
                        walkawayCW(trackletNo) = trackletData{trackletNo}.closestCW(end-1);
                    end         
                end
                % add a 2 time step delay between end of crossing and
                % start of new tracklets
                if flag.finishedCrossing(trackletNo) && ~flag.finishedCrossingDelayReached(trackletNo) && timeStep-finishedCrossTime(trackletNo)>=2
                    flag.finishedCrossingDelayReached(trackletNo) = true;
                end
                %%%%%%%%%%%%%
                %check reach goal for 1st tracklet (when there is no goal
                %location assigned)
                if trackletNo==1
                    tempFlag = flag;
                    tempTracklet = trackletData;
                    % sample a goal location
                    resetFlag.check_goal(1) = false;
                    resetFlag.sample_goal(1) = true;
                    [~,pedGoalPixels, resetFlag] = func_checkSampleGoal(tempTracklet{1}, 1, resetStates, Params, tempFlag, resetFlag);
                    tempTracklet{1}.goalPositionPixels(end,:) = pedGoalPixels;
                    % check if goalis reached
                    resetFlag.sample_goal(1) = false;
                    resetFlag.check_goal(1) = true;
                    [tempFlag,~, resetFlag] = func_checkSampleGoal(tempTracklet{1}, 1, resetStates, Params, tempFlag, resetFlag);
                    flag.reachGoal(1) = tempFlag.reachGoal(1);
                end
                %%%%%%%%%%%%%%%%%%%%%%%%%
                % Step 4c: (Reach Crosswalk) Check if the pedestrian
                % has reached the crosswalk - (a) Either in Approach or Wait State, (b) Goal Disp (dist to sidewalk) is small, (c) Was not at crosswalk before, (d) was approaching the same crosswalk                
                if ( size(trackletData{trackletNo}.trackLifetime,1) >= 2 &&...
                    (strcmp(trackletData{trackletNo}.HybridState(end),'Approach') || strcmp(trackletData{trackletNo}.HybridState(end),'Wait')) &&...
                    (trackletData{trackletNo}.goalDisp(end) < 2 || flag.reachGoal(trackletNo)) && ~flag.atCrosswalk(trackletNo) && ...
                     trackletData{trackletNo}.closestCW(end)==trackletData{trackletNo}.closestCW(end-1) ) 
                   % update flags
                   flag.reachCrosswalk(trackletNo) = true; 
                   flag.checkIntent(trackletNo) = true;
                   flag.atCrosswalk(trackletNo) = true;
                   % compile cross intent check features and check
                   % probability based on whether there is an ego-car
                   % or not
                   trackletData{trackletNo}.probCrossingIntent = 1;
                else
                    flag.reachCrosswalk(trackletNo) = false;                         
                end

                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %% 5) Has any event occured and it is not a new
                % tracklet?
                if ( (timeStep == predHorizon) || ...
                     (flag.startToCross(trackletNo) && ( flag.atCrosswalk(trackletNo)||flag.startingFromWait(trackletNo) )  && ~flag.predHorizonEnd(trackletNo)) ||...
                     (flag.reachCrosswalk(trackletNo) && ~flag.predHorizonEnd(trackletNo)) || ...
                     (flag.finishedCrossingDelayReached(trackletNo) && ~flag.predHorizonEnd(trackletNo)) )
                           
                    %update event flag
                    trackletEventFlag(trackletNo) = true;  
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %note the end node for the current tracklet
                    node_no = node_no + 1;
                    trackletEndNode(trackletNo) = node_no;
                                                          
                    %%%%%%%%%%%%%%%%%%%%%%%%%
                    % create a new tracklet based on the event
                    % 1) if end of prediction horizon (no more
                    % tracklets should be created in this case)
                    if timeStep == predHorizon
                            % current tracklet becomes inactive
                            trackletIsActive(trackletNo) = false;
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            trackletData{trackletNo}.HybridState(end+1,:)  = trackletData{trackletNo}.HybridState(end,:); 
                            flag.hybridStatePred(trackletNo) = false;
                            %%%%%%%%%%%%%%%%%%%%%%%%%
                            % update states and other parameters of
                            % pedestrian for the tracklet's last step
                            kf = predictionKFtracklet{trackletNo};
                            [tempData, kf, flag, resetFlag] = func_updatePedContStates(kf, trackletData{trackletNo}, trackletNo, Params, resetStates, timeStep, flag, resetFlag);
%                             [tempData, kf] = func_updatePedContStates(kf, trackletData{trackletNo}, trackletNo, AVStates, cw, Params, resetStates, trackletGoal(trackletNo,:), timeStep, flag);
                            trackletData{trackletNo} = tempData; 
                            trackletKfData{trackletNo}(end+1, :) = [kf.x', diag(kf.P)']; 
                            predictionKFtracklet{trackletNo} = kf;
                            % Update the hybrid state and Close CW
                            [tempData2, flag] = func_hybridState_v2(trackletData{trackletNo}, cw, flag, annotatedImageEnhanced, Params, trackletNo, resetStates);
                            trackletData{trackletNo} = tempData2;
                            % Find the ego-car (note that timeStep is incremented by '1' as ego-car for current time step is needed) and update the ped-car states
                            tempData = func_egoCarFunc(trackletData{trackletNo}, currentTSActiveCarData, carTrackCurrentTimeStep, cw, annotatedImageEnhanced,  Params, timeStep, resetStates);                                       
                            trackletData{trackletNo} = tempData;
                            %%%%%%%%%%%%%%%%%%%%%%%%%
                            % update flag
                            flag.predHorizonEnd(trackletNo) = true;
                            %%%%%%%%%%%%%%%%%%%%%%%
                    %%%%%%%%%%%%%%%%%%%%%%%%%
                    % 2) if pedestrian reached the crosswalk
                    elseif (flag.reachCrosswalk(trackletNo) && ~flag.predHorizonEnd(trackletNo))
                            % current tracklet becomes inactive
                            trackletIsActive(trackletNo) = false;
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
                            % 3a) create a tracklet for not having a crossing
                            % intent
                            if trackletData{trackletNo}.probCrossingIntent~=1
                                [trackletData, trackletKfData, trackletProbability, trackletStartNode, trackletEndNode, trackletIsActive, trackletEventFlag, trackletGoal, newTrackletId, flag, resetFlag] = func_newTracklet(trackletData, trackletKfData, trackletProbability, trackletStartNode, trackletEndNode, trackletIsActive, trackletEventFlag, trackletGoal, node_no, 1-trackletData{trackletNo}.probCrossingIntent, newTrackletId, trackletNo, flag);
                                parentTracklet(newTrackletId) = trackletNo;
                                pedHeading = atan2(trackletData{trackletNo}.yVelocity(end), trackletData{trackletNo}.xVelocity(end))*180/pi;
                                cwInd = trackletData{trackletNo}.closestCW(end);
                                Lane = trackletData{trackletNo}.Lane(end);
                                if  ( (cwInd==1 && abs(pedHeading)>90) || (cwInd==2 && abs(pedHeading)<90) || ...
                                      (cwInd==3 && pedHeading>0) || (cwInd==4 && pedHeading<0) )
                                    trackletData{end}.HybridState(end,:) = 'Approach'; 
                                    resetFlag.approachReset(newTrackletId) = true;
                                else
                                    trackletData{end}.HybridState(end,:) = 'Walk_away';
                                    resetFlag.walkawayReset(newTrackletId) = true;
                                end

                                predictionKFtracklet{newTrackletId} = predictionKFtracklet{newTrackletId-1};   
                                %%%%%%%%%%%%%%%%%%%%%%%%%
                                % update the tracklets flags
                                flag.hybridStatePred(newTrackletId) = true;
                                flag.atCrosswalk(newTrackletId) = false;
                                trackletEventFlag(newTrackletId) = false;
                                %%%%%%%%%%%%%%%%%%%%%%%%%
                                % update the heading for the approach
                                % tracklet based on the next goal location
                                kf = predictionKFtracklet{newTrackletId};
%                                 [tempData, kf] = approachReset(trackletData{newTrackletId}, kf, Params, cw, resetStates, flag.reachCrosswalk(trackletNo)); 
                                resetFlag.sample_goal(newTrackletId)  = true;
                                [tempData, kf, flag, resetFlag] = func_updatePedContStates(kf, trackletData{newTrackletId}, newTrackletId, Params, resetStates, timeStep, flag, resetFlag);
                                trackletData{newTrackletId} = tempData; 
                                trackletKfData{newTrackletId}(1, :) = [kf.x', diag(kf.P)']; 
                                predictionKFtracklet{newTrackletId} = kf;
                                % update close CW
                                [tempData2, flag] = func_hybridState_v2(trackletData{newTrackletId}, cw, flag, annotatedImageEnhanced, Params, newTrackletId, resetStates);
                                trackletData{newTrackletId} = tempData2;           
                                % Find the ego-car (note that timeStep is incremented by '1' as ego-car for current time step is needed) and update the ped-car states
                                tempData = func_egoCarFunc(trackletData{newTrackletId}, currentTSActiveCarData, carTrackCurrentTimeStep, cw, annotatedImageEnhanced,  Params, timeStep, resetStates);                                       
                                trackletData{newTrackletId} = tempData;                              
                            end
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            % 3b) create a tracklet for having a crossing intent
                            if trackletData{trackletNo}.probCrossingIntent
                               [trackletData, trackletKfData, trackletProbability, trackletStartNode, trackletEndNode, trackletIsActive, trackletEventFlag, trackletGoal, newTrackletId, flag, resetFlag] = func_newTracklet(trackletData, trackletKfData, trackletProbability, trackletStartNode, trackletEndNode, trackletIsActive, trackletEventFlag, trackletGoal, node_no, trackletData{trackletNo}.probCrossingIntent, newTrackletId, trackletNo, flag);
                               parentTracklet(newTrackletId) = trackletNo; 
                               trackletData{newTrackletId}.HybridState(end,:) = 'Wait';  %instantaneous wait
                                %%%%%%%%%%%%%%%%%%%%%%%%%
                                % update the tracklets flags
                                flag.GapStart(end) = flag.GapStart(trackletNo); % update the gap check flag for the new tracklet (only for 'Wait' tracklet) with the current gap start flag.
                                flag.atCrosswalk(end) = true;
                                flag.hybridStatePred(end) = true;
                                flag.startingFromWait(end) = true;
                                trackletEventFlag(newTrackletId) = false;
                                %
                                predictionKFtracklet{newTrackletId} = predictionKFtracklet{newTrackletId-1};
                                % update the gap acceptance probability of
                                % the parent tracklet
                                trackletData{newTrackletId}.probGapAccept = trackletData{parentTracklet(newTrackletId)}.probGapAccept;                            
                                %%%%%%%%%%%%%%%%%%%%%%%%%
                                % update the heading for the approach
                                % tracklet based on the next goal
                                % location
                                kf = predictionKFtracklet{newTrackletId};
%                                 [tempData, kf] = waitresetStates(trackletData{newTrackletId}, kf, trackletNo, Params, cw, resetStates); 
                                resetFlag.sample_goal(newTrackletId)  = true;
                                [tempData, kf, flag, resetFlag] = func_updatePedContStates(kf, trackletData{newTrackletId}, newTrackletId, Params, resetStates, timeStep, flag, resetFlag);
                                tempData.waitTimeSteps(end) =  tempData.waitTimeSteps(end) + reSampleRate;
                                trackletData{newTrackletId} = tempData; 
                                trackletKfData{newTrackletId}(1, :) = [kf.x', diag(kf.P)']; 
                                predictionKFtracklet{newTrackletId} = kf;
                                % Find the ego-car (note that timeStep is incremented by '1' as ego-car for current time step is needed) and update the ped-car states
                                tempData = func_egoCarFunc(trackletData{newTrackletId}, currentTSActiveCarData, carTrackCurrentTimeStep, cw, annotatedImageEnhanced,  Params, timeStep, resetStates);                                       
                                trackletData{newTrackletId} = tempData;
                            end
                            %resetStates probabilities
                            trackletData{trackletNo}.probCrossingIntent = 0;
                            %%%%%%%%%%%%%%%%%%%%%%%%%
                    % 3) if (a) gap was accepted and sampled wait time is reached, and (b) pedestrian reached
                    % crosswalk (or at crosswalk) or started from wait, and
                    % (c) not the end of prediction horizon
                    elseif (flag.startToCross(trackletNo) && ( flag.atCrosswalk(trackletNo)||flag.startingFromWait(trackletNo) )  && ~flag.predHorizonEnd(trackletNo) && ~flag.startedCrossing(trackletNo))
                            % current tracklet becomes inactive
                            trackletIsActive(trackletNo) = false;
                            %%%%%%%%%%%%%%%%%%%%%%%%%
                            % 2a) create a tracklet for 'wait' (if non-zero gap
                            % rejection probability)
                            if (trackletData{trackletNo}.probGapAccept~=1)
                                % create a tracklet for starting to wait
                                [trackletData, trackletKfData, trackletProbability, trackletStartNode, trackletEndNode, trackletIsActive, trackletEventFlag, trackletGoal, newTrackletId, flag, resetFlag] = func_newTracklet(trackletData, trackletKfData, trackletProbability, trackletStartNode, trackletEndNode, trackletIsActive, trackletEventFlag, trackletGoal, node_no, 1-trackletData{trackletNo}.probGapAccept, newTrackletId, trackletNo, flag);
                                parentTracklet(newTrackletId) = trackletNo;
                                predictionKFtracklet{newTrackletId} = predictionKFtracklet{newTrackletId-1};
                                % update the hybrid state
                                trackletData{end}.HybridState(end,:) = 'Wait'; 
                                trackletEventFlag(newTrackletId) = false;
                                %%%%%%%%%%%%%%%%%%%%%%%%%
                                % update the tracklets flags
                                flag.hybridStatePred(newTrackletId) = true;
                                flag.atCrosswalk(newTrackletId) = true;
                                %%%%%%%%%%%%%%%%%%%%%%%%%
%                                 % update states and other parameters of
%                                 % pedestrian for the first tracklet time step
                                kf = predictionKFtracklet{newTrackletId};
%                                 [tempData, kf] = waitresetStates(trackletData{newTrackletId}, kf, trackletNo, Params, cw, resetStates); 
                                resetFlag.sample_goal(newTrackletId)  = true;
                                [tempData, kf, flag, resetFlag] = func_updatePedContStates(kf, trackletData{newTrackletId}, newTrackletId, Params, resetStates, timeStep, flag, resetFlag);
                                tempData.waitTimeSteps(end) = tempData.waitTimeSteps(end) + reSampleRate;
                                trackletData{newTrackletId} = tempData; 
                                trackletKfData{newTrackletId}(1, :) = [kf.x', diag(kf.P)']; 
                                predictionKFtracklet{newTrackletId} = kf;                               
                                % Find the ego-car (note that timeStep is incremented by '1' as ego-car for current time step is needed) and update the ped-car states
                                tempData = func_egoCarFunc(trackletData{newTrackletId}, currentTSActiveCarData, carTrackCurrentTimeStep, cw, annotatedImageEnhanced,  Params, timeStep, resetStates);                                       
                                trackletData{newTrackletId} = tempData;
                                %%%%%%%%%%%%%%%%%%%%%%%%%
                            end
                            %%%%%%%%%%%%%%%%%%%%%%%%%
                            % 2b) create a tracklet for 'Crossing' (if non-zero gap
                            % acceptance probability)
                            if (trackletData{trackletNo}.probGapAccept~=0)
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                % create a tracklet for starting to cross
                                [trackletData, trackletKfData, trackletProbability, trackletStartNode, trackletEndNode, trackletIsActive, trackletEventFlag, trackletGoal, newTrackletId, flag, resetFlag] = func_newTracklet(trackletData, trackletKfData, trackletProbability, trackletStartNode, trackletEndNode, trackletIsActive, trackletEventFlag, trackletGoal, node_no, trackletData{trackletNo}.probGapAccept, newTrackletId, trackletNo, flag);
                                parentTracklet(newTrackletId) = trackletNo;
                                trackletData{newTrackletId}.HybridState(end,:) = 'Crossing'; 
                                flag.startedCrossing(newTrackletId) = true;
                                trackletEventFlag(newTrackletId) = false;
                                % update gap acceptance probability of
                                % parent tracklet
%                                 trackletData{newTrackletId}.probGapAccept = trackletData{trackletNo}.probGapAccept;
                                predictionKFtracklet{newTrackletId} = predictionKFtracklet{newTrackletId-1};                       
                                %%%%%%%%%%%%%%%%%%%%%%%%%
                                % update states and other parameters of
                                % pedestrian for the first tracklet time step
                                kf = predictionKFtracklet{newTrackletId};
                                resetFlag.sample_goal(newTrackletId)  = true;
                                [tempData, kf, flag, resetFlag] = func_updatePedContStates(kf, trackletData{newTrackletId}, newTrackletId, Params, resetStates, timeStep, flag, resetFlag);
%                                 [tempData, kf] = func_updatePedContStates(kf, trackletData{newTrackletId}, trackletNo, AVStates, cw, Params, resetStates, trackletGoal(newTrackletId,:), timeStep, flag);
                                trackletData{newTrackletId} = tempData; 
                                trackletKfData{newTrackletId}(1, :) = [kf.x', diag(kf.P)']; 
                                predictionKFtracklet{newTrackletId} = kf;
                                % hybrid state reset-the pedestrian has
                                % started walking towards the road to
                                % cross, but is still on the sidewalk, so
                                % considered as 'Approach'
                                trackletData{newTrackletId}.HybridState(end,:) = 'Approach';
                                %%%%%%%%%%%%%%%%%%%%%%%%%
                                % update flags
                                flag.hybridStatePred(newTrackletId) = true;
                                flag.atCrosswalk(newTrackletId) = true;
                                %%%%%%%%%%%%%%%%%%%%%%%%%
                                % update other parameters of pedestrian for the last
                                % time step
                                [tempData2, flag] = func_hybridState_v2(trackletData{newTrackletId}, cw, flag, annotatedImageEnhanced, Params, newTrackletId, resetStates);
                                trackletData{newTrackletId} = tempData2;
                                % Find the ego-car (note that timeStep is incremented by '1' as ego-car for current time step is needed) and update the ped-car states
                                tempData = func_egoCarFunc(trackletData{newTrackletId}, currentTSActiveCarData, carTrackCurrentTimeStep, cw, annotatedImageEnhanced,  Params, timeStep, resetStates);                                       
                                trackletData{newTrackletId} = tempData;
                                %%%%%%%%%%%%%%%%%%%%%%%%%
                            end
                            %%%%%%%%%%%%%%%%%%%%%%%%%
                           % resetStates flags for gap acceptance event
%                            flag.startToCross(trackletNo) = false; % so that whether is reached is not checked again for the parent tracklet
                           % resetStates probabilities
                           trackletData{trackletNo}.probGapAccept = 0;
                    

                    %%%%%%%%%%%%%%%%%%%%%%%%%
                    % 4) if pedestrian finished crossing
                    elseif (flag.finishedCrossingDelayReached(trackletNo) && ~flag.predHorizonEnd(trackletNo))
                            % current tracklet becomes inactive
                            trackletIsActive(trackletNo) = false;
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                            % create a tracklet for turning and approching another
                            % crosswalk at the intersection
                            [trackletData, trackletKfData, trackletProbability, trackletStartNode, trackletEndNode, trackletIsActive, trackletEventFlag, trackletGoal, newTrackletId, flag, resetFlag] = func_newTracklet(trackletData, trackletKfData, trackletProbability, trackletStartNode, trackletEndNode, trackletIsActive, trackletEventFlag, trackletGoal, node_no, 0.5, newTrackletId, trackletNo, flag);
                            parentTracklet(newTrackletId) = trackletNo;
                            trackletData{end}.HybridState(end,:) = 'Approach';
                            trackletGoal(end) = "Approach"; %This tracklet approaches the next crosswalk
                            % update the CW to the one the pedestrian just
                            % crossed
                            trackletData{newTrackletId}.closestCW(end) =  walkawayCW(trackletNo); 
                            trackletEventFlag(newTrackletId) = false;
                            predictionKFtracklet{newTrackletId} = predictionKFtracklet{newTrackletId-1};
                            %%%%%%%%%%%%%%%%%%%%%%%%%
                            % update states and other parameters of
                            % pedestrian for the first tracklet time step
                            kf = predictionKFtracklet{newTrackletId};
                            resetFlag.approachReset(newTrackletId) = true;
                            resetFlag.sample_goal(newTrackletId)  = true;
                            [tempData, kf, flag, resetFlag] = func_updatePedContStates(kf, trackletData{newTrackletId}, newTrackletId, Params, resetStates, timeStep, flag, resetFlag);
                            trackletData{newTrackletId} = tempData; 
                            trackletKfData{newTrackletId}(1, :) = [kf.x', diag(kf.P)']; 
                            predictionKFtracklet{newTrackletId} = kf;
                            % update hybrid state and close CW
                            [tempData2, flag] = func_hybridState_v2(trackletData{newTrackletId}, cw, flag, annotatedImageEnhanced, Params, newTrackletId, resetStates);
                            trackletData{newTrackletId} = tempData2;                            
                            % Find the ego-car (note that timeStep is incremented by '1' as ego-car for current time step is needed) and update the ped-car states
                            tempData = func_egoCarFunc(trackletData{newTrackletId}, currentTSActiveCarData, carTrackCurrentTimeStep, cw, annotatedImageEnhanced,  Params, timeStep, resetStates);                                       
                            trackletData{newTrackletId} = tempData;
                            %%%%%%%%%%%%%%%%%%%%%%%%%
                            % update flag
                            flag.hybridStatePred = true;

                            %%%%%%%%%%%%%%%%%%%%%%%%%
                            % create a tracklet for turning and walking away
                            % from the crosswalk
                            [trackletData, trackletKfData, trackletProbability, trackletStartNode, trackletEndNode, trackletIsActive, trackletEventFlag, trackletGoal, newTrackletId, flag, resetFlag] = func_newTracklet(trackletData, trackletKfData, trackletProbability, trackletStartNode, trackletEndNode, trackletIsActive, trackletEventFlag, trackletGoal, node_no, 0.5, newTrackletId, trackletNo, flag);
                            parentTracklet(newTrackletId) = trackletNo;
                            trackletData{newTrackletId}.HybridState(end,:) = 'Walk_away'; 
                            trackletGoal(newTrackletId,:) = 'Walk_away'; % This tracklet walks away
                            % for walkaway tracklet, update the crosswalk to the crosswalk they
                            % crossed, not the one they are going to approach
                            trackletData{newTrackletId}.closestCW(end) =  walkawayCW(trackletNo); 
                            %%%%%%%%%%%%%%%%%%%%%%%%%
                            % update states and other parameters of
                            % pedestrian for the first tracklet time step
                            predictionKFtracklet{newTrackletId} = predictionKFtracklet{newTrackletId-1};
                            kf = predictionKFtracklet{newTrackletId};
                            resetFlag.sample_goal(newTrackletId)  = true;
                            [tempData, kf, flag, resetFlag] = func_updatePedContStates(kf, trackletData{newTrackletId}, newTrackletId, Params, resetStates, timeStep, flag, resetFlag);
                            trackletData{end} = tempData; 
                            trackletKfData{newTrackletId}(1, :) = [kf.x', diag(kf.P)']; 
                            predictionKFtracklet{newTrackletId} = kf;
                            % update hybrid state and close CW
                            [tempData2, flag] = func_hybridState_v2(trackletData{newTrackletId}, cw, flag, annotatedImageEnhanced, Params, newTrackletId, resetStates);
                            trackletData{newTrackletId} = tempData2;                            
                            % Find the ego-car (note that timeStep is incremented by '1' as ego-car for current time step is needed) and update the ped-car states
                            tempData = func_egoCarFunc(trackletData{newTrackletId}, currentTSActiveCarData, carTrackCurrentTimeStep, cw, annotatedImageEnhanced,  Params, timeStep, resetStates);                                       
                            trackletData{newTrackletId} = tempData;
                            %%%%%%%%%%%%%%%%%%%%%%%%%
                            % update the tracklets flags
                            flag.hybridStatePred(newTrackletId) = true; % it was true anyway
                            trackletEventFlag(newTrackletId) = false;
                            %%%%%%%%%%%%%%%%%%%%%%%%%
                    end
%                 else
%                         %%%%%%%%%%%%%%%%%%%%%%%%%
%                         % no event
%                         trackletEventFlag(trackletNo) = false;  
                end     % end of event check        
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %% update the continuous state of pedestrian if the tracklet is still active
                if trackletIsActive(trackletNo)  
                    % check if a new goal needs to be sampled
                    %% debug
                    if flag.reachGoal(trackletNo)==1
                        x=1;
                    end
                    if flag.reachGoal(trackletNo) && strcmp(trackletData{trackletNo}.HybridState(end),"Walk_away") && (trackletData{trackletNo}.closestCW(end)==1 || trackletData{trackletNo}.closestCW(end)==4)
                        resetFlag.sample_goal(trackletNo) = true;
                        resetFlag.check_goal(trackletNo) = false;
                        [tempData, ~, flag, resetFlag] = func_updatePedContStates(kf, trackletData{trackletNo}, trackletNo, Params, resetStates, timeStep, flag, resetFlag);
                        trackletData{trackletNo}.goalPositionPixels(end,:) = tempData.goalPositionPixels(end,:);
                        % reset flags
                        resetFlag.sample_goal(trackletNo) = false;
                        resetFlag.check_goal(trackletNo) = true;
                    end
        
                    % state updates
                    trackletData{trackletNo}.HybridState(end+1,:) = trackletData{trackletNo}.HybridState(end,:); 
                    flag.hybridStatePred(trackletNo) = false;
                    % continuous state updates
                    kf = predictionKFtracklet{trackletNo};
                    [tempData, kf, flag, resetFlag] = func_updatePedContStates(kf, trackletData{trackletNo}, trackletNo, Params, resetStates, timeStep, flag, resetFlag);
                    trackletData{trackletNo} = tempData; 
                    trackletKfData{trackletNo}(end+1, :) = [kf.x', diag(kf.P)']; 
                    predictionKFtracklet{trackletNo} = kf;
                    %%%%%%%%%%%%%%%%%%%%%%%%%
                    % update other parameters of pedestrian for the last time step
                    [tempData2, flag] = func_hybridState_v2(trackletData{trackletNo}(end,:), cw, flag, annotatedImageEnhanced, Params, trackletNo, resetStates);
                    trackletData{trackletNo}(end,:) = tempData2;
                    % Find the ego-car in the current time step and save the car track index
                    tempData = func_egoCarFunc(trackletData{trackletNo}, currentTSActiveCarData, carTrackCurrentTimeStep, cw, annotatedImageEnhanced,  Params, timeStep, resetStates);                                       
                    trackletData{trackletNo} = tempData;
                    %%%%%%%%%%%%%%%%%%%%%%%%%
                end
        end % end of current prediction tracklet
    end % end of loop for all prediction tracklets  
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     
end  % end of loop for prediction horizon
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% compile predictions
% compile the start and end nodes for each prediction tracklet
N_tracklets = size(trackletData,1);

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
                        predictionTrajectory{N_new}(1,2) = predictionTrajectory{N_new}(1,2)*trackletProbability(trackletId);                          
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
% 
% %% debug
% %% plot predicted vehicle and pedestrian states
% % if (pedPosPixels(1)>=100 && pedPosPixels(1)<=950 && pedPosPixels(2)>=-560 && pedPosPixels(2)<=-100)
%     if ~isempty(currentTSActiveCarData_copy)
%         for ii=1:size(currentTSActiveCarData_copy,1)
%             carPosPixels = int32([currentTSActiveCarData_copy{ii}.xCenter, currentTSActiveCarData_copy{ii}.yCenter]/(orthopxToMeter*scaleFactor));      
%             for zz=1:size(carPosPixels,1)
%                 annotatedImageEnhanced(-carPosPixels(zz,2), carPosPixels(zz,1)) = 150;
%             end
%         end
%     end
%     
%     % predicted trajectory
%     for jj=1:size(trackletData,1)
%         pedPixels = int32([trackletData{jj}.xCenter, trackletData{jj}.yCenter]/(orthopxToMeter*scaleFactor));
%         for zz=1:size(pedPixels,1)
%             if int32(pedPixels(zz,1)) > 0 && int32(pedPixels(zz,2)) < 0
%                 annotatedImageEnhanced(-pedPixels(zz,2), pedPixels(zz,1)) = 90;
%             end
%         end
%     end
% 
% %     % constant velocity
% %     CVPixels = trajectory_CV/(orthopxToMeter*scaleFactor);
% %     for zz=1:size(CVPixels,1)
% %         if int32(CVPixels(zz,1)) > 0 && int32(CVPixels(zz,2)) < 0
% %             annotatedImageEnhanced(int32(-CVPixels(zz,2)), int32(CVPixels(zz,1))) = 125;
% %         end
% %     end
%     
%     % ground truth
%     GTPixels = trajectory_GT/(orthopxToMeter*scaleFactor);
%     for zz=1:size(GTPixels,1)
%         if int32(GTPixels(zz,1)) > 0 && int32(GTPixels(zz,2)) < 0
%             annotatedImageEnhanced(int32(-GTPixels(zz,2)), int32(GTPixels(zz,1))) = 255;  
%         end
%     end
% % end
% imshow(annotatedImageEnhanced);
% pause(0.3);

% % find most likely trajectory
% if ~isempty(trajectory_GT)
%     [~,maxProbTrajIndex] = max(predictionTrajectoryMatrix(:,2));
%     mostLikelyTrajectory = reshape(predictionTrajectoryMatrix(maxProbTrajIndex,3:end),[2,30])';
%     NN = size(trajectory_GT,1);
%     trajError = vecnorm(mostLikelyTrajectory(1:NN,:) - trajectory_GT,2,2);
% 
%     if trajError(1)>0.2 || trajError(int32(NN/2))>3 || trajError(NN)>6
%         x=1;
%     end
%     text = strcat('intial error: ', num2str(trajError(1)), '; Final error: ',num2str(trajError(end)));
% else
%     text = '';
% end
% 
% % add images
% annotatedImageEnhanced_text = insertText(annotatedImageEnhanced,[10, 10], text);
% imshow(annotatedImageEnhanced_text);
% pause(0.3);
% filename = strcat(num2str(posixtime(datetime('now')) * 1e6), '.jpg');
% imwrite(annotatedImageEnhanced_text, filename);
% 
% %     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% remove empty data
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
end % end of the function