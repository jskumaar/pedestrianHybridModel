function [predData, kf, flag, resetFlag] = func_updatePedContStates(kf, predData, trackletNo, Params, resetStates, predTimeStep, flag, resetFlag)

%% setup
% parameters
scaleFactor = Params.scaleFactor;
orthopxToMeter = Params.orthopxToMeter;
del_t = Params.delta_T;
reSampleRate = Params.reSampleRate;

%%%%%%%%%%%%%%%%%%%%%%%
% initialize default update values for constant velocity model
if size(predData.HybridState,1)~=1
    predData.trackLifetime(end+1,1) = predData.trackLifetime(end);
    predData.xCenter(end+1,1) = predData.xCenter(end); 
    predData.yCenter(end+1,1) = predData.yCenter(end);
    predData.xVelocity(end+1,1) = predData.xVelocity(end);
    predData.yVelocity(end+1,1) = predData.yVelocity(end);
    predData.closestCW(end+1,1) = predData.closestCW(end);
    predData.calcHeading(end+1,1) = predData.calcHeading(end);
%     predData.isLooking(end+1,1) = predData.isLooking(end); % comment it out when gaze is not updated within the prediction horizon
    predData.isPedSameDirection(end+1,1) = predData.isPedSameDirection(end);
    predData.waitTimeSteps(end+1,1) = predData.waitTimeSteps(end);
    predData.longDispPedCw(end+1,1) = predData.longDispPedCw(end);
    predData.latDispPedCw(end+1,1) = predData.latDispPedCw(end);
    predData.isNearLane(end+1,1) = predData.isNearLane(end);
    predData.goalDisp(end+1,1) = predData.goalDisp(end);
    predData.closeCar_ind(end+1,1) = predData.closeCar_ind(end);
    predData.activeCar_ind(end+1,1) = predData.activeCar_ind(end);
    predData.lonVelocity(end+1,1) = predData.lonVelocity(end);
    predData.long_disp_ped_car(end+1,1) = predData.long_disp_ped_car(end);
    predData.Lane(end+1,:) = predData.Lane(end,:);
    predData.swInd(end+1,:) = predData.swInd(end,:);
    predData.goalPositionPixels(end+1,:) = predData.goalPositionPixels(end,:);
end

%%%%%%%%%%%%%%%%%%%%%%%
% struct copy variables to variables (is more faster for downstream processing)
xCenter = predData.xCenter;
yCenter = predData.yCenter;
pedPosPixels = [xCenter(end),  yCenter(end)]/(orthopxToMeter*scaleFactor);
calcHeading = predData.calcHeading;
xVelocity = predData.xVelocity;
yVelocity = predData.yVelocity;
% closeCar_ind = predData.closeCar_ind;
HybridState = predData.HybridState;
closestCW = predData.closestCW;
trackLifetime = predData.trackLifetime;
pedGoalPixels = predData.goalPositionPixels(end,:);
pedGoalDispPixels = [inf, inf];

% sample new goal location 
if ( (size(HybridState,1)>1 && closestCW(end)~=closestCW(end-1)) || resetFlag.sample_goal(trackletNo) )  
    [~, pedGoalPixels, resetFlag] = func_checkSampleGoal(predData, trackletNo, resetStates, Params, flag, resetFlag);
    % reset reach goal flag
    flag.reachGoal(trackletNo) = false;
end

% update heading and velocity when there is a valid goal location
if (pedGoalPixels(1)~=0 && pedGoalPixels(1)~=inf) && ( pedGoalPixels(2)~=0 && pedGoalPixels(2)~=inf)
% when there is no close CW and when the previous goal is not origin (default value), retain the previous goal location
    pedGoalDispPixels = pedGoalPixels - pedPosPixels;
    calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
    vel = norm( [xVelocity(end), yVelocity(end)]);
    % if pedestrian is starting to cross from wait, then sample a velocity
    if (strcmp(HybridState(end),'Crossing')||strcmp(HybridState(end),'Jaywalking')) && ((size(HybridState,1)>1 && strcmp(HybridState(end-1),'Wait')) || vel < 0.2)
        vel = rand + 1; % choose a velocity between 1 - 2 m/s
    elseif strcmp(HybridState(end),'Wait')
        vel = 0; % reset velocity to zero when waiting
    end    
    xVelocity(end) = vel*cosd(calcHeading(end));
    yVelocity(end) = vel*sind(calcHeading(end)); 
end   % end of all four crosswalk conditions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
% update states for 1st time step of new tracklet
xCenter(end) = xCenter(end) + del_t*xVelocity(end);
yCenter(end) = yCenter(end) + del_t*yVelocity(end);
trackLifetime(end) = trackLifetime(end) + reSampleRate;
% KF states
kf = kalmanPredict(kf);
updated_X = [xCenter(end);
             yCenter(end);
             xVelocity(end);
             yVelocity(end)];
kf.x = updated_X;   % in case there is a discrete transition triggering state reset, the states get updated based on the rest while the covariance remains the same as if it were a constant velocity propagation            
%%%%%%%%%%%%%%%%%%%%%%%          

%updatedPredData
predData.trackLifetime = trackLifetime;
predData.xCenter = xCenter;
predData.yCenter = yCenter;
predData.xVelocity = xVelocity;
predData.yVelocity = yVelocity;
predData.calcHeading = calcHeading;
predData.goalDisp(end) = norm(pedGoalDispPixels)*(orthopxToMeter*scaleFactor);
predData.goalPositionPixels(end, :) = pedGoalPixels;

% check if pedestrian has reached goal location
resetFlag.check_goal(trackletNo) = true;
resetFlag.sample_goal(trackletNo) = false;
[flag,~,~] = func_checkSampleGoal(predData, trackletNo, resetStates, Params, flag, resetFlag);
% if flag.reachGoal(trackletNo) && strcmp(predData.HybridState(end),'Approach')
%     flag.reachCrosswalk(trackletNo) = true;
% end

%% debug
if flag.reachGoal(trackletNo)
    x=1;
end

end