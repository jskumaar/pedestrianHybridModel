function [predData, kf] = func_waitReset(predData, kf, trackletNo, Params, cw, reset)

%% 1) setup
% parameters
scaleFactor = Params.scaleFactor;
orthopxToMeter = Params.orthopxToMeter;
del_t = Params.delta_T;
reSampleRate = Params.reSampleRate;
% initialize
pedPosPixels = [predData.xCenter, predData.yCenter]/(orthopxToMeter*scaleFactor);
% calculate angle between pedestrian and approaching crosswalk
pedCwAngle = [ atan2((cw.center_y(1) - pedPosPixels(2)), (cw.center_x(1) - pedPosPixels(1)))*180/pi;
               atan2((cw.center_y(2) - pedPosPixels(2)), (cw.center_x(2) - pedPosPixels(1)))*180/pi;
               atan2((cw.center_y(3) - pedPosPixels(2)), (cw.center_x(3) - pedPosPixels(1)))*180/pi;
               atan2((cw.center_y(4) - pedPosPixels(2)), (cw.center_x(4) - pedPosPixels(1)))*180/pi];          
%%%%%%%%%%%%%%%%%%%%%%%
% struct copy variables to variables (is more faster for downstream processing)
xCenter = predData.xCenter;
yCenter = predData.yCenter;
calcHeading = predData.calcHeading;
xVelocity = predData.xVelocity;
yVelocity = predData.yVelocity;
closestCW = predData.closestCW;
trackLifetime = predData.trackLifetime;
waitTimeSteps = predData.waitTimeSteps;
% pedGoalDisp =  [inf, inf];
lonVelocity = inf;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
% %% 2) tracklet for crossing intent
% % if crosswalk 1
% if closestCW==1
%     if pedCwAngle(1) < 0  % West right lane (Lane 1)
%              calcHeading = reset.wait.heading(1,:);
%              xVelocity = 0;
%              yVelocity = 0;
%              waitTimeSteps = 0;
%          
%     else        % West Left Lane (lane 2)
%              calcHeading = reset.wait.heading(2,:);
%              xVelocity = 0;
%              yVelocity = 0;
%              waitTimeSteps = 0;
%     end
% 
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
% % if crosswalk 2
% elseif closestCW==2
% 
%     if pedCwAngle(2) > 0  % East right lane (Lane 3)
%              calcHeading = reset.wait.heading(3,:);
%              xVelocity = 0;
%              yVelocity = 0;
%              waitTimeSteps = 0;
%     else        % East Left Lane (Lane 4)
%              calcHeading = reset.wait.heading(4,:);
%              xVelocity = 0;
%              yVelocity = 0;
%              waitTimeSteps = 0;
%     end
%     
% 
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % if crosswalk 3
% elseif closestCW==3
%    if abs(pedCwAngle(3)) > 90  % South right lane (lane 5)
%              calcHeading = reset.wait.heading(5,:);
%              xVelocity = 0;
%              yVelocity = 0;
%              waitTimeSteps = 0;
%     else        % South Left Lane (Lane 6)
%              calcHeading = reset.wait.heading(6,:);
%              xVelocity = 0;
%              yVelocity = 0;
%              waitTimeSteps = 0;
%     end
%     
% 
% 
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % if crosswalk 4
% elseif closestCW==4
%    if abs(pedCwAngle(4)) < 90  % North right lane (Lane 7)
%              calcHeading = reset.wait.heading(7,:);
%              xVelocity = 0;
%              yVelocity = 0;
%              waitTimeSteps = 0;
%     else        % North Left Lane (Lane 8)
%              calcHeading = reset.wait.heading(8,:);
%              xVelocity = 0;
%              yVelocity = 0;
%              waitTimeSteps = 0;
%    end
%     
% end
%%%%%%%%%%%%%

check_goal = false;
sample_goal = true;
[~, pedGoalPixels] = check_sample_goal(predData, trackletNo, reset, Params, flag, check_goal, sample_goal);
pedGoalDisp = pedGoalPixels - pedPosPixels;
calcHeading = atan2(pedGoalDisp(2), pedGoalDisp(1)) *180/pi;
xVelocity = 0;
yVelocity = 0;
waitTimeSteps = reSampleRate;
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% update states for 1st time step of new tracklet
xCenter = xCenter + del_t*xVelocity;
yCenter = yCenter + del_t*yVelocity;
trackLifetime = trackLifetime + reSampleRate;


%%%%%%%%%%%%%%%%%%%
% update states (kalman predict noise is included)
% kf predict
kf = kalmanPredict(kf);
updated_X = [xCenter;
             yCenter;
             xVelocity;
             yVelocity];
kf.x = updated_X; 

%%%%%%%%%%%%%%%%%%%%%%%          
%updatedPredData
predData.trackLifetime = trackLifetime;
predData.xCenter = xCenter;
predData.yCenter = yCenter;
predData.xVelocity = xVelocity;
predData.yVelocity = yVelocity;
predData.calcHeading = calcHeading;
% predData.goalDisp = (pedGoalDisp);
predData.goalPositionPixels = pedGoalPixels;
predData.waitTimeSteps = waitTimeSteps;
predData.lonVelocity = lonVelocity;
end