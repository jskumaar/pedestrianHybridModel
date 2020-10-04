function [predData, kf] = updatePedContStates(kf, predData, AVStates, cw, Params, reset, walkawayDirection, predTimeStep)

%% setup
% parameters
scaleFactor = Params.scaleFactor;
orthopxToMeter = Params.orthopxToMeter;
del_t = Params.delta_T;
reSampleRate = Params.reSampleRate;
resetState = true;

%%%%%%%%%%%%%%%%%%%%%%%
% initialize default update values for constant velocity model
N = size(predData.xVelocity,1);
N_hybrid = size(predData.HybridState,1);


if size(predData.HybridState,1)~=1
%     if size(predData.HybridState,1)==size(predData.xVelocity,1)
%         predData.HybridState(end+1,1) = predData.HybridState(end);
%     end
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
end

%%%%%%%%%%%%%%%%%%%%%%%
% struct copy variables to variables (is more faster for downstream processing)
xCenter = predData.xCenter;
yCenter = predData.yCenter;
pedPosPixels = [xCenter(end),  yCenter(end)]/(orthopxToMeter*scaleFactor);
calcHeading = predData.calcHeading;
xVelocity = predData.xVelocity;
yVelocity = predData.yVelocity;
closeCar_ind = predData.closeCar_ind;
waitTimeSteps = predData.waitTimeSteps;
HybridState = predData.HybridState;
closestCW = predData.closestCW;
trackLifetime = predData.trackLifetime;
pedGoalDispPixels =  [inf, inf];
%%%%%%%%%%%%%%%%%%%%%%%
% calculate angle between pedestrian and approaching crosswalk
pedCwAngle = [ atan2((cw.center_y(1) - pedPosPixels(2)), (cw.center_x(1) - pedPosPixels(1)))*180/pi;
               atan2((cw.center_y(2) - pedPosPixels(2)), (cw.center_x(2) - pedPosPixels(1)))*180/pi;
               atan2((cw.center_y(3) - pedPosPixels(2)), (cw.center_x(3) - pedPosPixels(1)))*180/pi;
               atan2((cw.center_y(4) - pedPosPixels(2)), (cw.center_x(4) - pedPosPixels(1)))*180/pi];             
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% reset continuous states if discrete state changed for first time
% if (size(HybridState,1) > 1 && HybridState(end)~="" )

% if (size(HybridState,1)~=1 && predTimeStep ~=1 && resetState) % goals have to be reset in the first time step
% %%%%%%%%%%%%%%%%%%%%%%%   
% % if crosswalk 1
% if closestCW(end-1)==1
%     if pedCwAngle(1) < 0  % East right lane (Lane 1)
%          % Approach
% %          if ( strcmp(HybridState(end), 'Approach') &&  ~strcmp(HybridState(end-1), 'Approach') )
%          if  strcmp(HybridState(end), 'Approach') 
%              pedGoalDispPixels = reset.approach.goal(1,:) - pedPosPixels;
%              calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              vel = norm( [xVelocity(end-1), predData.yVelocity(end-1)]);
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end));   
%              resetState = false;
%          % 1st Wait
%          elseif ( strcmp(HybridState(end), 'Wait') &&  strcmp(HybridState(end-1), 'Approach') )
%              pedGoalDispPixels = reset.approach.goal(2,:) - pedPosPixels;
%              calcHeading(end) = reset.wait.heading(1,:);
%              xVelocity(end) = 0;
%              yVelocity(end) = 0;
%              waitTimeSteps(end) = 0;
%              resetState = false;
%          % 1st Cross
%          elseif ( strcmp(HybridState(end), 'Cross') &&  (strcmp(HybridState(end-1), 'Wait') || strcmp(HybridState(end-1), 'Wait') ) )
%              pedGoalDispPixels = reset.approach.goal(2,:) - pedPosPixels;
%              calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              %sample longitudinal velocity in the range of 1 - 2 m/s
%              vel = rand(1) + 1;
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end));  
%              resetState = false;
%          % 1st Walkaway
%          elseif ( strcmp(HybridState(end), 'Walkaway') &&  (strcmp(HybridState(end-1), 'Cross') || strcmp(HybridState(end-1), 'Jaywalking') ) )
%              if strcmp(walkawayDirection, 'Approach')         
%                     pedGoalDispPixels = reset.approach.goal(8,:) - pedPosPixels;
%                     calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              else
%                     pedGoalDispPixels = reset.walkaway.goal(1,:) - pedPosPixels;
%                     calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              end
%              vel = norm( [xVelocity(end-1), yVelocity(end-1)]);
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end)); 
%              resetState = false;
%          end
%     else        % East Left Lane (lane 2)
%          % Approach
% %          if ( strcmp(HybridState(end), 'Approach') &&  ~strcmp(HybridState(end-1), 'Approach') )
%          if  strcmp(HybridState(end), 'Approach') 
%              pedGoalDispPixels = reset.approach.goal(2,:) - pedPosPixels;
%              calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              vel = norm( [xVelocity(end-1), yVelocity(end-1)]);
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end));  
%              resetState = false;
%          % 1st Wait
%          elseif ( strcmp(HybridState(end), 'Wait') &&  strcmp(HybridState(end-1), 'Approach') )
%              pedGoalDispPixels = reset.approach.goal(1,:) - pedPosPixels;
%              calcHeading(end) = reset.wait.heading(2,:);
%              xVelocity(end) = 0;
%              yVelocity(end) = 0;
%              waitTimeSteps(end) = 0;
%              resetState = false;
%          % 1st Cross
%          elseif ( strcmp(HybridState(end), 'Cross') &&  (strcmp(HybridState(end-1), 'Wait') || strcmp(HybridState(end-1), 'Wait') ) )
%              pedGoalDispPixels = reset.approach.goal(1,:) - pedPosPixels;
%              calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              %sample longitudinal velocity in the range of 1 - 2 m/s
%              vel = rand(1) + 1;
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end));
%              resetState = false;
%          % 1st Walkaway
%          elseif ( strcmp(HybridState(end), 'Walkaway') &&  (strcmp(HybridState(end-1), 'Cross') || strcmp(HybridState(end-1), 'Jaywalking') ) )
%              if strcmp(walkawayDirection, 'Approach')         
%                    pedGoalDispPixels = reset.approach.goal(5,:) - pedPosPixels;
%                    calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              else
%                    pedGoalDispPixels = reset.walkaway.goal(2,:) - pedPosPixels;
%                    calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              end
%              vel = norm( [xVelocity(end-1), yVelocity(end-1)]);
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end)); 
%              resetState = false;
%          end
%     end
% 
% %%%%%%%%%%%%%%%%%%%%%%%   
% % if crosswalk 2
% elseif closestCW(end-1)==2
% 
%     if pedCwAngle(2) > 0  % West right lane (Lane 3)
%          % Approach
% %          if ( strcmp(HybridState(end), 'Approach') &&  ~strcmp(HybridState(end-1), 'Approach') )
%          if  strcmp(HybridState(end), 'Approach') 
%              pedGoalDispPixels = reset.approach.goal(3,:) - pedPosPixels;
%              calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              vel = norm( [xVelocity(end-1), yVelocity(end-1)]);
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end)); 
%              resetState = false;
%          % 1st Wait
%          elseif ( strcmp(HybridState(end), 'Wait') &&  strcmp(HybridState(end-1), 'Approach') )
%              pedGoalDispPixels = reset.approach.goal(4,:) - pedPosPixels;
%              calcHeading(end) = reset.wait.heading(3,:);
%              xVelocity(end) = 0;
%              yVelocity(end) = 0;
%              waitTimeSteps(end) = 0;
%              resetState = false;
%          % 1st Cross
%          elseif ( strcmp(HybridState(end), 'Cross') &&  (strcmp(HybridState(end-1), 'Wait') || strcmp(HybridState(end-1), 'Wait') ) )
%              pedGoalDispPixels = reset.approach.goal(4,:) - pedPosPixels;
%              calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              %sample longitudinal velocity in the range of 1 - 2 m/s
%              vel = rand(1) + 1;
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end));
%              resetState = false;
%          % 1st Walkaway
%          elseif ( strcmp(HybridState(end), 'Walkaway') &&  (strcmp(HybridState(end-1), 'Cross') || strcmp(HybridState(end-1), 'Jaywalking') ) )
%              if strcmp(walkawayDirection, 'Approach')         
%                     pedGoalDispPixels = reset.approach.goal(6,:) - pedPosPixels;
%                     calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              else
%                     pedGoalDispPixels = reset.walkaway.goal(3,:) - pedPosPixels;
%                     calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              end
%              vel = norm( [xVelocity(end-1), yVelocity(end-1)]);
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end));
%              resetState = false;
%          end
%     else        % West Left Lane (Lane 4)
%          % Approach
%          %if ( strcmp(HybridState(end), 'Approach') &&  ~strcmp(HybridState(end-1), 'Approach') )
%          if  strcmp(HybridState(end), 'Approach') 
%              pedGoalDispPixels = reset.approach.goal(4,:) - pedPosPixels;
%              calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              vel = norm( [xVelocity(end-1), yVelocity(end-1)]);
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end)); 
%              resetState = false;
%          % 1st Wait
%          elseif ( strcmp(HybridState(end), 'Wait') &&  strcmp(HybridState(end-1), 'Approach') )
%              pedGoalDispPixels = reset.approach.goal(3,:) - pedPosPixels;
%              calcHeading(end) = reset.wait.heading(4,:);
%              xVelocity(end) = 0;
%              yVelocity(end) = 0;
%              waitTimeSteps(end) = 0;
%              resetState = false;
%          % 1st Cross
%          elseif ( strcmp(HybridState(end), 'Cross') &&  (strcmp(HybridState(end-1), 'Wait') || strcmp(HybridState(end-1), 'Wait') ) )
%              pedGoalDispPixels = reset.approach.goal(3,:) - pedPosPixels;
%              calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              %sample longitudinal velocity in the range of 1 - 2 m/s
%              vel = rand(1) + 1;
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end));
%              resetState = false;
%          % 1st Walkaway
%          elseif ( strcmp(HybridState(end), 'Walkaway') &&  (strcmp(HybridState(end-1), 'Cross') || strcmp(HybridState(end-1), 'Jaywalking') ) )
%              if strcmp(walkawayDirection, 'Approach')         
%                     pedGoalDispPixels = reset.approach.goal(7,:) - pedPosPixels;
%                     calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              else
%                     pedGoalDispPixels = reset.walkaway.goal(4,:) - pedPosPixels;
%                     calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              end
%              vel = norm( [xVelocity(end-1), yVelocity(end-1)]);
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end)); 
%              resetState = false;
%          end
%     end
%     
% %%%%%%%%%%%%%%%%%%%%%%%
% % if crosswalk 3
% elseif closestCW(end-1)==3
%    if abs(pedCwAngle(3)) > 90  % South right lane (lane 5)
%          % Approach
% %          if ( strcmp(HybridState(end), 'Approach') &&  ~strcmp(HybridState(end-1), 'Approach') )
%          if  strcmp(HybridState(end), 'Approach') 
%              pedGoalDispPixels = reset.approach.goal(5,:) - pedPosPixels;
%              calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              vel = norm( [xVelocity(end-1), yVelocity(end-1)]);
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end)); 
%              resetState = false;
%          % 1st Wait
%          elseif ( strcmp(HybridState(end), 'Wait') &&  strcmp(HybridState(end-1), 'Approach') )
%              pedGoalDispPixels = reset.approach.goal(6,:) - pedPosPixels;
%              calcHeading(end) = reset.wait.heading(5,:);
%              xVelocity(end) = 0;
%              yVelocity(end) = 0;
%              waitTimeSteps(end) = 0;
%              resetState = false;
%          % 1st Cross
%          elseif ( strcmp(HybridState(end), 'Cross') &&  (strcmp(HybridState(end-1), 'Wait') || strcmp(HybridState(end-1), 'Wait') ) )
%              pedGoalDispPixels = reset.approach.goal(6,:) - pedPosPixels;
%              calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              %sample longitudinal velocity in the range of 1 - 2 m/s
%              vel = rand(1) + 1;
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end));
%              resetState = false;
%          % 1st Walkaway
%          elseif ( strcmp(HybridState(end), 'Walkaway') &&  (strcmp(HybridState(end-1), 'Cross') || strcmp(HybridState(end-1), 'Jaywalking') ) )
%              if strcmp(walkawayDirection, 'Approach')         
%                     pedGoalDispPixels = reset.approach.goal(2,:) - pedPosPixels;
%                     calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              else
%                     pedGoalDispPixels = reset.walkaway.goal(5,:) - pedPosPixels;
%                     calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              end
%              vel = norm( [xVelocity(end-1), yVelocity(end-1)]);
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end)); 
%              resetState = false;
%          end
%     else        % South Left Lane (Lane 6)
%          % Approach
% %          if ( strcmp(HybridState(end), 'Approach') &&  ~strcmp(HybridState(end-1), 'Approach') )
%          if  strcmp(HybridState(end), 'Approach') 
%              pedGoalDispPixels = reset.approach.goal(6,:) - pedPosPixels;
%              calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              vel = norm( [xVelocity(end-1), yVelocity(end-1)]);
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end)); 
%              resetState = false;
%          % 1st Wait
%          elseif ( strcmp(HybridState(end), 'Wait') &&  strcmp(HybridState(end-1), 'Approach') )
%              pedGoalDispPixels = reset.approach.goal(5,:) - pedPosPixels;
%              calcHeading(end) = reset.wait.heading(6,:);
%              xVelocity(end) = 0;
%              yVelocity(end) = 0;
%              waitTimeSteps(end) = 0;
%              resetState = false;
%          % 1st Cross
%          elseif ( strcmp(HybridState(end), 'Cross') &&  (strcmp(HybridState(end-1), 'Wait') || strcmp(HybridState(end-1), 'Wait') ) )
%              pedGoalDispPixels = reset.approach.goal(5,:) - pedPosPixels;
%              calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              %sample longitudinal velocity in the range of 1 - 2 m/s
%              vel = rand(1) + 1;
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end));
%              resetState = false;
%          % 1st Walkaway
%          elseif ( strcmp(HybridState(end), 'Walkaway') &&  (strcmp(HybridState(end-1), 'Cross') || strcmp(HybridState(end-1), 'Jaywalking') ) )
%              if strcmp(walkawayDirection, 'Approach')         
%                     pedGoalDispPixels = reset.approach.goal(3,:) - pedPosPixels;
%                     calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              else
%                     pedGoalDispPixels = reset.walkaway.goal(6,:) - pedPosPixels;
%                     calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              end
%              vel = norm( [xVelocity(end-1), yVelocity(end-1)]);
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end));
%              resetState = false;
%          end
%     end
%     
% %%%%%%%%%%%%%%%%%%%%%%%
% % if crosswalk 4
% elseif closestCW(end-1)==4
%    if abs(pedCwAngle(4)) < 90  % North right lane (Lane 7)
%          % Approach
% %          if ( strcmp(HybridState(end), 'Approach') &&  ~strcmp(HybridState(end-1), 'Approach') )
%          if  strcmp(HybridState(end), 'Approach') 
%              pedGoalDispPixels = reset.approach.goal(7,:) - pedPosPixels;
%              calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              vel = norm( [xVelocity(end-1), yVelocity(end-1)]);
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end)); 
%              resetState = false;
%          % 1st Wait
%          elseif ( strcmp(HybridState(end), 'Wait') &&  strcmp(HybridState(end-1), 'Approach') )
%              pedGoalDispPixels = reset.approach.goal(8,:) - pedPosPixels;
%              calcHeading(end) = reset.wait.heading(7,:);
%              xVelocity(end) = 0;
%              yVelocity(end) = 0;
%              waitTimeSteps(end) = 0;
%              resetState = false;
%          % 1st Cross
%          elseif ( strcmp(HybridState(end), 'Cross') &&  (strcmp(HybridState(end-1), 'Wait') || strcmp(HybridState(end-1), 'Wait') ) )
%              pedGoalDispPixels = reset.approach.goal(8,:) - pedPosPixels;
%              calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              %sample longitudinal velocity in the range of 1 - 2 m/s
%              vel = rand(1) + 1;
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end));
%              resetState = false;
%          % 1st Walkaway
%          elseif ( strcmp(HybridState(end), 'Walkaway') &&  (strcmp(HybridState(end-1), 'Cross') || strcmp(HybridState(end-1), 'Jaywalking') ) )
%              if strcmp(walkawayDirection, 'Approach')         
%                     pedGoalDispPixels = reset.approach.goal(4,:) - pedPosPixels;
%                     calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              else
%                     pedGoalDispPixels = reset.walkaway.goal(7,:) - pedPosPixels;
%                     calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              end
%              vel = norm( [xVelocity(end-1), yVelocity(end-1)]);
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end)); 
%              resetState = false;
%          end
%     else        % North Left Lane (Lane 8)
%          % Approach
% %          if ( strcmp(HybridState(end), 'Approach') &&  ~strcmp(HybridState(end-1), 'Approach') )
%          if  strcmp(HybridState(end), 'Approach') 
%              pedGoalDispPixels = reset.approach.goal(8,:) - pedPosPixels;
%              calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              vel = norm( [xVelocity(end-1), yVelocity(end-1)]);
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end)); 
%              resetState = false;
%          % 1st Wait
%          elseif ( strcmp(HybridState(end), 'Wait') &&  strcmp(HybridState(end-1), 'Approach') )
%              pedGoalDispPixels = reset.approach.goal(7,:) - pedPosPixels;
%              calcHeading(end) = reset.wait.heading(8,:);
%              xVelocity(end) = 0;
%              yVelocity(end) = 0;
%              waitTimeSteps(end) = 0;
%              resetState = false;
%          % 1st Cross
%          elseif ( strcmp(HybridState(end), 'Cross') &&  (strcmp(HybridState(end-1), 'Wait') || strcmp(HybridState(end-1), 'Wait') ) )
%              pedGoalDispPixels = reset.approach.goal(7,:) - pedPosPixels;
%              calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              %sample longitudinal velocity in the range of 1 - 2 m/s
%              vel = rand(1) + 1;
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end));
%              resetState = false;
%          % 1st Walkaway
%          elseif ( strcmp(HybridState(end), 'Walkaway') &&  (strcmp(HybridState(end-1), 'Cross') || strcmp(HybridState(end-1), 'Jaywalking') ) )
%              if strcmp(walkawayDirection, 'Approach')         
%                     pedGoalDispPixels = reset.approach.goal(1,:) - pedPosPixels;
%                     calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              else
%                     pedGoalDispPixels = reset.walkaway.goal(8,:) - pedPosPixels;
%                     calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              end
%              vel = norm( [xVelocity(end-1), yVelocity(end-1)]);
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end)); 
%              resetState = false;
%          end
%    end
%     
% end   % end of all four crosswalk conditions
% %%%%%%%%%%%%%%%%%%%%%%%%%
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% if it is a new tracklet (or 1st time step) or no previous reset state
% if (size(HybridState,1)==1 || closeCar_ind(end)~=closeCar_ind(end-1))
% if crosswalk 1
if closestCW(end)==1
    if pedCwAngle(1) < 0  % East right lane (Lane 1)
         % Approach
%          if ( strcmp(HybridState(end), 'Approach') &&  ~strcmp(HybridState(end), 'Approach') )
         if  strcmp(HybridState(end), 'Approach')
             if ( size(HybridState,1)>1 && (closestCW(end)~=closestCW(end-1)) )
                 pedGoalDispPixels = reset.approach.goal(1,:) - pedPosPixels;
                 calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
                 vel = norm( [xVelocity(end), predData.yVelocity(end)]);
                 xVelocity(end) = vel*cosd(calcHeading(end));
                 yVelocity(end) = vel*sind(calcHeading(end)); 
             end
         % 1st Wait
         elseif ( strcmp(HybridState(end), 'Wait'))
             pedGoalDispPixels = reset.approach.goal(2,:) - pedPosPixels;
             calcHeading(end) = reset.wait.heading(1,:);
             xVelocity(end) = 0;
             yVelocity(end) = 0;
             waitTimeSteps(end) = 0;
         % 1st Cross
         elseif ( strcmp(HybridState(end), 'Cross') )
             pedGoalDispPixels = reset.approach.goal(2,:) - pedPosPixels;
             calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             xVelocity(end) = vel*cosd(calcHeading(end));
             yVelocity(end) = vel*sind(calcHeading(end));                 
         % 1st Walkaway
         elseif ( strcmp(HybridState(end), 'Walkaway') )
             if strcmp(walkawayDirection, 'Approach')         
                    pedGoalDispPixels = reset.approach.goal(8,:) - pedPosPixels;
                    calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
             else
                    pedGoalDispPixels = reset.walkaway.goal(1,:) - pedPosPixels;
                    calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
             end
             vel = norm( [xVelocity(end), yVelocity(end)]);
             xVelocity(end) = vel*cosd(calcHeading(end));
             yVelocity(end) = vel*sind(calcHeading(end)); 
         end
    else        % East Left Lane (lane 2)
         % Approach
%          if ( strcmp(HybridState(end), 'Approach') &&  ~strcmp(HybridState(end), 'Approach') )
         if  strcmp(HybridState(end), 'Approach') 
             if ( size(HybridState,1)>1 && (closestCW(end)~=closestCW(end-1)) )
                 pedGoalDispPixels = reset.approach.goal(2,:) - pedPosPixels;
                 calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
                 vel = norm( [xVelocity(end), yVelocity(end)]);
                 xVelocity(end) = vel*cosd(calcHeading(end));
                 yVelocity(end) = vel*sind(calcHeading(end)); 
             end
         % 1st Wait
         elseif ( strcmp(HybridState(end), 'Wait')  )
             calcHeading(end) = reset.wait.heading(2,:);
             pedGoalDispPixels = reset.approach.goal(1,:) - pedPosPixels;
             xVelocity(end) = 0;
             yVelocity(end) = 0;
             waitTimeSteps(end) = 0;
         % 1st Cross
         elseif ( strcmp(HybridState(end), 'Cross')  )
             pedGoalDispPixels = reset.approach.goal(1,:) - pedPosPixels;
             calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             xVelocity(end) = vel*cosd(calcHeading(end));
             yVelocity(end) = vel*sind(calcHeading(end));
         % 1st Walkaway
         elseif ( strcmp(HybridState(end), 'Walkaway')  )
             if strcmp(walkawayDirection, 'Approach')         
                   pedGoalDispPixels = reset.approach.goal(5,:) - pedPosPixels;
                   calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
             else
                   pedGoalDispPixels = reset.walkaway.goal(2,:) - pedPosPixels;
                   calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
             end
             vel = norm( [xVelocity(end), yVelocity(end)]);
             xVelocity(end) = vel*cosd(calcHeading(end));
             yVelocity(end) = vel*sind(calcHeading(end)); 
         end
    end

%%%%%%%%%%%%%%%%%%%%%%%   
% if crosswalk 2
elseif closestCW(end)==2

    if pedCwAngle(2) > 0  % West right lane (Lane 3)
         % Approach
%          if ( strcmp(HybridState(end), 'Approach') &&  ~strcmp(HybridState(end), 'Approach') )
         if  strcmp(HybridState(end), 'Approach') 
             if ( size(HybridState,1)>1 && (closestCW(end)~=closestCW(end-1)) )
                 pedGoalDispPixels = reset.approach.goal(3,:) - pedPosPixels;
                 calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
                 vel = norm( [xVelocity(end), yVelocity(end)]);
                 xVelocity(end) = vel*cosd(calcHeading(end));
                 yVelocity(end) = vel*sind(calcHeading(end)); 
             end
         % 1st Wait
         elseif ( strcmp(HybridState(end), 'Wait') )
             pedGoalDispPixels = reset.approach.goal(4,:) - pedPosPixels;
             calcHeading(end) = reset.wait.heading(3,:);
             xVelocity(end) = 0;
             yVelocity(end) = 0;
             waitTimeSteps(end) = 0;
         % 1st Cross
         elseif ( strcmp(HybridState(end), 'Cross')  )
             pedGoalDispPixels = reset.approach.goal(4,:) - pedPosPixels;
             calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             xVelocity(end) = vel*cosd(calcHeading(end));
             yVelocity(end) = vel*sind(calcHeading(end));
         % 1st Walkaway
         elseif ( strcmp(HybridState(end), 'Walkaway')  )
             if strcmp(walkawayDirection, 'Approach')         
                    pedGoalDispPixels = reset.approach.goal(6,:) - pedPosPixels;
                    calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
             else
                    pedGoalDispPixels = reset.walkaway.goal(3,:) - pedPosPixels;
                    calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
             end
             vel = norm( [xVelocity(end), yVelocity(end)]);
             xVelocity(end) = vel*cosd(calcHeading(end));
             yVelocity(end) = vel*sind(calcHeading(end)); 
         end
    else        % West Left Lane (Lane 4)
         % Approach
         %if ( strcmp(HybridState(end), 'Approach') &&  ~strcmp(HybridState(end), 'Approach') )
         if  strcmp(HybridState(end), 'Approach') 
             if ( size(HybridState,1)>1 && (closestCW(end)~=closestCW(end-1)) )
                 pedGoalDispPixels = reset.approach.goal(4,:) - pedPosPixels;
                 calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
                 vel = norm( [xVelocity(end), yVelocity(end)]);
                 xVelocity(end) = vel*cosd(calcHeading(end));
                 yVelocity(end) = vel*sind(calcHeading(end)); 
             end
         % 1st Wait
         elseif ( strcmp(HybridState(end), 'Wait') )
             pedGoalDispPixels = reset.approach.goal(3,:) - pedPosPixels;
             calcHeading(end) = reset.wait.heading(4,:);
             xVelocity(end) = 0;
             yVelocity(end) = 0;
             waitTimeSteps(end) = 0;
         % 1st Cross
         elseif ( strcmp(HybridState(end), 'Cross')  )
             pedGoalDispPixels = reset.approach.goal(3,:) - pedPosPixels;
             calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             xVelocity(end) = vel*cosd(calcHeading(end));
             yVelocity(end) = vel*sind(calcHeading(end));
         % 1st Walkaway
         elseif ( strcmp(HybridState(end), 'Walkaway')  )
             if strcmp(walkawayDirection, 'Approach')         
                    pedGoalDispPixels = reset.approach.goal(7,:) - pedPosPixels;
                    calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
             else
                    pedGoalDispPixels = reset.walkaway.goal(4,:) - pedPosPixels;
                    calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
             end
             vel = norm( [xVelocity(end), yVelocity(end)]);
             xVelocity(end) = vel*cosd(calcHeading(end));
             yVelocity(end) = vel*sind(calcHeading(end)); 
         end
    end
    
%%%%%%%%%%%%%%%%%%%%%%%
% if crosswalk 3
elseif closestCW(end)==3
   if abs(pedCwAngle(3)) > 90  % South right lane (lane 5)
         % Approach
%          if ( strcmp(HybridState(end), 'Approach') &&  ~strcmp(HybridState(end), 'Approach') )
         if  strcmp(HybridState(end), 'Approach') 
             if ( size(HybridState,1)>1 && (closestCW(end)~=closestCW(end-1)) )
                 pedGoalDispPixels = reset.approach.goal(5,:) - pedPosPixels;
                 calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
                 vel = norm( [xVelocity(end), yVelocity(end)]);
                 xVelocity(end) = vel*cosd(calcHeading(end));
                 yVelocity(end) = vel*sind(calcHeading(end)); 
             end
         % 1st Wait
         elseif ( strcmp(HybridState(end), 'Wait') )
             pedGoalDispPixels = reset.approach.goal(6,:) - pedPosPixels;
             calcHeading(end) = reset.wait.heading(5,:);
             xVelocity(end) = 0;
             yVelocity(end) = 0;
             waitTimeSteps(end) = 0;
         % 1st Cross
         elseif ( strcmp(HybridState(end), 'Cross') )
             pedGoalDispPixels = reset.approach.goal(6,:) - pedPosPixels;
             calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
             %sample longitudinal velocity in the range of 1 - 1.5 m/s
             vel = 0.5*rand(1) + 1;
             xVelocity(end) = vel*cosd(calcHeading(end));
             yVelocity(end) = vel*sind(calcHeading(end));
         % 1st Walkaway
         elseif ( strcmp(HybridState(end), 'Walkaway') )
             if strcmp(walkawayDirection, 'Approach')         
                    pedGoalDispPixels = reset.approach.goal(2,:) - pedPosPixels;
                    calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
             else
                    pedGoalDispPixels = reset.walkaway.goal(5,:) - pedPosPixels;
                    calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
             end
             vel = norm( [xVelocity(end), yVelocity(end)]);
             xVelocity(end) = vel*cosd(calcHeading(end));
             yVelocity(end) = vel*sind(calcHeading(end)); 
         end
    else        % South Left Lane (Lane 6)
         % Approach
%          if ( strcmp(HybridState(end), 'Approach') &&  ~strcmp(HybridState(end), 'Approach') )
         if  strcmp(HybridState(end), 'Approach') 
             if ( size(HybridState,1)>1 && (closestCW(end)~=closestCW(end-1)) )
                 pedGoalDispPixels = reset.approach.goal(6,:) - pedPosPixels;
                 calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
                 vel = norm( [xVelocity(end), yVelocity(end)]);
                 xVelocity(end) = vel*cosd(calcHeading(end));
                 yVelocity(end) = vel*sind(calcHeading(end)); 
             end
         % 1st Wait
         elseif ( strcmp(HybridState(end), 'Wait') )
             pedGoalDispPixels = reset.approach.goal(5,:) - pedPosPixels;
             calcHeading(end) = reset.wait.heading(6,:);
             xVelocity(end) = 0;
             yVelocity(end) = 0;
             waitTimeSteps(end) = 0;
         % 1st Cross
         elseif ( strcmp(HybridState(end), 'Cross') )
             pedGoalDispPixels = reset.approach.goal(5,:) - pedPosPixels;
             calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             xVelocity(end) = vel*cosd(calcHeading(end));
             yVelocity(end) = vel*sind(calcHeading(end));
         % 1st Walkaway
         elseif ( strcmp(HybridState(end), 'Walkaway')  )
             if strcmp(walkawayDirection, 'Approach')         
                    pedGoalDispPixels = reset.approach.goal(3,:) - pedPosPixels;
                    calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
             else
                    pedGoalDispPixels = reset.walkaway.goal(6,:) - pedPosPixels;
                    calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
             end
             vel = norm( [xVelocity(end), yVelocity(end)]);
             xVelocity(end) = vel*cosd(calcHeading(end));
             yVelocity(end) = vel*sind(calcHeading(end)); 
         end
    end
    
%%%%%%%%%%%%%%%%%%%%%%%
% if crosswalk 4
elseif closestCW(end)==4
   if abs(pedCwAngle(4)) < 90  % North right lane (Lane 7)
         % Approach
%          if ( strcmp(HybridState(end), 'Approach') &&  ~strcmp(HybridState(end), 'Approach') )
         if  strcmp(HybridState(end), 'Approach') 
             if ( size(HybridState,1)>1 && (closestCW(end)~=closestCW(end-1)) )
                 pedGoalDispPixels = reset.approach.goal(7,:) - pedPosPixels;
                 calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
                 vel = norm( [xVelocity(end), yVelocity(end)]);
                 xVelocity(end) = vel*cosd(calcHeading(end));
                 yVelocity(end) = vel*sind(calcHeading(end));
             end
         % 1st Wait
         elseif ( strcmp(HybridState(end), 'Wait') )
             pedGoalDispPixels = reset.approach.goal(8,:) - pedPosPixels;
             calcHeading(end) = reset.wait.heading(7,:);
             xVelocity(end) = 0;
             yVelocity(end) = 0;
             waitTimeSteps(end) = 0;
         % 1st Cross
         elseif ( strcmp(HybridState(end), 'Cross') )
             pedGoalDispPixels = reset.approach.goal(8,:) - pedPosPixels;
             calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             xVelocity(end) = vel*cosd(calcHeading(end));
             yVelocity(end) = vel*sind(calcHeading(end));
         % 1st Walkaway
         elseif ( strcmp(HybridState(end), 'Walkaway') )
             if strcmp(walkawayDirection, 'Approach')         
                    pedGoalDispPixels = reset.approach.goal(4,:) - pedPosPixels;
                    calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
             else
                    pedGoalDispPixels = reset.walkaway.goal(7,:) - pedPosPixels;
                    calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
             end
             vel = norm( [xVelocity(end), yVelocity(end)]);
             xVelocity(end) = vel*cosd(calcHeading(end));
             yVelocity(end) = vel*sind(calcHeading(end)); 
         end
    else        % North Left Lane (Lane 8)
         % Approach
%          if ( strcmp(HybridState(end), 'Approach') &&  ~strcmp(HybridState(end), 'Approach') )
         if  strcmp(HybridState(end), 'Approach') 
             if ( size(HybridState,1)>1 && (closestCW(end)~=closestCW(end-1)) )
                 pedGoalDispPixels = reset.approach.goal(8,:) - pedPosPixels;
                 calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
                 vel = norm( [xVelocity(end), yVelocity(end)]);
                 xVelocity(end) = vel*cosd(calcHeading(end));
                 yVelocity(end) = vel*sind(calcHeading(end)); 
             end
         % 1st Wait
         elseif ( strcmp(HybridState(end), 'Wait') )
             pedGoalDispPixels = reset.approach.goal(7,:) - pedPosPixels;
             calcHeading(end) = reset.wait.heading(8,:);
             xVelocity(end) = 0;
             yVelocity(end) = 0;
             waitTimeSteps(end) = 0;
         % 1st Cross
         elseif ( strcmp(HybridState(end), 'Cross') )
             pedGoalDispPixels = reset.approach.goal(7,:) - pedPosPixels;
             calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             xVelocity(end) = vel*cosd(calcHeading(end));
             yVelocity(end) = vel*sind(calcHeading(end));
         % 1st Walkaway
         elseif ( strcmp(HybridState(end), 'Walkaway') )
             if strcmp(walkawayDirection, 'Approach')         
                    pedGoalDispPixels = reset.approach.goal(1,:) - pedPosPixels;
                    calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
             else
                    pedGoalDispPixels = reset.walkaway.goal(8,:) - pedPosPixels;
                    calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
             end
             vel = norm( [xVelocity(end), yVelocity(end)]);
             xVelocity(end) = vel*cosd(calcHeading(end));
             yVelocity(end) = vel*sind(calcHeading(end)); 
         end
   end
    
end   % end of all four crosswalk conditions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% update states (kalman predict noise is included)
% kf predict
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
% predData.HybridState = HybridState;
predData.calcHeading = calcHeading;
predData.waitTimeSteps = waitTimeSteps;
predData.goalDisp(end) = norm(pedGoalDispPixels)*(orthopxToMeter*scaleFactor);

if pedGoalDispPixels(1)==inf
    x=1;
end

end