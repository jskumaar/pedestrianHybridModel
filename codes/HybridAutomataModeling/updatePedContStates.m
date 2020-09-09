function [predData, kf] = updatePedContStates(kf, predData, cw, Params, reset, walkawayDirection)

% parameters
scaleFactor = Params.scaleFactor;
orthopxToMeter = Params.orthopxToMeter;
del_t = Params.delta_T;
reSampleRate = Params.reSampleRate;



% initialize
pedPosPixels = double([predData.xCenterPix(end-1), predData.yCenterPix(end-1)]);
predData.recordingId(end) = predData.recordingId(end-1);
predData.trackId(end) = predData.trackId(end-1);


% set default update values for constant velocity model
predData.calcHeading(end) = predData.calcHeading(end-1);
predData.xVelocity(end) = predData.xVelocity(end-1);
predData.yVelocity(end) = predData.yVelocity(end-1);
predData.waitTimeSteps(end) = predData.waitTimeSteps(end-1);


% calculate angle between pedestrian and approaching crosswalk
pedCwAngle = [ atan2(([cw.center_y(1) - pedPosPixels(2)]), ([cw.center_x(1) - pedPosPixels(1)]))*180/pi;
                 atan2(([cw.center_y(2) - pedPosPixels(2)]), ([cw.center_x(2) - pedPosPixels(1)]))*180/pi;
                 atan2(([cw.center_y(3) - pedPosPixels(2)]), ([cw.center_x(3) - pedPosPixels(1)]))*180/pi;
                 atan2(([cw.center_y(4) - pedPosPixels(2)]), ([cw.center_x(4) - pedPosPixels(1)]))*180/pi];
             
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% reset continuous states if discrete state changed for first time
if size(predData,1) > 1

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
% if crosswalk 1
if predData.closestCW(end-1)==1
    if pedCwAngle(1) < 0  % West right lane (Lane 1)
         % 1st Approach
         if ( strcmp(predData.HybridState(end), 'Approach') &&  ~strcmp(predData.HybridState(end-1), 'Approach') )
             pedGoalDisp = reset.approach.goal(1,:) - pedPosPixels;
             predData.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1));
             predData.xVelocity(end) = predData.xVelocity(end-1);
             predData.yVelocity(end) = predData.yVelocity(end-1);
             
         % 1st Wait
         elseif ( strcmp(predData.HybridState(end), 'Wait') &&  strcmp(predData.HybridState(end-1), 'Approach') )
             predData.calcHeading(end) = reset.wait.heading(1,:);
             predData.xVelocity(end) = 0;
             predData.yVelocity(end) = 0;
             predData.waitTimeSteps(end) = 0;
         % 1st Cross
         elseif ( strcmp(predData.HybridState(end), 'Cross') &&  (strcmp(predData.HybridState(end-1), 'Approach') || strcmp(predData.HybridState(end-1), 'Wait') ) )
             pedGoalDisp = reset.approach.goal(2,:) - pedPosPixels;
             predData.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1));
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             predData.xVelocity(end) = vel*cosd(predData.calcHeading(end));
             predData.yVelocity(end) = vel*sind(predData.calcHeading(end));                 
         % 1st Walkaway
         elseif ( strcmp(predData.HybridState(end), 'Walkaway') &&  (strcmp(predData.HybridState(end-1), 'Cross') || strcmp(predData.HybridState(end-1), 'Jaywalking') ) )
             if strcmp(walkawayDirection, 'Approach')         
                    predData.calcHeading(end) = reset.approach.heading(5,:);
             else
                    predData.calcHeading(end) = reset.walkaway.heading(2,:);
             end
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             predData.xVelocity(end) = vel*cosd(predData.calcHeading(end));
             predData.yVelocity(end) = vel*sind(predData.calcHeading(end));
         end
    else        % West Left Lane (lane 2)
         % 1st Approach
         if ( strcmp(predData.HybridState(end), 'Approach') &&  ~strcmp(predData.HybridState(end-1), 'Approach') )
             pedGoalDisp = reset.approach.goal(2,:) - pedPosPixels;
             predData.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1));
             predData.xVelocity(end) = predData.xVelocity(end-1);
             predData.yVelocity(end) = predData.yVelocity(end-1);                  
         % 1st Wait
         elseif ( strcmp(predData.HybridState(end), 'Wait') &&  strcmp(predData.HybridState(end-1), 'Approach') )
             predData.calcHeading(end) = reset.wait.heading(2,:);
             predData.xVelocity(end) = 0;
             predData.yVelocity(end) = 0;
             predData.waitTimeSteps(end) = 0;
         % 1st Cross
         elseif ( strcmp(predData.HybridState(end), 'Cross') &&  (strcmp(predData.HybridState(end-1), 'Approach') || strcmp(predData.HybridState(end-1), 'Wait') ) )
             pedGoalDisp = reset.approach.goal(1,:) - pedPosPixels;
             predData.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1));
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             predData.xVelocity(end) = vel*cosd(predData.calcHeading(end));
             predData.yVelocity(end) = vel*sind(predData.calcHeading(end));
         % 1st Walkaway
         elseif ( strcmp(predData.HybridState(end), 'Walkaway') &&  (strcmp(predData.HybridState(end-1), 'Cross') || strcmp(predData.HybridState(end-1), 'Jaywalking') ) )
             if strcmp(walkawayDirection, 'Approach')         
                    predData.calcHeading(end) = reset.approach.heading(8,:);
             else
                    predData.calcHeading(end) = reset.walkaway.heading(1,:);
             end
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             predData.xVelocity(end) = vel*cosd(predData.calcHeading(end));
             predData.yVelocity(end) = vel*sind(predData.calcHeading(end));
         end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
% if crosswalk 2
if predData.closestCW(end-1)==2

    if pedCwAngle(2) > 0  % East right lane (Lane 3)
         % 1st Approach
         if ( strcmp(predData.HybridState(end), 'Approach') &&  ~strcmp(predData.HybridState(end-1), 'Approach') )
             pedGoalDisp = reset.approach.goal(4,:) - pedPosPixels;
             predData.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1));
             predData.xVelocity(end) = predData.xVelocity(end-1);
             predData.yVelocity(end) = predData.yVelocity(end-1);
         % 1st Wait
         elseif ( strcmp(predData.HybridState(end), 'Wait') &&  strcmp(predData.HybridState(end-1), 'Approach') )
             predData.calcHeading(end) = reset.wait.heading(3,:);
             predData.xVelocity(end) = 0;
             predData.yVelocity(end) = 0;
             predData.waitTimeSteps(end) = 0;
         % 1st Cross
         elseif ( strcmp(predData.HybridState(end), 'Cross') &&  (strcmp(predData.HybridState(end-1), 'Approach') || strcmp(predData.HybridState(end-1), 'Wait') ) )
             pedGoalDisp = reset.approach.goal(4,:) - pedPosPixels;
             predData.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1));
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             predData.xVelocity(end) = vel*cosd(predData.calcHeading(end));
             predData.yVelocity(end) = vel*sind(predData.calcHeading(end));
         % 1st Walkaway
         elseif ( strcmp(predData.HybridState(end), 'Walkaway') &&  (strcmp(predData.HybridState(end-1), 'Cross') || strcmp(predData.HybridState(end-1), 'Jaywalking') ) )
             if strcmp(walkawayDirection, 'Approach')         
                    predData.calcHeading(end) = reset.approach.heading(7,:);
             else
                    predData.calcHeading(end) = reset.walkaway.heading(4,:);
             end
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             predData.xVelocity(end) = vel*cosd(predData.calcHeading(end));
             predData.yVelocity(end) = vel*sind(predData.calcHeading(end));
         end
    else        % East Left Lane (Lane 4)
         % 1st Approach
         if ( strcmp(predData.HybridState(end), 'Approach') &&  ~strcmp(predData.HybridState(end-1), 'Approach') )
             pedGoalDisp = reset.approach.goal(4,:) - pedPosPixels;
             predData.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1));
             predData.xVelocity(end) = predData.xVelocity(end-1);
             predData.yVelocity(end) = predData.yVelocity(end-1);
         % 1st Wait
         elseif ( strcmp(predData.HybridState(end), 'Wait') &&  strcmp(predData.HybridState(end-1), 'Approach') )
             predData.calcHeading(end) = reset.wait.heading(4,:);
             predData.xVelocity(end) = 0;
             predData.yVelocity(end) = 0;
             predData.waitTimeSteps(end) = 0;
         % 1st Cross
         elseif ( strcmp(predData.HybridState(end), 'Cross') &&  (strcmp(predData.HybridState(end-1), 'Approach') || strcmp(predData.HybridState(end-1), 'Wait') ) )
             pedGoalDisp = reset.approach.goal(3,:) - pedPosPixels;
             predData.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1));
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             predData.xVelocity(end) = vel*cosd(predData.calcHeading(end));
             predData.yVelocity(end) = vel*sind(predData.calcHeading(end));
         % 1st Walkaway
         elseif ( strcmp(predData.HybridState(end), 'Walkaway') &&  (strcmp(predData.HybridState(end-1), 'Cross') || strcmp(predData.HybridState(end-1), 'Jaywalking') ) )
             if strcmp(walkawayDirection, 'Approach')         
                    predData.calcHeading(end) = reset.approach.heading(6,:);
             else
                    predData.calcHeading(end) = reset.walkaway.heading(3,:);
             end
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             predData.xVelocity(end) = vel*cosd(predData.calcHeading(end));
             predData.yVelocity(end) = vel*sind(predData.calcHeading(end));
         end
    end
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% if crosswalk 3
if predData.closestCW(end-1)==3
   if abs(pedCwAngle(3)) > 90  % South right lane (lane 5)
         % 1st Approach
         if ( strcmp(predData.HybridState(end), 'Approach') &&  ~strcmp(predData.HybridState(end-1), 'Approach') )
             pedGoalDisp = reset.approach.goal(5,:) - pedPosPixels;
             predData.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1));
             predData.xVelocity(end) = predData.xVelocity(end-1);
             predData.yVelocity(end) = predData.yVelocity(end-1);
         % 1st Wait
         elseif ( strcmp(predData.HybridState(end), 'Wait') &&  strcmp(predData.HybridState(end-1), 'Approach') )
             predData.calcHeading(end) = reset.wait.heading(5,:);
             predData.xVelocity(end) = 0;
             predData.yVelocity(end) = 0;
             predData.waitTimeSteps(end) = 0;
         % 1st Cross
         elseif ( strcmp(predData.HybridState(end), 'Cross') &&  (strcmp(predData.HybridState(end-1), 'Approach') || strcmp(predData.HybridState(end-1), 'Wait') ) )
             pedGoalDisp = reset.approach.goal(6,:) - pedPosPixels;
             predData.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1));
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             predData.xVelocity(end) = vel*cosd(predData.calcHeading(end));
             predData.yVelocity(end) = vel*sind(predData.calcHeading(end));
         % 1st Walkaway
         elseif ( strcmp(predData.HybridState(end), 'Walkaway') &&  (strcmp(predData.HybridState(end-1), 'Cross') || strcmp(predData.HybridState(end-1), 'Jaywalking') ) )
             if strcmp(walkawayDirection, 'Approach')         
                    predData.calcHeading(end) = reset.approach.heading(3,:);
             else
                    predData.calcHeading(end) = reset.walkaway.heading(6,:);
             end
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             predData.xVelocity(end) = vel*cosd(predData.calcHeading(end));
             predData.yVelocity(end) = vel*sind(predData.calcHeading(end));
         end
    else        % South Left Lane (Lane 6)
         % 1st Approach
         if ( strcmp(predData.HybridState(end), 'Approach') &&  ~strcmp(predData.HybridState(end-1), 'Approach') )
             pedGoalDisp = reset.approach.goal(6,:) - pedPosPixels;
             predData.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1));
             predData.xVelocity(end) = predData.xVelocity(end-1);
             predData.yVelocity(end) = predData.yVelocity(end-1);
         % 1st Wait
         elseif ( strcmp(predData.HybridState(end), 'Wait') &&  strcmp(predData.HybridState(end-1), 'Approach') )
             predData.calcHeading(end) = reset.wait.heading(6,:);
             predData.xVelocity(end) = 0;
             predData.yVelocity(end) = 0;
             predData.waitTimeSteps(end) = 0;
         % 1st Cross
         elseif ( strcmp(predData.HybridState(end), 'Cross') &&  (strcmp(predData.HybridState(end-1), 'Approach') || strcmp(predData.HybridState(end-1), 'Wait') ) )
             pedGoalDisp = reset.approach.goal(5,:) - pedPosPixels;
             predData.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1));
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             predData.xVelocity(end) = vel*cosd(predData.calcHeading(end));
             predData.yVelocity(end) = vel*sind(predData.calcHeading(end));
         % 1st Walkaway
         elseif ( strcmp(predData.HybridState(end), 'Walkaway') &&  (strcmp(predData.HybridState(end-1), 'Cross') || strcmp(predData.HybridState(end-1), 'Jaywalking') ) )
             if strcmp(walkawayDirection, 'Approach')         
                    predData.calcHeading(end) = reset.approach.heading(2,:);
             else
                    predData.calcHeading(end) = reset.walkaway.heading(5,:);
             end
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             predData.xVelocity(end) = vel*cosd(predData.calcHeading(end));
             predData.yVelocity(end) = vel*sind(predData.calcHeading(end));
         end
    end
    
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% if crosswalk 4
if predData.closestCW(end-1)==4
   if abs(pedCwAngle(4)) < 90  % North right lane (Lane 7)
         % 1st Approach
         if ( strcmp(predData.HybridState(end), 'Approach') &&  ~strcmp(predData.HybridState(end-1), 'Approach') )
             pedGoalDisp = reset.approach.goal(7,:) - pedPosPixels;
             predData.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1));
             predData.xVelocity(end) = predData.xVelocity(end-1);
             predData.yVelocity(end) = predData.yVelocity(end-1);
         % 1st Wait
         elseif ( strcmp(predData.HybridState(end), 'Wait') &&  strcmp(predData.HybridState(end-1), 'Approach') )
             predData.calcHeading(end) = reset.wait.heading(7,:);
             predData.xVelocity(end) = 0;
             predData.yVelocity(end) = 0;
             predData.waitTimeSteps(end) = 0;
         % 1st Cross
         elseif ( strcmp(predData.HybridState(end), 'Cross') &&  (strcmp(predData.HybridState(end-1), 'Approach') || strcmp(predData.HybridState(end-1), 'Wait') ) )
             pedGoalDisp = reset.approach.goal(8,:) - pedPosPixels;
             predData.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1));
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             predData.xVelocity(end) = vel*cosd(predData.calcHeading(end));
             predData.yVelocity(end) = vel*sind(predData.calcHeading(end));
         % 1st Walkaway
         elseif ( strcmp(predData.HybridState(end), 'Walkaway') &&  (strcmp(predData.HybridState(end-1), 'Cross') || strcmp(predData.HybridState(end-1), 'Jaywalking') ) )
             if strcmp(walkawayDirection, 'Approach')         
                    predData.calcHeading(end) = reset.approach.heading(1,:);
             else
                    predData.calcHeading(end) = reset.walkaway.heading(8,:);
             end
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             predData.xVelocity(end) = vel*cosd(predData.calcHeading(end));
             predData.yVelocity(end) = vel*sind(predData.calcHeading(end));
         end
    else        % North Left Lane (Lane 8)
         % 1st Approach
         if ( strcmp(predData.HybridState(end), 'Approach') &&  ~strcmp(predData.HybridState(end-1), 'Approach') )
             pedGoalDisp = reset.approach.goal(8,:) - pedPosPixels;
             predData.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1));
             predData.xVelocity(end) = predData.xVelocity(end-1);
             predData.yVelocity(end) = predData.yVelocity(end-1);
         % 1st Wait
         elseif ( strcmp(predData.HybridState(end), 'Wait') &&  strcmp(predData.HybridState(end-1), 'Approach') )
             predData.calcHeading(end) = reset.wait.heading(8,:);
             predData.xVelocity(end) = 0;
             predData.yVelocity(end) = 0;
             predData.waitTimeSteps(end) = 0;
         % 1st Cross
         elseif ( strcmp(predData.HybridState(end), 'Cross') &&  (strcmp(predData.HybridState(end-1), 'Approach') || strcmp(predData.HybridState(end-1), 'Wait') ) )
             pedGoalDisp = reset.approach.goal(7,:) - pedPosPixels;
             predData.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1));
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             predData.xVelocity(end) = vel*cosd(predData.calcHeading(end));
             predData.yVelocity(end) = vel*sind(predData.calcHeading(end));
         % 1st Walkaway
         elseif ( strcmp(predData.HybridState(end), 'Walkaway') &&  (strcmp(predData.HybridState(end-1), 'Cross') || strcmp(predData.HybridState(end-1), 'Jaywalking') ) )
             if strcmp(walkawayDirection, 'Approach')         
                    predData.calcHeading(end) = reset.approach.heading(4,:);
             else
                    predData.calcHeading(end) = reset.walkaway.heading(7,:);
             end
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             predData.xVelocity(end) = vel*cosd(predData.calcHeading(end));
             predData.yVelocity(end) = vel*sind(predData.calcHeading(end));
         end
   end
    
end


end  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% update states (currently no noise is directly included)
predData.xCenterPix(end) = predData.xCenterPix(end-1) + del_t*predData.xVelocity(end)/(orthopxToMeter*scaleFactor);
predData.yCenterPix(end) = predData.yCenterPix(end-1) + del_t*predData.yVelocity(end)/(orthopxToMeter*scaleFactor);
predData.xCenter(end) = predData.xCenter(end-1) + del_t*predData.xVelocity(end);
predData.yCenter(end) = predData.yCenter(end-1) + del_t*predData.yVelocity(end);

predData.trackLifetime(end) = predData.trackLifetime(end-1) + reSampleRate;

% update states (kalman predict noise is included)
% kf predict
kf = kalmanPredict(kf);
updated_X = [predData.xCenter(end);
             predData.yCenter(end);
             predData.xVelocity(end);
             predData.yVelocity(end)];
kf.x = updated_X;   % in case there is a discrete transition triggering state reset, the states get updated based on the rest while the covariance remains the same as if it were a constant velocity propagation            
                  

end