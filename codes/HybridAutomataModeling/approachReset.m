function [predData, kf] = approachReset(predData, kf, Params, cw, reset, flag_reachCrosswalk)

%% 1) setup
% parameters
scaleFactor = Params.scaleFactor;
orthopxToMeter = Params.orthopxToMeter;
del_t = Params.delta_T;
reSampleRate = Params.reSampleRate;
% initialize
N = length(predData.trackLifetime);
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
pedPosPixels = [xCenter,  yCenter]/(orthopxToMeter*scaleFactor);
calcHeading = predData.calcHeading;
xVelocity = predData.xVelocity;
yVelocity = predData.yVelocity;
closestCW = predData.closestCW;
Lane = predData.Lane;
trackLifetime = predData.trackLifetime;
pedGoalDisp =  [inf, inf];
lonVelocity = inf;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % sample a goal location
% check_goal = false;
% sample_goal = true;
% [~, pedGoalPixels] = check_sample_goal(predData, trackletNo, resetStates, Params, flag, check_goal, sample_goal);


%% 2) tracklet for crossing intent   
% if crosswalk 1
if closestCW==1
    if pedCwAngle(1) < 0  % East right lane (Lane 1)
        % approaching from left
        if abs(calcHeading) < 90 
             pedGoalPixels = reset.walkaway.goal(1,1:2);
             pedGoalDisp = pedGoalPixels - pedPosPixels;
             calcHeading = atan2(pedGoalDisp(2), pedGoalDisp(1)) *180/pi;
             vel = norm( [xVelocity, yVelocity]);
             xVelocity = vel*cosd(calcHeading);
             yVelocity = vel*sind(calcHeading); 
        else
             pedGoalPixels = reset.approach.goal(8,1:2);
             pedGoalDisp = reset.approach.goal(8,1:2) - pedPosPixels;
             calcHeading = atan2(pedGoalDisp(2), pedGoalDisp(1)) *180/pi;
             vel = norm( [xVelocity, yVelocity]);
             xVelocity = vel*cosd(calcHeading);
             yVelocity = vel*sind(calcHeading); 
        end
         
    else        % EAst Left Lane (lane 2)
        if abs(calcHeading) < 90 
             pedGoalPixels = reset.walkaway.goal(2,1:2);
             pedGoalDisp = reset.walkaway.goal(2,1:2) - pedPosPixels;
             calcHeading = atan2(pedGoalDisp(2), pedGoalDisp(1)) *180/pi;
             vel = norm( [xVelocity, yVelocity]);
             xVelocity = vel*cosd(calcHeading);
             yVelocity = vel*sind(calcHeading); 
        else
             pedGoalPixels = reset.approach.goal(5,1:2);
             pedGoalDisp = reset.approach.goal(5,1:2) - pedPosPixels;
             calcHeading = atan2(pedGoalDisp(2), pedGoalDisp(1)) *180/pi;
             vel = norm( [xVelocity, yVelocity]);
             xVelocity = vel*cosd(calcHeading);
             yVelocity = vel*sind(calcHeading); 
        end
    end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
% if crosswalk 2
elseif closestCW==2

    if pedCwAngle(2) > 0  % West right lane (Lane 3)
        if abs(calcHeading) > 90 
             pedGoalPixels = reset.walkaway.goal(3,1:2);
             pedGoalDisp = reset.walkaway.goal(3,1:2) - pedPosPixels;
             calcHeading = atan2(pedGoalDisp(2), pedGoalDisp(1)) *180/pi;
             vel = norm( [xVelocity, yVelocity]);
             xVelocity = vel*cosd(calcHeading);
             yVelocity = vel*sind(calcHeading);             
        else
             pedGoalPixels = reset.approach.goal(6,1:2);
             pedGoalDisp = reset.approach.goal(6,1:2) - pedPosPixels;
             calcHeading = atan2(pedGoalDisp(2), pedGoalDisp(1)) *180/pi;
             vel = norm( [xVelocity, yVelocity]);
             xVelocity = vel*cosd(calcHeading);
             yVelocity = vel*sind(calcHeading); 
        end
    else        % West Left Lane (Lane 4)
        if abs(calcHeading) > 90
             pedGoalPixels = reset.walkaway.goal(3,1:2);
             pedGoalDisp = reset.walkaway.goal(3,1:2) - pedPosPixels;
             calcHeading = atan2(pedGoalDisp(2), pedGoalDisp(1)) *180/pi;
             vel = norm( [xVelocity, yVelocity]);
             xVelocity = vel*cosd(calcHeading);
             yVelocity = vel*sind(calcHeading); 
        else
             pedGoalPixels = reset.approach.goal(7,1:2);
             pedGoalDisp = reset.approach.goal(7,1:2) - pedPosPixels;
             calcHeading = atan2(pedGoalDisp(2), pedGoalDisp(1)) *180/pi;
             vel = norm( [xVelocity, yVelocity]);
             xVelocity = vel*cosd(calcHeading);
             yVelocity = vel*sind(calcHeading); 
        end
    end
    


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% if crosswalk 3
elseif closestCW==3
   if abs(pedCwAngle(3)) > 90  % South right lane (lane 5)
       if (calcHeading) < 0 
             pedGoalPixels = reset.walkaway.goal(5,1:2);
             pedGoalDisp = reset.walkaway.goal(5,1:2) - pedPosPixels;
             calcHeading = atan2(pedGoalDisp(2), pedGoalDisp(1)) *180/pi;
             vel = norm( [xVelocity, yVelocity]);
             xVelocity = vel*cosd(calcHeading);
             yVelocity = vel*sind(calcHeading); 
       else
             pedGoalPixels = reset.approach.goal(2,1:2);
             pedGoalDisp = reset.approach.goal(2,1:2) - pedPosPixels;
             calcHeading = atan2(pedGoalDisp(2), pedGoalDisp(1)) *180/pi;
             vel = norm( [xVelocity, yVelocity]);
             xVelocity = vel*cosd(calcHeading);
             yVelocity = vel*sind(calcHeading); 
       end
    else        % South Left Lane (Lane 6)
       if (calcHeading) < 0 
             pedGoalPixels = reset.walkaway.goal(6,1:2) ;
             pedGoalDisp = reset.walkaway.goal(6,1:2) - pedPosPixels;
             calcHeading = atan2(pedGoalDisp(2), pedGoalDisp(1)) *180/pi;
             vel = norm( [xVelocity, yVelocity]);
             xVelocity = vel*cosd(calcHeading);
             yVelocity = vel*sind(calcHeading); 
       else
            pedGoalPixels = reset.approach.goal(3,1:2);
            pedGoalDisp = reset.approach.goal(3,1:2) - pedPosPixels;
             calcHeading = atan2(pedGoalDisp(2), pedGoalDisp(1)) *180/pi;
             vel = norm( [xVelocity, yVelocity]);
             xVelocity = vel*cosd(calcHeading);
             yVelocity = vel*sind(calcHeading); 
       end
    end
    



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% if crosswalk 4
elseif closestCW==4
   if abs(pedCwAngle(4)) < 90  % North right lane (Lane 7)
          if (calcHeading) > 0 
             pedGoalPixels = reset.walkaway.goal(7,1:2) ;
             pedGoalDisp = reset.walkaway.goal(7,1:2) - pedPosPixels;
             calcHeading = atan2(pedGoalDisp(2), pedGoalDisp(1)) *180/pi;
             vel = norm( [xVelocity, yVelocity]);
             xVelocity = vel*cosd(calcHeading);
             yVelocity = vel*sind(calcHeading); 
          else
             pedGoalPixels = reset.approach.goal(4,1:2);
             pedGoalDisp = reset.approach.goal(4,1:2) - pedPosPixels;
             calcHeading = atan2(pedGoalDisp(2), pedGoalDisp(1)) *180/pi;
             vel = norm( [xVelocity, yVelocity]);
             xVelocity = vel*cosd(calcHeading);
             yVelocity = vel*sind(calcHeading);
          end
    else        % North Left Lane (Lane 8)
          if (calcHeading) < 0 
             pedGoalPixels = reset.walkaway.goal(8,1:2);
             pedGoalDisp = reset.walkaway.goal(8,1:2) - pedPosPixels;
             calcHeading = atan2(pedGoalDisp(2), pedGoalDisp(1)) *180/pi;
             vel = norm( [xVelocity, yVelocity]);
             xVelocity = vel*cosd(calcHeading);
             yVelocity = vel*sind(calcHeading); 
          else
             pedGoalPixels = reset.approach.goal(1,1:2);
             pedGoalDisp = reset.approach.goal(1,1:2) - pedPosPixels;
             calcHeading = atan2(pedGoalDisp(2), pedGoalDisp(1)) *180/pi;
             vel = norm( [xVelocity, yVelocity]);
             xVelocity = vel*cosd(calcHeading);
             yVelocity = vel*sind(calcHeading); 
          end
   end
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% update states for 1st time step of new tracklet
xCenter = xCenter + del_t*xVelocity;
yCenter = yCenter + del_t*yVelocity;
trackLifetime = trackLifetime + reSampleRate;
%%%%%%%%%%%%%%%%%%%%
% longitudinal velocity (need to know closest CW to calculate this)
if closestCW~=0 && closestCW~=inf
   theta = cw.theta(closestCW);
   rot = [cosd(theta), -sind(theta); sind(theta), cosd(theta)];
   velRot = rot*[xVelocity; yVelocity];
   lonVelocity = velRot(1);
end
%%%%%%%%%%%%%%%%%%%
% update states (kalman predict noise is included)
% kf predict
kf = kalmanPredict(kf);
updated_X = [xCenter;
             yCenter;
             xVelocity;
             yVelocity];
kf.x = updated_X; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

%% reset close CW if it is a new tracklet and CW has been reached

if flag_reachCrosswalk && N==1
    if closestCW==1 && strcmp(Lane,'Right')
        closestCW = 4;
        swInd = 4;
        Lane= 'Left';
    elseif closestCW==1 && strcmp(Lane,'Left')
        closestCW = 3;
        swInd = 3;
        Lane = 'Right';
    elseif closestCW==2 && strcmp(Lane,'Right')
        closestCW = 3;
        swInd = 3;
        Lane= 'Left';
    elseif closestCW==2 && strcmp(Lane,'Left')
        closestCW = 4;
        swInd = 4;
        Lane = 'Right';
    elseif closestCW==3 && strcmp(Lane,'Right')
        closestCW = 1;
        swInd = 1;
        Lane= 'Left';
    elseif closestCW==3 && strcmp(Lane,'Left')
        closestCW = 2;
        swInd = 2;
        Lane = 'Right';
    elseif closestCW==4 && strcmp(Lane,'Right')
        closestCW = 2;
        swInd = 2;
        Lane= 'Left';
    elseif closestCW==4 && strcmp(Lane,'Left')
        closestCW = 1;
        swInd = 1;
        Lane = 'Right';
    end
end
    


%updatedPredData
predData.trackLifetime = trackLifetime;
predData.xCenter = xCenter;
predData.yCenter = yCenter;
predData.xVelocity = xVelocity;
predData.yVelocity = yVelocity;
predData.calcHeading = calcHeading;
predData.goalPositionPixels = pedGoalPixels;
predData.goalDisp = norm(pedGoalDisp)*(orthopxToMeter*scaleFactor);
predData.lonVelocity = lonVelocity;
predData.closestCW = closestCW;
predData.swInd = swInd;
predData.Lane = Lane;



end