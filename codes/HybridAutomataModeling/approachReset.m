function predData = approachReset(predData, cw, reset)

% initialize
pedPosPixels = double([predData.xCenterPix(end), predData.yCenterPix(end)]);


% calculate angle between pedestrian and approaching crosswalk
pedCwAngle = [ atan2(([cw.center_y(1) - pedPosPixels(2)]), ([cw.center_x(1) - pedPosPixels(1)]))*180/pi;
                 atan2(([cw.center_y(2) - pedPosPixels(2)]), ([cw.center_x(2) - pedPosPixels(1)]))*180/pi;
                 atan2(([cw.center_y(3) - pedPosPixels(2)]), ([cw.center_x(3) - pedPosPixels(1)]))*180/pi;
                 atan2(([cw.center_y(4) - pedPosPixels(2)]), ([cw.center_x(4) - pedPosPixels(1)]))*180/pi];
             


%% tracklet for crossing intent
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
% if crosswalk 1
if predData.closestCW(end)==1
    if pedCwAngle(1) < 0  % East right lane (Lane 1)
        %approaching from left
        if abs(predData.calcHeading(end)) < 90 
             pedGoalDisp = reset.walkaway.goal(1,:) - pedPosPixels;
             predData.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1)) *180/pi;
             vel = norm( [predData.xVelocity(end), predData.yVelocity(end)]);
             predData.xVelocity(end) = vel*cosd(predData.calcHeading(end));
             predData.yVelocity(end) = vel*sind(predData.calcHeading(end)); 
        else
             pedGoalDisp = reset.approach.goal(8,:) - pedPosPixels;
             predData.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1)) *180/pi;
             vel = norm( [predData.xVelocity(end), predData.yVelocity(end)]);
             predData.xVelocity(end) = vel*cosd(predData.calcHeading(end));
             predData.yVelocity(end) = vel*sind(predData.calcHeading(end)); 
        end
         
    else        % EAst Left Lane (lane 2)
        if abs(predData.calcHeading(end)) < 90 
             pedGoalDisp = reset.walkaway.goal(2,:) - pedPosPixels;
             predData.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1)) *180/pi;
             vel = norm( [predData.xVelocity(end), predData.yVelocity(end)]);
             predData.xVelocity(end) = vel*cosd(predData.calcHeading(end));
             predData.yVelocity(end) = vel*sind(predData.calcHeading(end)); 
        else
             pedGoalDisp = reset.approach.goal(5,:) - pedPosPixels;
             predData.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1)) *180/pi;
             vel = norm( [predData.xVelocity(end), predData.yVelocity(end)]);
             predData.xVelocity(end) = vel*cosd(predData.calcHeading(end));
             predData.yVelocity(end) = vel*sind(predData.calcHeading(end)); 
        end
    end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
% if crosswalk 2
elseif predData.closestCW(end)==2

    if pedCwAngle(2) > 0  % West right lane (Lane 3)
        if abs(predData.calcHeading(end)) > 90 
             pedGoalDisp = reset.walkaway.goal(3,:) - pedPosPixels;
             predData.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1)) *180/pi;
             vel = norm( [predData.xVelocity(end), predData.yVelocity(end)]);
             predData.xVelocity(end) = vel*cosd(predData.calcHeading(end));
             predData.yVelocity(end) = vel*sind(predData.calcHeading(end));             
        else
             pedGoalDisp = reset.approach.goal(6,:) - pedPosPixels;
             predData.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1)) *180/pi;
             vel = norm( [predData.xVelocity(end), predData.yVelocity(end)]);
             predData.xVelocity(end) = vel*cosd(predData.calcHeading(end));
             predData.yVelocity(end) = vel*sind(predData.calcHeading(end)); 
        end
    else        % West Left Lane (Lane 4)
        if abs(predData.calcHeading(end)) > 90 
             pedGoalDisp = reset.walkaway.goal(3,:) - pedPosPixels;
             predData.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1)) *180/pi;
             vel = norm( [predData.xVelocity(end), predData.yVelocity(end)]);
             predData.xVelocity(end) = vel*cosd(predData.calcHeading(end));
             predData.yVelocity(end) = vel*sind(predData.calcHeading(end)); 
        else
             pedGoalDisp = reset.approach.goal(7,:) - pedPosPixels;
             predData.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1)) *180/pi;
             vel = norm( [predData.xVelocity(end), predData.yVelocity(end)]);
             predData.xVelocity(end) = vel*cosd(predData.calcHeading(end));
             predData.yVelocity(end) = vel*sind(predData.calcHeading(end)); 
        end
    end
    


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% if crosswalk 3
elseif predData.closestCW(end)==3
   if abs(pedCwAngle(3)) > 90  % South right lane (lane 5)
       if (predData.calcHeading(end)) < 0 
             pedGoalDisp = reset.walkaway.goal(5,:) - pedPosPixels;
             predData.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1)) *180/pi;
             vel = norm( [predData.xVelocity(end), predData.yVelocity(end)]);
             predData.xVelocity(end) = vel*cosd(predData.calcHeading(end));
             predData.yVelocity(end) = vel*sind(predData.calcHeading(end)); 
       else
             pedGoalDisp = reset.approach.goal(2,:) - pedPosPixels;
             predData.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1)) *180/pi;
             vel = norm( [predData.xVelocity(end), predData.yVelocity(end)]);
             predData.xVelocity(end) = vel*cosd(predData.calcHeading(end));
             predData.yVelocity(end) = vel*sind(predData.calcHeading(end)); 
       end
    else        % South Left Lane (Lane 6)
       if (predData.calcHeading(end)) < 0 
             pedGoalDisp = reset.walkaway.goal(6,:) - pedPosPixels;
             predData.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1)) *180/pi;
             vel = norm( [predData.xVelocity(end), predData.yVelocity(end)]);
             predData.xVelocity(end) = vel*cosd(predData.calcHeading(end));
             predData.yVelocity(end) = vel*sind(predData.calcHeading(end)); 
       else
             pedGoalDisp = reset.approach.goal(3,:) - pedPosPixels;
             predData.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1)) *180/pi;
             vel = norm( [predData.xVelocity(end), predData.yVelocity(end)]);
             predData.xVelocity(end) = vel*cosd(predData.calcHeading(end));
             predData.yVelocity(end) = vel*sind(predData.calcHeading(end)); 
       end
    end
    



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% if crosswalk 4
elseif predData.closestCW(end)==4
   if abs(pedCwAngle(4)) < 90  % North right lane (Lane 7)
          if (predData.calcHeading(end)) > 0 
             pedGoalDisp = reset.walkaway.goal(7,:) - pedPosPixels;
             predData.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1)) *180/pi;
             vel = norm( [predData.xVelocity(end), predData.yVelocity(end)]);
             predData.xVelocity(end) = vel*cosd(predData.calcHeading(end));
             predData.yVelocity(end) = vel*sind(predData.calcHeading(end)); 
          else
             pedGoalDisp = reset.approach.goal(4,:) - pedPosPixels;
             predData.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1)) *180/pi;
             vel = norm( [predData.xVelocity(end), predData.yVelocity(end)]);
             predData.xVelocity(end) = vel*cosd(predData.calcHeading(end));
             predData.yVelocity(end) = vel*sind(predData.calcHeading(end));
          end
    else        % North Left Lane (Lane 8)
          if (predData.calcHeading(end)) < 0 
             pedGoalDisp = reset.walkaway.goal(8,:) - pedPosPixels;
             predData.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1)) *180/pi;
             vel = norm( [predData.xVelocity(end), predData.yVelocity(end)]);
             predData.xVelocity(end) = vel*cosd(predData.calcHeading(end));
             predData.yVelocity(end) = vel*sind(predData.calcHeading(end)); 
          else
             pedGoalDisp = reset.approach.goal(1,:) - pedPosPixels;
             predData.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1)) *180/pi;
             vel = norm( [predData.xVelocity(end), predData.yVelocity(end)]);
             predData.xVelocity(end) = vel*cosd(predData.calcHeading(end));
             predData.yVelocity(end) = vel*sind(predData.calcHeading(end)); 
          end
   end
    
end




end