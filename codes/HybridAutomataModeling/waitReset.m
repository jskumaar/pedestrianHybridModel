function predData = waitReset(predData, cw, reset)

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
    if pedCwAngle(1) < 0  % West right lane (Lane 1)
             predData.calcHeading(end) = reset.wait.heading(1,:);
             predData.xVelocity(end) = 0;
             predData.yVelocity(end) = 0;
             predData.waitTimeSteps(end) = 0;
         
    else        % West Left Lane (lane 2)
             predData.calcHeading(end) = reset.wait.heading(2,:);
             predData.xVelocity(end) = 0;
             predData.yVelocity(end) = 0;
             predData.waitTimeSteps(end) = 0;
    end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
% if crosswalk 2
elseif predData.closestCW(end)==2

    if pedCwAngle(2) > 0  % East right lane (Lane 3)
             predData.calcHeading(end) = reset.wait.heading(3,:);
             predData.xVelocity(end) = 0;
             predData.yVelocity(end) = 0;
             predData.waitTimeSteps(end) = 0;
    else        % East Left Lane (Lane 4)
             predData.calcHeading(end) = reset.wait.heading(4,:);
             predData.xVelocity(end) = 0;
             predData.yVelocity(end) = 0;
             predData.waitTimeSteps(end) = 0;
    end
    


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% if crosswalk 3
elseif predData.closestCW(end)==3
   if abs(pedCwAngle(3)) > 90  % South right lane (lane 5)
             predData.calcHeading(end) = reset.wait.heading(5,:);
             predData.xVelocity(end) = 0;
             predData.yVelocity(end) = 0;
             predData.waitTimeSteps(end) = 0;
    else        % South Left Lane (Lane 6)
             predData.calcHeading(end) = reset.wait.heading(6,:);
             predData.xVelocity(end) = 0;
             predData.yVelocity(end) = 0;
             predData.waitTimeSteps(end) = 0;
    end
    



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% if crosswalk 4
elseif predData.closestCW(end)==4
   if abs(pedCwAngle(4)) < 90  % North right lane (Lane 7)
             predData.calcHeading(end) = reset.wait.heading(7,:);
             predData.xVelocity(end) = 0;
             predData.yVelocity(end) = 0;
             predData.waitTimeSteps(end) = 0;
    else        % North Left Lane (Lane 8)
             predData.calcHeading(end) = reset.wait.heading(8,:);
             predData.xVelocity(end) = 0;
             predData.yVelocity(end) = 0;
             predData.waitTimeSteps(end) = 0;
   end
    
end




end