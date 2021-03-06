%Note: make sure the enhanced annotataed background image is the input to
%this function
function [tracksData, flag] = func_hybridState_v2(tracksData, cw, flag, annotatedImage_enhanced, Params, trackletNo, resetStates)
%% 1) setup
% fixed parameters
scaleDownFactor = Params.scaleFactor;
orthopxToMeter = Params.orthopxToMeter;
reSampleRate = Params.reSampleRate;
% parameters
cwHeadingThreshold = Params.cwHeadingThreshold; %45 degrees
swHeadingThreshold = Params.swHeadingThreshold; %45 degrees
stoppingThreshold = Params.stoppingThreshold; %speed m/s
walkingThreshold = Params.walkingThreshold; %speed m/s 
cwDistThreshold = Params.cwDistThreshold; %in pixels
cwCrossThreshold = Params.cwCrossThreshold; % approximate width of a lane
decZone = Params.decZone; %8 m radius (in pixels)
roadOrientation = Params.roadOrientation;
% initialize variables
% isCrossing = false;
lonVelocity = inf;
cwInd = tracksData.closestCW(end);
longDispPedCwPixels = inf;
latDispPedCwPixels = inf;
Lane = tracksData.Lane(end);

% the previous time step data is copied
prevCWDispSign = sign(tracksData.longDispPedCw(end));
prevSwInd = tracksData.swInd(end);
prevCwInd = tracksData.closestCW(end);
prevLane = tracksData.Lane(end);

% check sidewalk number 
if prevSwInd==1 && strcmp(prevLane,'Right')
    prevSwInd_side = 1;
elseif prevSwInd==1
    prevSwInd_side = 2;
elseif  prevSwInd==2 && strcmp(prevLane,'Right')
    prevSwInd_side = 3;
elseif prevSwInd==2
    prevSwInd_side = 4;
elseif  prevSwInd==3 && strcmp(prevLane,'Right')
    prevSwInd_side = 5;
elseif prevSwInd==3
    prevSwInd_side = 6;
elseif prevSwInd==4 && strcmp(prevLane,'Right')
    prevSwInd_side=7;
elseif prevSwInd==4
    prevSwInd_side=8;
end

if size(tracksData.HybridState,1) ~= 1
    hybrid_state = tracksData.HybridState(end-1,:);
    tracksData.waitTimeSteps(end) = tracksData.waitTimeSteps(end-1);
else
    hybrid_state = tracksData.HybridState(end,:);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% convert track data to pixels
xCenterPix = (tracksData.xCenter/(scaleDownFactor*orthopxToMeter));
yCenterPix = (tracksData.yCenter/(scaleDownFactor*orthopxToMeter));
pedHead = tracksData.calcHeading(end);
% length of this particular pedestrian track    
% N_instances = size(tracksData.xCenter,1);
N_instances = 1;    % need to find only for the last time step
Region = strings(N_instances,1);
cwHeadingState = strings(4, 1); % to check if they are headed towards each of the four crosswalks
cwHeadingState(1:4) = strings;
cwHeadingCloseCW = strings;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 2) Hybrid state loop    
if ~flag.outOfPlay
%for ii=1:N_instances   % need to find only for the last time step
       % initialize
       dist_cw_temp = [inf, inf, inf, inf];  
       % index based on the current pixel position of pedestrian
       posPixels = ([xCenterPix(end), yCenterPix(end)]);
       %%%%%%%%%%%%%%%%%%%%
       %% 2a)check pedestrian position, heading, and velocity discrete states
       % a) is pedestrian heading towards a crosswalk?
       % heading angle of pedestrians (to reduce noise in the estimation of
       % heading because of noisy position and velocity data)
        y_vel = tracksData.yVelocity(end);
        y_vel_head = y_vel;
        if ( abs(y_vel) < walkingThreshold )
            y_vel_head = 0;
        end
        x_vel = tracksData.xVelocity(end);
        x_vel_head = x_vel;
        if ( abs(x_vel) < walkingThreshold )
            x_vel_head = 0;
        end
        % when pedestrian is stopped, maintain the previous heading
        % instead of saying it as zero.
        if x_vel_head~=0 && y_vel_head~=0
            pedHead = atan2(y_vel_head, x_vel_head)*180/pi; 
        end
        %%%%%%%%%%%%%%%%%%%%
       % angle between pedestrian and the crosswalks
        ped_cw1_angle = atan2(double(cw.center_y(1) - posPixels(2)), double(cw.center_x(1) - posPixels(1)))*180/pi;
        ped_cw2_angle = atan2(double(cw.center_y(2) - posPixels(2)), double(cw.center_x(2) - posPixels(1)))*180/pi;  
        ped_cw3_angle = atan2(double(cw.center_y(3) - posPixels(2)), double(cw.center_x(3) - posPixels(1)))*180/pi;  
        ped_cw4_angle = atan2(double(cw.center_y(4) - posPixels(2)), double(cw.center_x(4) - posPixels(1)))*180/pi;  
        %%%%%%%%%%%%%%%%%%%%
       % distance between pedestrian and crosswalk (in pixels)
       dist_cw1 = sqrt(double(cw.center_x(1) - posPixels(1))^2 + double(cw.center_y(1) - posPixels(2))^2);
       dist_cw2 = sqrt(double(cw.center_x(2) - posPixels(1))^2 + double(cw.center_y(2) - posPixels(2))^2); 
       dist_cw3 = sqrt(double(cw.center_x(3) - posPixels(1))^2 + double(cw.center_y(3) - posPixels(2))^2);
       dist_cw4 = sqrt(double(cw.center_x(4) - posPixels(1))^2 + double(cw.center_y(4) - posPixels(2))^2);
       %%%%%%%%%%%%%%%%%%%%%%%%
       dist_cw_temp = [dist_cw1; dist_cw2; dist_cw3; dist_cw4];  
       %%%%%%%%%%%%%%%%%%%%%%%%
       dispRot_cw1 = [cosd(cw.theta(1)), -sind(cw.theta(1)); sind(cw.theta(1)), cosd(cw.theta(1))] * ([cw.center_x(1),cw.center_y(1)] - posPixels)';
       dispRot_cw2 = [cosd(cw.theta(2)), -sind(cw.theta(2)); sind(cw.theta(2)), cosd(cw.theta(2))] * ([cw.center_x(2),cw.center_y(2)] - posPixels)'; 
       dispRot_cw3 = [cosd(cw.theta(3)), -sind(cw.theta(3)); sind(cw.theta(3)), cosd(cw.theta(3))] * ([cw.center_x(3),cw.center_y(3)] - posPixels)';
       dispRot_cw4 = [cosd(cw.theta(4)), -sind(cw.theta(4)); sind(cw.theta(4)), cosd(cw.theta(4))] * ([cw.center_x(4),cw.center_y(4)] - posPixels)';
       
       % shift to align the long. disp. along column 1
       dispRot_cw3 = [dispRot_cw3(2); dispRot_cw3(1)];
       dispRot_cw4 = [dispRot_cw4(2); dispRot_cw4(1)];
       dispRot_cw = [dispRot_cw1'; dispRot_cw2'; dispRot_cw3'; dispRot_cw4'];
       %%%%%%%%%%%%%%%%%%%%%%%%
       
       Region(end) = "OutOfRange";
       onRoad = false;
       posPixels = int32([xCenterPix(end), yCenterPix(end)]);
       if posPixels(1) > 0 && posPixels(1)<= 1170 && posPixels(2) < 0 && posPixels(2) >= -780
           % current region of pedestrian 
            if (annotatedImage_enhanced(-posPixels(2), posPixels(1))==200)
               Region(end) = "Crosswalk_Marked";
               onRoad = true;
            elseif (annotatedImage_enhanced(-posPixels(2), posPixels(1))==100)
               Region(end) = "Crosswalk_UnMarked";
               onRoad = true;
            elseif (annotatedImage_enhanced(-posPixels(2), posPixels(1))==50)
               Region(end) = "Road";
               onRoad = true;
            elseif (annotatedImage_enhanced(-posPixels(2), posPixels(1))==150)
               Region(end) = "Sidewalk";  
               onRoad = false;
            elseif (annotatedImage_enhanced(-posPixels(2), posPixels(1))==0) && ...
                   ( ( (posPixels(1)>=Params.sidewalk.xmin(1)) && (posPixels(1)<=Params.sidewalk.xmax(1)) && (-posPixels(2)>=-Params.sidewalk.ymin(1)) && (-posPixels(2)<=-Params.sidewalk.ymax(1))  ) ||...
                     ( (posPixels(1)>=Params.sidewalk.xmin(2)) && (posPixels(1)<=Params.sidewalk.xmax(2)) && (-posPixels(2)>=-Params.sidewalk.ymin(2)) && (-posPixels(2)<=-Params.sidewalk.ymax(2))  ) )
               Region(end) = "Sidewalk";  
               onRoad = false;
            elseif (annotatedImage_enhanced(-posPixels(2), posPixels(1))==0)
                Region(end) = "OutOfRange";
                onRoad = false;
            else          
               onRoad = false;
            end 
       end
       %%%%%%%%%%%%%%%%%%%%
       posPixels = ([xCenterPix(end), yCenterPix(end)]);
       
       %% Pedestrian lane calculation
       %%%%%%%%%%%%%%%%%
       % Displacement to the sidewalk (near the crosswalk)
       dispPedSW(1,1:2) = (resetStates.approach.goal(1,1:2) - double(posPixels));
       dispPedSW(1,3:4) = (resetStates.approach.goal(2,1:2) - double(posPixels));
       dispPedSW(2,1:2) = (resetStates.approach.goal(3,1:2) - double(posPixels));
       dispPedSW(2,3:4) = (resetStates.approach.goal(4,1:2) - double(posPixels));
       dispPedSW(3,1:2) = (resetStates.approach.goal(5,1:2) - double(posPixels));
       dispPedSW(3,3:4) = (resetStates.approach.goal(6,1:2) - double(posPixels));
       dispPedSW(4,1:2) = (resetStates.approach.goal(7,1:2) - double(posPixels));
       dispPedSW(4,3:4) = (resetStates.approach.goal(8,1:2) - double(posPixels));      
       % distance to the sidewalk
       distPedLane(1,1) = norm(dispPedSW(1,1:2));
       distPedLane(2,1) = norm(dispPedSW(1,3:4));
       distPedLane(3,1) = norm(dispPedSW(2,1:2));
       distPedLane(4,1) = norm(dispPedSW(2,3:4));
       distPedLane(5,1) = norm(dispPedSW(3,1:2));
       distPedLane(6,1) = norm(dispPedSW(3,3:4));
       distPedLane(7,1) = norm(dispPedSW(4,1:2));
       distPedLane(8,1) = norm(dispPedSW(4,3:4));
       % initialize closest SW
       [~, sortedSWindex] = sort(distPedLane);
       swInd_side = sortedSWindex(1);
       swInd = ceil(swInd_side/2);
       if swInd==0
           swInd=4;
       end
       if mod(swInd_side,2) ~= 0
           Lane = "Right";
       else
           Lane = "Left";
       end
       
       % check if this is a feasible SW lane
       isSWTransPossible = false;
       sw_sort_ind = 0;
       while(sw_sort_ind<8 && ~isSWTransPossible)
           % initialize
           sw_sort_ind = sw_sort_ind + 1;
           swInd_side = sortedSWindex(sw_sort_ind);
           swInd = ceil(swInd_side/2);

           % sidewalk lane calculation
           if  mod(swInd_side,2) ~= 0
               Lane = "Right";
           elseif ~onRoad
               Lane = "Left";
           end

           % check
            if ( (prevSwInd==1 && strcmp(prevLane,"Right")) && ( (swInd==1 && strcmp(Lane,"Right")) || (swInd==1 && strcmp(Lane,"Left")) || (swInd==4 && strcmp(Lane,"Left")) ) )
                   isSWTransPossible = true;
            elseif ( (prevSwInd==1 && strcmp(prevLane,"Left") ) && ( (swInd==1 && strcmp(Lane,"Left")) || (swInd==1 && strcmp(Lane,"Right")) || (swInd==3 && strcmp(Lane,"Right")) ) )
                   isSWTransPossible = true;   
            elseif ( (prevSwInd==2 && strcmp(prevLane,"Right") ) && ( (swInd==2 && strcmp(Lane,"Right")) || (swInd==2 && strcmp(Lane,"Left")) || (swInd==3 && strcmp(Lane,"Left")) ) )
                   isSWTransPossible = true; 
            elseif ( (prevSwInd==2 && strcmp(prevLane,"Left") ) && ( (swInd==2 && strcmp(Lane,"Left")) || (swInd==2 && strcmp(Lane,"Right")) || (swInd==4 && strcmp(Lane,"Right")) ) )
                   isSWTransPossible = true; 
            elseif ( (prevSwInd==3 && strcmp(prevLane,"Right") ) && ( (swInd==3 && strcmp(Lane,"Right")) || (swInd==3 && strcmp(Lane,"Left")) || (swInd==1 && strcmp(Lane,"Left")) ) )
                   isSWTransPossible = true;
            elseif ( (prevSwInd==3 && strcmp(prevLane,"Left") ) && ( (swInd==3 && strcmp(Lane,"Left")) || (swInd==3 && strcmp(Lane,"Right")) || (swInd==2 && strcmp(Lane,"Right")) ) )
                   isSWTransPossible = true; 
            elseif ( (prevSwInd==4 && strcmp(prevLane,"Right") ) && ( (swInd==4 && strcmp(Lane,"Right")) || (swInd==4 && strcmp(Lane,"Left")) || (swInd==2 && strcmp(Lane,"Left")) ) )
                   isSWTransPossible = true; 
            elseif ( (prevSwInd==4 && strcmp(prevLane,"Left") ) && ( (swInd==4 && strcmp(Lane,"Left")) || (swInd==4 && strcmp(Lane,"Right")) || (swInd==1 && strcmp(Lane,"Right")) ) )
                   isSWTransPossible = true; 
            else
                 % transition not feasible; maintain the previous
                 % sidewalk lane
                 isSWTransPossible = false;
                 swInd = prevSwInd;
                 Lane = prevLane;  
                 swInd_side = prevSwInd_side;
            end
       end       
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
             
       % when the heading angle is within +/- heading_threshold and pedestrian is close to a crosswalk, 
       % then pedestrian is approaching one of the crosswalks   
       if  ( ( (abs(ped_cw1_angle - pedHead) <= cwHeadingThreshold || abs(ped_cw1_angle - pedHead) > 360-cwHeadingThreshold ) && dist_cw1 < cwDistThreshold ) || (dist_cw1 < cwCrossThreshold && onRoad))
            cwHeadingState(1) = 'Headed';
%             dist_cw_temp(1) = dist_cw1;
       elseif ( ( (abs(ped_cw1_angle - pedHead) > cwHeadingThreshold + 10 ||  abs(ped_cw1_angle - pedHead) < 360 - cwHeadingThreshold - 10 ) || dist_cw1 > cwDistThreshold ) && (dist_cw1 > cwCrossThreshold) || ~onRoad )
            cwHeadingState(1) = 'Not_Headed';
       end
       if ( ( (abs(ped_cw2_angle - pedHead) <= cwHeadingThreshold || abs(ped_cw1_angle - pedHead) > 360-cwHeadingThreshold ) && dist_cw2 < cwDistThreshold ) || (dist_cw2 < cwCrossThreshold && onRoad))
            cwHeadingState(2) = 'Headed';
%             dist_cw_temp(2) = dist_cw2;
       elseif ( ( (abs(ped_cw2_angle - pedHead) > cwHeadingThreshold + 10 ||  abs(ped_cw1_angle - pedHead) < 360 - cwHeadingThreshold - 10 ) || dist_cw2 > cwDistThreshold ) && (dist_cw2 > cwCrossThreshold) || ~onRoad )
            cwHeadingState(2) = 'Not_Headed';
       end
       if ( ( (abs(ped_cw3_angle - pedHead) <= cwHeadingThreshold || abs(ped_cw1_angle - pedHead) > 360-cwHeadingThreshold ) && dist_cw3 < cwDistThreshold ) || (dist_cw3 < cwCrossThreshold && onRoad))
            cwHeadingState(3) = 'Headed';
%             dist_cw_temp(3) = dist_cw3; 
       elseif ( ( ( abs(ped_cw3_angle - pedHead) > cwHeadingThreshold + 10 ||  abs(ped_cw1_angle - pedHead) < 360 - cwHeadingThreshold - 10 ) || dist_cw3 > cwDistThreshold ) && (dist_cw3 > cwCrossThreshold) || ~onRoad )
            cwHeadingState(3) = 'Not_Headed';
       end
       if ( ( (abs(ped_cw4_angle - pedHead) <= cwHeadingThreshold || abs(ped_cw1_angle - pedHead) > 360-cwHeadingThreshold ) && dist_cw4 < cwDistThreshold ) || (dist_cw4 < cwCrossThreshold && onRoad) )
            cwHeadingState(4) = 'Headed';
%             dist_cw_temp(4) = dist_cw4;
       elseif ( ( ( abs(ped_cw4_angle - pedHead) > cwHeadingThreshold + 10 ||  abs(ped_cw1_angle - pedHead) < 360 - cwHeadingThreshold - 10 ) || dist_cw4 > cwDistThreshold ) && (dist_cw4 > cwCrossThreshold) || ~onRoad )
            cwHeadingState(4) = 'Not_Headed';
       end
       %%%%%%%%%%%%%%%%%%%%
              
        %% closest CW calculation
       [~, sortedcwIndex] = sort(dist_cw_temp);
       % initialize while loop parameters
       sort_cw_ind = 0;
       runLoop = true;
       % while loop to find closest CW that satisfies all three conditions:
       % (i) feasible transition, (ii) closest CW, (iii) heading in the CW
       % direction
       while(sort_cw_ind<4 && runLoop && ~onRoad)          
               %%%%%%%%%%%%%%%%%%%%
               % initialize
               sort_cw_ind = sort_cw_ind + 1;
               cwInd = sortedcwIndex(sort_cw_ind);
               cwHeadingCloseCW = cwHeadingState(cwInd);
               %%%%%%%%%%%%%%%%%%%%
               % (i) check closest CW transition feasibility. Note that for
               % feasibilty check we use the sidewalk the pedestrian is
               % close to.
               if prevCwInd~=cwInd && swInd~=inf
                    if ( (swInd==1 && strcmp(Lane,"Right")) && (cwInd==1 || cwInd==4) )
                           isTransitionPossible = true;
                    elseif ( (swInd==1 && strcmp(Lane,"Left") ) && ( cwInd==1 || cwInd==3) )
                           isTransitionPossible = true;   
                    elseif ( (swInd==2 && strcmp(Lane,"Right") ) && ( cwInd==2 || cwInd==3) )
                           isTransitionPossible = true; 
                    elseif ( (swInd==2 && strcmp(Lane,"Left") ) && ( cwInd==2 || cwInd==4) )
                           isTransitionPossible = true; 
                    elseif ( (swInd==3 && strcmp(Lane,"Right") ) && ( cwInd==3 || cwInd==1) )
                           isTransitionPossible = true;
                    elseif ( (swInd==3 && strcmp(Lane,"Left") ) && ( cwInd==3 || cwInd==2) )
                           isTransitionPossible = true; 
                    elseif ( (swInd==4 && strcmp(Lane,"Right") ) && ( cwInd==4 || cwInd==2) )
                           isTransitionPossible = true; 
                    elseif ( (swInd==4 && strcmp(Lane,"Left") ) && ( cwInd==4 || cwInd==1) )
                           isTransitionPossible = true; 
                    else
                         isTransitionPossible = false; 
                    end
               % first time step or first close CW calculation
               elseif (prevCwInd==inf || prevCwInd==cwInd)
                   isTransitionPossible = true;
               end
               %%%%%%%%%%%%%%%%%%%%
               if isTransitionPossible
                   %%%%%%%%%%%%%%%%%%%%
                   % check close CW heading
                   if strcmp(cwHeadingCloseCW, 'Headed')
                       runLoop = false;
                   end   
               end
       end
       % if while loop did not find a close CW solution, use the previous
       % value
       if ( (runLoop && prevCwInd~=0) || swInd==0)
           cwInd = prevCwInd;
       elseif runLoop && prevCwInd==0
           cwInd = sortedcwIndex(1);
       end
       % update the SW distance (in m)
       distSW = distPedLane(swInd_side) * scaleDownFactor *orthopxToMeter;
       

       % longitudinal velocity (need to know closest CW to calculate this)
       if cwInd~=0 && cwInd~=inf
           theta = cw.theta(cwInd);
           rot = [cosd(theta), -sind(theta); sind(theta), cosd(theta)];
           velRot = rot*[x_vel; y_vel];
           lonVelocity = velRot(1);
       end
             
       %%%%%%%%%%%%%%%%%%%%
       % b) is pedestrian walking?
       ped_vel = sqrt(tracksData.xVelocity(end)^2 + tracksData.yVelocity(end)^2);
       if ped_vel < stoppingThreshold
           walk_state = 'Stopping';
       else
           walk_state = 'Walking';
       end
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       % check heading based on heading and road orientation
       if cwInd==1
%            if (pedHead >= roadOrientation(1) - 80 && pedHead <= roadOrientation(1) + 80)
           if (abs(pedHead - roadOrientation(1)) < 80 || abs(pedHead - roadOrientation(1)) > 280)
               if sign(dispRot_cw(1,1)) > 0
                   approachHeadingState = 'Headed';
                   longDispPedCwPixels = dispRot_cw(1,1);
               else
                   approachHeadingState = 'Not_Headed';
                   longDispPedCwPixels = dispRot_cw(1,1);
               end
           else
               if sign(dispRot_cw(1,1)) < 0
                   approachHeadingState = 'Headed';
                   longDispPedCwPixels = abs(dispRot_cw(1,1));
               else
                   approachHeadingState = 'Not_Headed';
                   longDispPedCwPixels = -abs(dispRot_cw(1,1));
               end
           end
           
       elseif cwInd==2
%            if (pedHead >= roadOrientation(2) - 80 && pedHead <= roadOrientation(2) + 80)
           if (abs(pedHead - roadOrientation(1)) < 80 || abs(pedHead - roadOrientation(1)) > 280)
               if sign(dispRot_cw(2,1)) < 0
                   approachHeadingState = 'Headed';
                   longDispPedCwPixels = abs(dispRot_cw(2,1));
               else
                   approachHeadingState = 'Not_Headed';
                   longDispPedCwPixels = -abs(dispRot_cw(2,1));
               end
           else
               if sign(dispRot_cw(2,1)) > 0
                   approachHeadingState = 'Headed';
                   longDispPedCwPixels = dispRot_cw(2,1);
               else
                   approachHeadingState = 'Not_Headed';
                   longDispPedCwPixels = dispRot_cw(2,1);
               end
           end
           
           
       elseif cwInd==3
%           if (pedHead >= roadOrientation(3) - 80 && pedHead <= roadOrientation(3) + 80)
          if (abs(pedHead - roadOrientation(1)) < 80 || abs(pedHead - roadOrientation(1)) > 280)
               if sign(dispRot_cw(3,1)) < 0
                   approachHeadingState = 'Headed';
                   longDispPedCwPixels = abs(dispRot_cw(3,1));
               else
                   approachHeadingState = 'Not_Headed';
                   longDispPedCwPixels = -abs(dispRot_cw(3,1));
               end
           else
               if sign(dispRot_cw(3,1)) > 0
                   approachHeadingState = 'Headed';
                   longDispPedCwPixels = (dispRot_cw(3,1));
               else
                   approachHeadingState = 'Not_Headed';
                   longDispPedCwPixels = (dispRot_cw(3,1));
               end
           end
           
       elseif cwInd==4
%            if (pedHead >= roadOrientation(4) - 80 && pedHead <= roadOrientation(4) + 80)
           if (abs(pedHead - roadOrientation(1)) < 80 || abs(pedHead - roadOrientation(1)) > 280)
               if sign(dispRot_cw(4,1)) > 0
                   approachHeadingState = 'Headed';
                   longDispPedCwPixels = (dispRot_cw(4,1));
               else
                   approachHeadingState = 'Not_Headed';
                   longDispPedCwPixels = (dispRot_cw(4,1));
               end
           else
               if sign(dispRot_cw(4,1)) < 0
                   approachHeadingState = 'Headed';
                   longDispPedCwPixels = abs(dispRot_cw(4,1));
               else
                   approachHeadingState = 'Not_Headed';
                   longDispPedCwPixels = -abs(dispRot_cw(4,1));
               end
           end
           
       end
       % lateral displacement to crosswalk
        latDispPedCwPixels = abs(dispRot_cw(cwInd,2)) - cw.centerLatOffset(cwInd);
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% 2b) Hybrid state update    
    % run the hybrid state update only when needed (i.e. during update
    % stage, and when the hybrid state needs to be identified from the
    % continuous state predictions)
    if ~flag.hybridStatePred(trackletNo)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % conditions for Hybrid State of Pedestrians
        % (1) approach: if pedestrian is on sidewalk and headed towards a crosswalk
%             if ( strcmp(Region(end),"Sidewalk") && strcmp(cwHeadingCloseCW,'Headed') && (strcmp(walk_state, 'Walking') || (strcmp(walk_state, 'Stopping') && (distSW > decZone) ) ) )
            if ( ~strcmp(hybrid_state,"Walk_away") && strcmp(Region(end),"Sidewalk") && ( strcmp(walk_state, 'Walking') || (strcmp(walk_state, 'Stopping') && distSW > decZone) )...
               && ( strcmp(cwHeadingCloseCW,'Headed') || (length(tracksData.xCenter)==1 && strcmp(approachHeadingState,'Headed') ) ) )
                hybrid_state = "Approach";
        % (2) wait: if pedestrian is on sidewalk near a crosswalk and is walking very slowly. 
        % Note: Using absolute distance to center of crosswalk for now.
            elseif ( ~strcmp(hybrid_state,"Walk_away") && strcmp(Region(end),"Sidewalk") && strcmp(cwHeadingCloseCW,'Headed') && strcmp(walk_state, 'Stopping') && (distSW < decZone) )
                hybrid_state = "Wait"; 
        % (3) cross: if pedestrian is on the road (note if they are crossing or
        % jaywalking)
            elseif (strcmp(Region(end),"Crosswalk_Marked") || strcmp(Region(end),"Crosswalk_UnMarked") )
                hybrid_state = "Crossing";                
            elseif (strcmp(Region(end),"Road") )
%                hybrid_state = 'Jaywalking'; % neglect Jaywalking for now
                hybrid_state = "Crossing";
        % (4) walkaway: if pedestrian just crossed the street and is on the
        % sidewalk
            elseif ( (strcmp(Region(end),"Sidewalk") || strcmp(Region(end),"OutOfRange")) && ( sign(dispRot_cw(cwInd,1))~= prevCWDispSign || strcmp(approachHeadingState,'Not_Headed') ) )
                hybrid_state = "Walk_away";               
            end

            % update the state variables not relevant for predictions
            tracksData.HybridState(end) = hybrid_state;  

    end % end of hybrid update (not prediction) loop
    
    % check if pedestrian is close to the crosswalk
    if abs(longDispPedCwPixels) < 5 && abs(latDispPedCwPixels) < 50
        flag.closeToCrosswalk(trackletNo) = true;
    else
        flag.closeToCrosswalk(trackletNo) = false;
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% update other variables
    tracksData.closestCW(end) = cwInd;
    tracksData.swInd(end) = swInd;
    tracksData.lonVelocity(end) = lonVelocity;
    tracksData.Lane(end) = Lane;
    tracksData.longDispPedCw(end) = longDispPedCwPixels * scaleDownFactor*orthopxToMeter;
    tracksData.latDispPedCw(end) = latDispPedCwPixels * scaleDownFactor*orthopxToMeter;
    
    % update wait time; wait time initialized to previous time step
    % wait time in 'updatePedContStates'
    if (strcmp(tracksData.HybridState(end), 'Wait') )
        if tracksData.waitTimeSteps(end)==-1 %i.e. no wait before this
            tracksData.waitTimeSteps(end) = 0; % wait starts
        else
            tracksData.waitTimeSteps(end) = tracksData.waitTimeSteps(end) + reSampleRate;
        end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% end

% update region
%tracksData.Region = Region;
end

end

