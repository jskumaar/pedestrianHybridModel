%Note: make sure the enhanced annotataed background image is the input to
%this function
function [tracksData] = hybridStateCopy_v3(tracksData, cw, flag, annotatedImage_enhanced, Params, trackletNo, resetStates)
%% 1) setup
% fixed parameters
scaleDownFactor = Params.scaleFactor;
orthopxToMeter = Params.orthopxToMeter;
reSampleRate = Params.reSampleRate;
% parameters
cwHeadingThreshold = Params.cwHeadingThreshold; %95 degrees
swHeadingThreshold = Params.swHeadingThreshold; %45 degrees
stoppingThreshold = Params.stoppingThreshold; %speed m/s
walkingThreshold = Params.walkingThreshold; %speed m/s 
cwDistThreshold = Params.cwDistThreshold; %in pixels
cwCrossThreshold = Params.cwCrossThreshold; % approximate width of a lane
decZone = (Params.decZone + 5)/(scaleDownFactor*orthopxToMeter); %10 m radius (in pixels)
roadOrientation = [10, -166, -67, 140]; % in degrees; calculated CCW
roadOrientationOtherDirection = [-170, 14, 113, -40]; % degrees
% initialize variables
isCrossing = false;
lonVelocity = inf;
% hybrid_state = 'None';
% Lane = 'None';
pedHead = tracksData.calcHeading(1);
lonVelocity = inf*ones(size(tracksData.frame));
prob_hybrid_state = [0,0,0,0,0];
prevCWDispSign = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% convert track data to pixels
xCenterPix = (tracksData.xCenter/(scaleDownFactor*orthopxToMeter));
yCenterPix = (tracksData.yCenter/(scaleDownFactor*orthopxToMeter));


% length of this particular pedestrian track    
N_instances = size(tracksData.xCenter,1);
% N_instances = 1;    % need to find only for the last time step
Region = strings(N_instances,1);
hybrid_state = strings(N_instances,1);
Lane = strings(N_instances,1);
cwHeadingState = strings(4, 1); % to check if they are headed towards each of the four crosswalks
approachHeadingState = strings(4,1);
distSW = inf;
cwInd = 0;
% cwInd_new = inf;
swInd = 0;
onRoad = false;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 2) Hybrid state loop    
% if ~flag.outOfPlay
for ii=1:N_instances   % need to find only for the last time step
       % initialize

       cwHeadingState(1:4) = 'Not_Headed';
       cwHeadingCloseCW = 'Not_Headed';
       prevSwInd = swInd;
       prevCwInd = cwInd;
       prevPedHead = pedHead;
       isTransitionPossible = false;
       
       % index based on the current pixel position of pedestrian
       posPixels = int32([xCenterPix(ii), yCenterPix(ii)]);
       %%%%%%%%%%%%%%%%%%%%
       %% 2a)check pedestrian position, heading, and velocity discrete states
       % a) is pedestrian heading towards a crosswalk?
       % heading angle of pedestrians (to reduce noise in the estimation of
       % heading because of noisy position and velocity data)
        y_vel = tracksData.yVelocity(ii);
        y_vel_head = y_vel;
        if ( abs(y_vel) < walkingThreshold )
            y_vel_head = 0;
        end
        x_vel = tracksData.xVelocity(ii);
        x_vel_head = x_vel;
        if ( abs(x_vel) < walkingThreshold )
            x_vel_head = 0;
        end
        % when pedestrian is stopped, maintain the previous heading
        % instead of saying it as zero.
        if x_vel_head~=0 && y_vel_head~=0
            pedHead = atan2(y_vel_head, x_vel_head)*180/pi; 
        end
        % use average heading from two timesteps for crosswalk calculations
%         closeCW_pedHead = mean([pedHead, prevPedHead]);
        closeCW_pedHead = pedHead;
        
        
        
        
       %%%%%%%%%%%%%%%%%%%%%%%%%%%
       % current region of pedestrian
%        Region(ii) = strings;
        if (annotatedImage_enhanced(-posPixels(2), posPixels(1))==200)
           Region(ii) = "Crosswalk_Marked";
           onRoad = true;
        elseif (annotatedImage_enhanced(-posPixels(2), posPixels(1))==100)
           Region(ii) = "Crosswalk_UnMarked";
           onRoad = true;
        elseif (annotatedImage_enhanced(-posPixels(2), posPixels(1))==50)
           Region(ii) = "Road";
           onRoad = true;
        elseif (annotatedImage_enhanced(-posPixels(2), posPixels(1))==150)
           Region(ii) = "Sidewalk";  
           onRoad = false;
        end 
       %%%%%%%%%%%%%%%%%%%%
        posPixels = ([xCenterPix(ii), yCenterPix(ii)]); % to have the pixels in double format
       % angle between pedestrian and the crosswalks
        ped_cw_angle(1) = atan2(double(cw.center_y(1) - posPixels(2)), double(cw.center_x(1) - posPixels(1)))*180/pi;
        ped_cw_angle(2) = atan2(double(cw.center_y(2) - posPixels(2)), double(cw.center_x(2) - posPixels(1)))*180/pi;  
        ped_cw_angle(3) = atan2(double(cw.center_y(3) - posPixels(2)), double(cw.center_x(3) - posPixels(1)))*180/pi;  
        ped_cw_angle(4) = atan2(double(cw.center_y(4) - posPixels(2)), double(cw.center_x(4) - posPixels(1)))*180/pi;  
       %%%%%%%%%%%%%%%%%%%%
       % distance between pedestrian and crosswalk (in pixels)
       dist_cw1 = sqrt(double(cw.center_x(1) - posPixels(1))^2 + double(cw.center_y(1) - posPixels(2))^2);
       dist_cw2 = sqrt(double(cw.center_x(2) - posPixels(1))^2 + double(cw.center_y(2) - posPixels(2))^2); 
       dist_cw3 = sqrt(double(cw.center_x(3) - posPixels(1))^2 + double(cw.center_y(3) - posPixels(2))^2);
       dist_cw4 = sqrt(double(cw.center_x(4) - posPixels(1))^2 + double(cw.center_y(4) - posPixels(2))^2);
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
       
%      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       %% Pedestrian lane calculation
       %%%%%%%%%%%%%%%%%
       % Displacement to the sidewalk (near the crosswalk)
       dispPedSW(1,1:2) = (resetStates.approach.goal(1,:) - double(posPixels));
       dispPedSW(1,3:4) = (resetStates.approach.goal(2,:) - double(posPixels));
       dispPedSW(2,1:2) = (resetStates.approach.goal(3,:) - double(posPixels));
       dispPedSW(2,3:4) = (resetStates.approach.goal(4,:) - double(posPixels));
       dispPedSW(3,1:2) = (resetStates.approach.goal(5,:) - double(posPixels));
       dispPedSW(3,3:4) = (resetStates.approach.goal(6,:) - double(posPixels));
       dispPedSW(4,1:2) = (resetStates.approach.goal(7,:) - double(posPixels));
       dispPedSW(4,3:4) = (resetStates.approach.goal(8,:) - double(posPixels));      
       % distance to the sidewalk
       distPedLane(1,1) = norm(dispPedSW(1,1:2));
       distPedLane(2,1) = norm(dispPedSW(1,3:4));
       distPedLane(3,1) = norm(dispPedSW(2,1:2));
       distPedLane(4,1) = norm(dispPedSW(2,3:4));
       distPedLane(5,1) = norm(dispPedSW(3,1:2));
       distPedLane(6,1) = norm(dispPedSW(3,3:4));
       distPedLane(7,1) = norm(dispPedSW(4,1:2));
       distPedLane(8,1) = norm(dispPedSW(4,3:4));
       % displacement to the walkaway goals
       dispPedSW_walkaway(1,1:2) = (resetStates.walkaway.erCw1Final(1,:) - double(posPixels));
       dispPedSW_walkaway(1,3:4) = (resetStates.walkaway.goal(2,:) - double(posPixels));
       dispPedSW_walkaway(2,1:2) = (resetStates.walkaway.goal(3,:) - double(posPixels));
       dispPedSW_walkaway(2,3:4) = (resetStates.walkaway.goal(4,:) - double(posPixels));
       dispPedSW_walkaway(3,1:2) = (resetStates.walkaway.goal(5,:) - double(posPixels));
       dispPedSW_walkaway(3,3:4) = (resetStates.walkaway.goal(6,:) - double(posPixels));
       dispPedSW_walkaway(4,1:2) = (resetStates.walkaway.goal(7,:) - double(posPixels));
       dispPedSW_walkaway(4,3:4) = (resetStates.walkaway.goal(8,:) - double(posPixels)); 
       % displacement to the walkaway goals
       headingPedSW_walkaway(1,1) = atan2(dispPedSW_walkaway(1,2),dispPedSW_walkaway(1,1) ) * 180/pi;
       headingPedSW_walkaway(2) =  atan2(dispPedSW_walkaway(1,4),dispPedSW_walkaway(1,3) ) * 180/pi;
       headingPedSW_walkaway(3) =  atan2(dispPedSW_walkaway(2,2),dispPedSW_walkaway(2,1) ) * 180/pi;
       headingPedSW_walkaway(4) =  atan2(dispPedSW_walkaway(2,4),dispPedSW_walkaway(2,3) ) * 180/pi;
       headingPedSW_walkaway(5) =  atan2(dispPedSW_walkaway(3,2),dispPedSW_walkaway(3,1) ) * 180/pi;
       headingPedSW_walkaway(6) =  atan2(dispPedSW_walkaway(3,4),dispPedSW_walkaway(3,3) ) * 180/pi;
       headingPedSW_walkaway(7) =  atan2(dispPedSW_walkaway(4,2),dispPedSW_walkaway(4,1) ) * 180/pi;
       headingPedSW_walkaway(8,1) =  atan2(dispPedSW_walkaway(4,4),dispPedSW_walkaway(4,3) ) * 180/pi;
       % check closest SW
       % a)
%        [sortedSWdist_right, sortedSWindex_right] = sort(distPedLane(1:4));
%        [sortedSWdist_left, sortedSWindex_left] = sort(distPedLane(5:8));   
%        if sortedSWindex_right(1)==sortedSWindex_left(1)
%            swInd = sortedSWindex_right(1);
%            if sortedSWdist_right(1) < sortedSWdist_left(1)
%                Lane(ii) = 'Right';
%            else
%                Lane(ii) = 'Left';
%            end
%        end
       %%%%%%%%%%%%%%%%%%%%%%%%%%%
       % b) alternate
       [sortedSWdist, sortedSWindex] = sort(distPedLane);
       swInd_temp = sortedSWindex(1);
       swInd = ceil(swInd_temp/2);
       if swInd==0
           swInd=4;
       end
       if mod(swInd_temp,2) ~= 0
           Lane(ii) = 'Right';
       else
           Lane(ii) = 'Left';
%        elseif ii>1
%            Lane(ii) = Lane(ii-1);
       end
       
       % check if this is a feasible SW lane
       isSWTransPossible = false;
       sw_sort_ind = 0;
       if ii>1
           while(sw_sort_ind<8 && ~isSWTransPossible)
               % initialize
               sw_sort_ind = sw_sort_ind + 1;
               swInd_temp = sortedSWindex(sw_sort_ind);
               swInd = ceil(swInd_temp/2);
               % sidewalk
%                if swInd==0
%                    swInd=4;
%                end
               % sidewalk lane calculation
               if  mod(swInd_temp,2) ~= 0
                   Lane(ii) = 'Right';
               elseif ~onRoad
                   Lane(ii) = 'Left';
               else
                   Lane(ii) = Lane(ii-1);
               end
               
               % check
                if ( (prevSwInd==1 && strcmp(Lane(ii-1),'Right')) && ( (swInd==1 && strcmp(Lane(ii),'Right')) || (swInd==1 && strcmp(Lane(ii),'Left')) || (swInd==4 && strcmp(Lane(ii),'Left')) ) )
                       isSWTransPossible = true;
                elseif ( (prevSwInd==1 && strcmp(Lane(ii-1),'Left') ) && ( (swInd==1 && strcmp(Lane(ii),'Left')) || (swInd==1 && strcmp(Lane(ii),'Right')) || (swInd==3 && strcmp(Lane(ii),'Right')) ) )
                       isSWTransPossible = true;   
                elseif ( (prevSwInd==2 && strcmp(Lane(ii-1),'Right') ) && ( (swInd==2 && strcmp(Lane(ii),'Right')) || (swInd==2 && strcmp(Lane(ii),'Left')) || (swInd==3 && strcmp(Lane(ii),'Left')) ) )
                       isSWTransPossible = true; 
                elseif ( (prevSwInd==2 && strcmp(Lane(ii-1),'Left') ) && ( (swInd==2 && strcmp(Lane(ii),'Left')) || (swInd==2 && strcmp(Lane(ii),'Right')) || (swInd==4 && strcmp(Lane(ii),'Right')) ) )
                       isSWTransPossible = true; 
                elseif ( (prevSwInd==3 && strcmp(Lane(ii-1),'Right') ) && ( (swInd==3 && strcmp(Lane(ii),'Right')) || (swInd==3 && strcmp(Lane(ii),'Left')) || (swInd==1 && strcmp(Lane(ii),'Left')) ) )
                       isSWTransPossible = true;
                elseif ( (prevSwInd==3 && strcmp(Lane(ii-1),'Left') ) && ( (swInd==3 && strcmp(Lane(ii),'Left')) || (swInd==3 && strcmp(Lane(ii),'Right')) || (swInd==2 && strcmp(Lane(ii),'Right')) ) )
                       isSWTransPossible = true; 
                elseif ( (prevSwInd==4 && strcmp(Lane(ii-1),'Right') ) && ( (swInd==4 && strcmp(Lane(ii),'Right')) || (swInd==4 && strcmp(Lane(ii),'Left')) || (swInd==2 && strcmp(Lane(ii),'Left')) ) )
                       isSWTransPossible = true; 
                elseif ( (prevSwInd==4 && strcmp(Lane(ii-1),'Left') ) && ( (swInd==4 && strcmp(Lane(ii),'Left')) || (swInd==4 && strcmp(Lane(ii),'Right')) || (swInd==1 && strcmp(Lane(ii),'Right')) ) )
                       isSWTransPossible = true; 
% %                 elseif strcmp(Lane(ii), '')
%                      swInd = 0; % Not on any sidewalk 
                else
                     % transition not feasible; maintain the previous
                     % sidewalk lane
                     isSWTransPossible = false;
                     swInd = prevSwInd;
                     Lane(ii) = Lane(ii-1);                  
                end
            end
       end       
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
       
       % check heading:
       % when the heading angle is within +/- heading_threshold and pedestrian is close to a crosswalk, 
       % then pedestrian is approaching one of the crosswalks   
       if  ( ( abs(ped_cw_angle(1) - closeCW_pedHead) <= cwHeadingThreshold && dist_cw1 < cwDistThreshold ) || (dist_cw1 < cwCrossThreshold && onRoad) )
            cwHeadingState(1) = 'Headed';
       end
       if ( ( abs(ped_cw_angle(2) - closeCW_pedHead) <= cwHeadingThreshold && dist_cw2 < cwDistThreshold ) || (dist_cw2 < cwCrossThreshold && onRoad))
            cwHeadingState(2) = 'Headed';
       end
       if ( ( abs(ped_cw_angle(3) - closeCW_pedHead) <= cwHeadingThreshold && dist_cw3 < cwDistThreshold ) || (dist_cw3 < cwCrossThreshold && onRoad))
            cwHeadingState(3) = 'Headed';   
       end
       if ( ( abs(ped_cw_angle(4) - closeCW_pedHead) <= cwHeadingThreshold && dist_cw4 < cwDistThreshold ) || (dist_cw4 < cwCrossThreshold && onRoad) )
            cwHeadingState(4) = 'Headed';
       end

       
  
       
       
%        % method using road orientations
%        if  ( ( roadOrientation(1) - pedHead) <= 170 && dist_cw1 < cwDistThreshold ) || (dist_cw1 < cwCrossThreshold && onRoad) 
%             cwHeadingState(1) = 'Headed';
%        end
%        if ( ( roadOrientation(2) - pedHead) <= 170 && dist_cw2 < cwDistThreshold ) || (dist_cw2 < cwCrossThreshold && onRoad) 
%             cwHeadingState(2) = 'Headed';
%        end
%        if ( ( roadOrientation(3) - pedHead) <= 170 && dist_cw3 < cwDistThreshold ) || (dist_cw3 < cwCrossThreshold && onRoad)
%             cwHeadingState(3) = 'Headed';   
%        end
%        if ( ( roadOrientation(4) - pedHead) <= 170 && dist_cw4 < cwDistThreshold ) || (dist_cw4 < cwCrossThreshold && onRoad) 
%             cwHeadingState(4) = 'Headed';
%        end



       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       %% closest CW calculation
       [sortedCWdist, sortedcwIndex] = sort(dist_cw_temp);
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
               if ii>1 && prevCwInd~=cwInd && swInd~=inf
                    if ( (swInd==1 && strcmp(Lane(ii),'Right')) && (cwInd==1 || cwInd==4) )
                           isTransitionPossible = true;
                    elseif ( (swInd==1 && strcmp(Lane(ii),'Left') ) && ( cwInd==1 || cwInd==3) )
                           isTransitionPossible = true;   
                    elseif ( (swInd==2 && strcmp(Lane(ii),'Right') ) && ( cwInd==2 || cwInd==3) )
                           isTransitionPossible = true; 
                    elseif ( (swInd==2 && strcmp(Lane(ii),'Left') ) && ( cwInd==2 || cwInd==4) )
                           isTransitionPossible = true; 
                    elseif ( (swInd==3 && strcmp(Lane(ii),'Right') ) && ( cwInd==3 || cwInd==1) )
                           isTransitionPossible = true;
                    elseif ( (swInd==3 && strcmp(Lane(ii),'Left') ) && ( cwInd==3 || cwInd==2) )
                           isTransitionPossible = true; 
                    elseif ( (swInd==4 && strcmp(Lane(ii),'Right') ) && ( cwInd==4 || cwInd==2) )
                           isTransitionPossible = true; 
                    elseif ( (swInd==4 && strcmp(Lane(ii),'Left') ) && ( cwInd==4 || cwInd==1) )
                           isTransitionPossible = true; 
                    else
                         isTransitionPossible = false; 
                    end
               % first time step or first close CW calculation
               elseif (ii==1 || prevCwInd==inf || prevCwInd==cwInd)
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
       % update th cw distance
       distSW = dist_cw_temp(cwInd);
       
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       % longitudinal velocity (need to know closest CW to calculate this)
       if cwInd~=0 && cwInd~=inf
           theta = cw.theta(cwInd);
           rot = [cosd(theta), -sind(theta); sind(theta), cosd(theta)];
           velRot = rot*[x_vel; y_vel];
           lonVelocity = velRot(1);
       end
      
       %%%%%%%%%%%%%%%%%%%%
       % b) is pedestrian walking?
       ped_vel = sqrt(tracksData.xVelocity(ii)^2 + tracksData.yVelocity(ii)^2);
       if ped_vel < stoppingThreshold
           walk_state = 'Stopping';
       else
           walk_state = 'Walking';
       end
       
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       % check heading based on heading and road orientation
       if cwInd==1
           if (pedHead >= roadOrientation(1) - 80 && pedHead <= roadOrientation(1) + 80)
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
           if (pedHead >= roadOrientation(2) - 80 && pedHead <= roadOrientation(2) + 80)
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
          if (pedHead >= roadOrientation(3) - 80 && pedHead <= roadOrientation(3) + 80)
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
           if (pedHead >= roadOrientation(4) - 80 && pedHead <= roadOrientation(4) + 80)
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
       latDispCwPixels = abs(dispRot_cw(cwInd,2)) - cw.centerLatOffset(cwInd);
       

%        if  (pedHead >= roadOrientation(cwInd) - 90 && pedHead <= roadOrientation(cwInd) + 90)
%             approachHeadingState = 'Headed';
%             longDispDWApproach = dispRot_cw(cwInd,1);
%        else
%             approachHeadingState = 'Not_Headed';
%             longDispDWApproach = -dispRot_cw(cwInd,1);
%        end

       
       
       
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% 2b) Hybrid state update    
    % run the hybrid state update only when needed (i.e. during update
    % stage, and when the hybrid state needs to be identified from the
    % continuous state predictions)
    if ~flag.hybridStatePred(trackletNo)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % conditions for Hybrid State of Pedestrians
        % (1) approach: if pedestrian is on sidewalk and headed towards a
        % crosswalk (even when velocity is low, if outside decision zone,
        % considered as approach only)
            if ( strcmp(Region(ii),"Sidewalk") && ( strcmp(walk_state, 'Walking') || (strcmp(walk_state, 'Stopping') && distSW > decZone) )...
              && ( strcmp(cwHeadingCloseCW,'Headed') || (ii==1 && strcmp(approachHeadingState,'Headed') ) ) )
               hybrid_state(ii) = 'Approach';
               prob_hybrid_state = [1, 0, 0, 0, 0];
%                annotatedImage_enhanced_w_tracks(-pixel_pos(2), pixel_pos(1)) = 60;
            
        % (2) wait: if pedestrian is on sidewalk near a crosswalk and is walking very slowly. 
        % Note: Using absolute distance to center of crosswalk for now.
%             elseif ( strcmp(Region(ii),"Sidewalk") && strcmp(cwHeadingCloseCW,'Headed') && strcmp(walk_state, 'Stopping') && (distSW <= decZone) )
            elseif ( strcmp(Region(ii),"Sidewalk") && strcmp(walk_state, 'Stopping') && (distSW <= decZone) )
               hybrid_state(ii) = 'Wait'; 
               prob_hybrid_state = [0, 1, 0, 0, 0];
%                annotatedImage_enhanced_w_tracks(-pixel_pos(2), pixel_pos(1)) = 0;
               isCrossing = true;
            

        % (3) cross: if pedestrian is on the road (note if they are crossing or
        % jaywalking)
            elseif (strcmp(Region(ii),"Crosswalk_Marked") || strcmp(Region(ii),"Crosswalk_UnMarked"))
               hybrid_state(ii) = 'Crossing';
               prob_hybrid_state = [0, 0, 1, 0, 0];
%                annotatedImage_enhanced_w_tracks(-pixel_pos(2), pixel_pos(1)) = 255;
               isCrossing = true;
                
            elseif (strcmp(Region(ii),"Road") )
               hybrid_state(ii) = 'Jaywalking'; % neglect Jaywalking for now
               prob_hybrid_state = [0, 0, 0, 1, 0];
%                annotatedImage_enhanced_w_tracks(-pixel_pos(2), pixel_pos(1)) = 255;
               isCrossing = true;
            
        % (4) walkaway: if pedestrian just crossed the street and is on the
        % sidewalk
%             elseif ( strcmp(Region(ii),"Sidewalk") && strcmp(cwHeadingCloseCW,'Not_Headed') && strcmp(walk_state, 'Walking') )
%               elseif ( strcmp(Region(ii),"Sidewalk") && strcmp(cwHeadingCloseCW,'Not_Headed') )
            elseif ( (strcmp(Region(ii),"Sidewalk") || strcmp(Region(ii),'')) && ( sign(dispRot_cw(cwInd,1))~= prevCWDispSign || strcmp(approachHeadingState,'Not_Headed') ) )
               hybrid_state(ii) = 'Walk_away'; 
               prob_hybrid_state = [0, 0, 0, 0, 1];
%                annotatedImage_enhanced_w_tracks(-pixel_pos(2), pixel_pos(1)) = 120;

            elseif ii>1
                hybrid_state(ii) = hybrid_state(ii-1);
            end
            
            
            %% debug
            if (strcmp(walk_state, 'Stopping') && ~strcmp(hybrid_state(ii),'Wait') )
                x=1;
            end
            
            if (strcmp(walk_state, 'Stopping') && strcmp(hybrid_state(ii),'Wait') )
                x=1;
            end
            
            if (strcmp(hybrid_state(ii), 'Approach') && strcmp(approachHeadingState,'Not_Headed') && strcmp(cwHeadingCloseCW,'Not_Headed'))
                x=1;
            end

    end % end of hybrid update (not prediction) loop
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    %% update variables  
    prevCWDispSign = sign(dispRot_cw(cwInd,1));
    tracksData.ProbHybridState(ii,:) = prob_hybrid_state;  
    tracksData.isCrossing(ii) = isCrossing; 
%     tracksData.distCW(ii) = dist_cw;
    tracksData.closestCW(ii) = cwInd;
    tracksData.calcLonVelocity(ii,1) = lonVelocity;
%     tracksData.calcHeading(ii) = pedHead;
    tracksData.swInd(ii,1) = swInd;
    tracksData.latDispPedCw(ii,1) = latDispCwPixels*(scaleDownFactor*orthopxToMeter);
    tracksData.longDispPedCw(ii,1) = longDispPedCwPixels*(scaleDownFactor*orthopxToMeter);

   
    % plot the tracks (according to hybrid states) on the enhanced image    
    % annotatedImage_enhanced(swInd_right, swInd_left) = 255;

%     % update wait time; wait time initialized to previous time step
%     % wait time in 'updatePedContStates'
%     if (strcmp(tracksData.HybridState(ii), 'Wait') )
%         if tracksData.waitTimeSteps(ii)==-1 %i.e. no wait before this
%             tracksData.waitTimeSteps(ii) = 0; % wait starts
%         else
%             tracksData.waitTimeSteps(ii) = tracksData.waitTimeSteps(ii) + reSampleRate;
%         end
%     end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% end


end

% % update region
tracksData.Region = Region;
tracksData.Lane = Lane;
tracksData.HybridState = hybrid_state;
% tracksData.xCenterPix = xCenterPix;
% tracksData.yCenterPix = yCenterPix;
x=1;


% %% debug: check for zeno close CW
% 
% diff_closeCw = diff(tracksData.closestCW);
% ind = find(diff_closeCw~=0);
% diff_ind = diff(ind);
% diff_ind_2 = diff_ind;
% diff_ind_2(diff_ind_2 > 5) = [];
% if ~isempty(diff_ind_2)
%     x=1;
% end
% 
% %% debug: check for zeno hybrid state
% 
% diff_hybrid = [];
% for ii = 2:length(tracksData.frame)
%    if (~strcmp(tracksData.HybridState(ii), tracksData.HybridState(ii-1)) && ii>5 && ~(strcmp(tracksData.HybridState(ii-1),'Wait')) )
%        diff_hybrid = [diff_hybrid; ii];
%    end
% end
% diff_ind_hybrid = diff(diff_hybrid);
% diff_ind_hybrid_2 = diff_ind_hybrid;
% diff_ind_hybrid_2(diff_ind_hybrid_2 > 5) = [];
% if ~isempty(diff_ind_hybrid_2)
%     x=1;
% end



end

