%Note: make sure the enhanced annotataed background image is the input to
%this function
function [tracksData] = hybridStateCopy_v2(tracksData, cw, flag, annotatedImage_enhanced, Params, trackletNo, resetStates)
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
decZone = Params.decZone; %8 m radius (in pixels)
roadOrientation = [10, -166, -67, 140]; % in degrees; calculated CCW
roadOrientationOtherDirection = [-170, 14, 113, -40]; % degrees
% initialize variables
isCrossing = false;
lonVelocity = inf;
hybrid_state = 'None';
Lane = 'None';
pedHead = tracksData.calcHeading(1);
lonVelocity = inf*ones(size(tracksData));
prob_hybrid_state = [0,0,0,0,0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% convert track data to pixels
xCenterPix = (tracksData.xCenter/(scaleDownFactor*orthopxToMeter));
yCenterPix = (tracksData.yCenter/(scaleDownFactor*orthopxToMeter));


% length of this particular pedestrian track    
N_instances = size(tracksData.xCenter,1);
% N_instances = 1;    % need to find only for the last time step
Region = strings(N_instances,1);
Lane = strings(N_instances,1);
cwHeadingState = strings(4, 1); % to check if they are headed towards each of the four crosswalks
dist_cw = inf;
cwInd = inf;
% prevCwInd = inf;
% cwInd_new = inf;
swInd = inf;
onRoad = false;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 2) Hybrid state loop    
% if ~flag.outOfPlay
for ii=1:N_instances   % need to find only for the last time step
       % initialize
       dist_cw_temp = [inf, inf, inf, inf];  
       cwHeadingState(1:4) = 'Not_Headed';
       cwHeadingCloseCW = 'Not_Headed';
       
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
        %%%%%%%%%%%%%%%%%%%%
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
       %%%%%%%%%%%%%%%%%%%%%%%%
%        dispRot_cw1 = [cosd(cw.theta(1)), -sind(cw.theta(1)); sind(cw.theta(1)), cosd(cw.theta(1))] * ([cw.center_x(1),cw.center_y(1)] - posPixels)';
%        dispRot_cw2 = [cosd(cw.theta(2)), -sind(cw.theta(2)); sind(cw.theta(2)), cosd(cw.theta(2))] * ([cw.center_x(2),cw.center_y(2)] - posPixels)'; 
%        dispRot_cw3 = [cosd(cw.theta(3)), -sind(cw.theta(3)); sind(cw.theta(3)), cosd(cw.theta(3))] * ([cw.center_x(3),cw.center_y(3)] - posPixels)';
%        dispRot_cw4 = [cosd(cw.theta(4)), -sind(cw.theta(4)); sind(cw.theta(4)), cosd(cw.theta(4))] * ([cw.center_x(4),cw.center_y(4)] - posPixels)';
%            
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
       
       %%%%%%%%%%%%%%%%%%%%
       % when the heading angle is within +/- heading_threshold and pedestrian is close to a crosswalk, 
       % then pedestrian is approaching one of the crosswalks   
       if  ( ( abs(ped_cw_angle(1) - pedHead) <= cwHeadingThreshold && dist_cw1 < cwDistThreshold ) || (dist_cw1 < cwCrossThreshold && onRoad) )
            cwHeadingState(1) = 'Headed';
%             dist_cw_temp(1) = dist_cw1;
       end
       if ( ( abs(ped_cw_angle(2) - pedHead) <= cwHeadingThreshold && dist_cw2 < cwDistThreshold ) || (dist_cw2 < cwCrossThreshold && onRoad))
            cwHeadingState(2) = 'Headed';
%             dist_cw_temp(2) = dist_cw2;
       end
       if ( ( abs(ped_cw_angle(3) - pedHead) <= cwHeadingThreshold && dist_cw3 < cwDistThreshold ) || (dist_cw3 < cwCrossThreshold && onRoad))
            cwHeadingState(3) = 'Headed';
%             dist_cw_temp(3) = dist_cw3;   
       end
       if ( ( abs(ped_cw_angle(4) - pedHead) <= cwHeadingThreshold && dist_cw4 < cwDistThreshold ) || (dist_cw4 < cwCrossThreshold && onRoad) )
            cwHeadingState(4) = 'Headed';
%             dist_cw_temp(4) = dist_cw4;
       end
       %%%%%%%%%%%%%%%%%%%%
       % Pedestrian lane calculation
       %%%%%%%%%%%%%%%%%
       % Displacement to the sidewalk
       dispPedSW(1,1:2) = (resetStates.approach.goal(1,:) - double(posPixels));
       dispPedSW(1,3:4) = (resetStates.approach.goal(2,:) - double(posPixels));
       dispPedSW(2,1:2) = (resetStates.approach.goal(3,:) - double(posPixels));
       dispPedSW(2,3:4) = (resetStates.approach.goal(4,:) - double(posPixels));
       dispPedSW(3,1:2) = (resetStates.approach.goal(5,:) - double(posPixels));
       dispPedSW(3,3:4) = (resetStates.approach.goal(6,:) - double(posPixels));
       dispPedSW(4,1:2) = (resetStates.approach.goal(7,:) - double(posPixels));
       dispPedSW(4,3:4) = (resetStates.approach.goal(8,:) - double(posPixels));      
       % distance
       distPedLane(1,1) = norm(dispPedSW(1,1:2));
       distPedLane(1,2) = norm(dispPedSW(1,3:4));
       distPedLane(2,1) = norm(dispPedSW(2,1:2));
       distPedLane(2,2) = norm(dispPedSW(2,3:4));
       distPedLane(3,1) = norm(dispPedSW(3,1:2));
       distPedLane(3,2) = norm(dispPedSW(3,3:4));
       distPedLane(4,1) = norm(dispPedSW(4,1:2));
       distPedLane(4,2) = norm(dispPedSW(4,3:4));
       % closest Sidewalk
       [sortedRightSWdist, sortedRightSWindex] = sort(distPedLane(:,1));
       [sortedLeftSWdist, sortedLeftSWindex] = sort(distPedLane(:,2));
       swInd_right = sortedRightSWindex(1);
       swInd_left = sortedLeftSWindex(1);
%        [value,index] = min(distPedLane);

       % check if the closest sw satisfies the margin, else update the close
       % sw
       if swInd_right~=swInd_left
            cw_ii = 1;
            while(cw_ii<4)
                if sortedRightSWdist(cw_ii) - sortedRightSWdist(cw_ii+1) > -10
                    swInd_right = sortedRightSWindex(cw_ii+1);
                end
                cw_ii=cw_ii+1;
            end           
            cw_ii = 1;
            while(cw_ii<4)
                if sortedLeftSWdist(cw_ii) - sortedLeftSWdist(cw_ii+1) > -10
                    swInd_left = sortedLeftSWindex(cw_ii+1);
                end
                cw_ii=cw_ii+1;
            end           
       end
        
        % the same direction (east, west, north, south) is the closest for both left and right
        % sidewalks
        if swInd_right==swInd_left
            % if the right sidewalk is closer than the left sidewalk
            if sortedRightSWdist(1) < sortedLeftSWdist(1)
                Lane(ii) = "Right";
            else
                Lane(ii) = "Left";  
            end
            swInd = swInd_right;
        else
           % which sidewalk is the pedestrian heading towards
           if ii>2
                if ((abs(pedHead - roadOrientation(swInd_right) ) <  swHeadingThreshold || abs(pedHead - roadOrientationOtherDirection(swInd_right) ) <  swHeadingThreshold )...
                     && (sortedRightSWdist(1) - distPedLane(swInd,1) < -10 )  )   
                    Lane(ii) = "Right";
                    swInd = sortedRightSWswInd_right;
                elseif ((abs(pedHead - roadOrientation(swInd_left) ) <  swHeadingThreshold || abs(pedHead - roadOrientationOtherDirection(swInd_left) ) <  swHeadingThreshold )...
                        && (sortedLeftSWdist(1) - distPedLane(swInd,2) < -10 )  )  
                    Lane(ii) = "Left"; 
                    swInd = sortedLeftSWswInd_left;
                else
                    x=1;
                end
           else
                if ((abs(pedHead - roadOrientation(swInd_right) ) <  swHeadingThreshold || abs(pedHead - roadOrientationOtherDirection(swInd_right) ) <  swHeadingThreshold ) )   
                    Lane(ii) = "Right";
                    swInd = swInd_right;
                elseif ((abs(pedHead - roadOrientation(swInd_left) ) <  swHeadingThreshold || abs(pedHead - roadOrientationOtherDirection(swInd_left) ) <  swHeadingThreshold ) )  
                    Lane(ii) = "Left"; 
                    swInd = swInd_left;
                else
                    x=1;
                end
               
           end
             %%%%%%%%%%%%
        end
       
        if ii==71
            x=1;
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

       % dont update the close CW if the pedestrian is on the road
       if ~onRoad       
%            % distance to the closest crosswalk when headed towards that
%            % crosswalk
%            [dist_cw, cwInd] = min(dist_cw_temp);
%            if dist_cw==inf
%                cwInd = inf;
%                cwInd_new = inf;
%            end   
            cwInd = swInd;
       end
       %%%%%%%%%%%%%%%%%%%%
       
%        % check if cwInd and swInd matches
%        if (cwInd~=inf && cwInd~=0) && swInd~=inf
%            if (swInd==1 && strcmp(Lane(ii),'Right') )
%                if ~(cwInd==1 || cwInd==4)
%                    if (dist_cw_temp(1) <  dist_cw_temp(4))
%                        cwInd_new = 1;
%                    else
%                        cwInd_new = 4;
%                    end
%                else
%                    cwInd_new = cwInd;
%                end
%                
%            elseif  (swInd==1 && strcmp(Lane(ii),'Left') )
%                if ~(cwInd==1 || cwInd==3) 
%                    if dist_cw1 <  dist_cw3
%                        cwInd_new = 1;
%                    else
%                        cwInd_new = 3;
%                    end
%                else
%                    cwInd_new = cwInd;
%                end
%            
%            elseif  (swInd==2 && strcmp(Lane(ii),'Right') ) 
%                if ~(cwInd==2 || cwInd==3) 
%                    if dist_cw2 <  dist_cw3
%                        cwInd_new = 2;
%                    else
%                        cwInd_new = 3;
%                    end
%                else
%                    cwInd_new = cwInd;
%                end
%                
%            elseif  (swInd==2 && strcmp(Lane(ii),'Left') ) 
%                if ~(cwInd==2 || cwInd==4) 
%                    if dist_cw2 <  dist_cw4
%                        cwInd_new = 2;
%                    else
%                        cwInd_new = 4;
%                    end
%                else
%                    cwInd_new = cwInd;
%                end
%                
%            elseif  (swInd==3 && strcmp(Lane(ii),'Right') ) 
%                if ~(cwInd==3 || cwInd==1) 
%                    if dist_cw1 <  dist_cw3
%                        cwInd_new = 1;
%                    else
%                        cwInd_new = 3;
%                    end
%                else
%                    cwInd_new = cwInd;
%                end
%               
%            elseif  (swInd==3 && strcmp(Lane(ii),'Left') ) 
%                if ~(cwInd==3 || cwInd==2) 
%                    if dist_cw2 <  dist_cw3
%                        cwInd_new = 2;
%                    else
%                        cwInd_new = 3;
%                    end
%                else
%                    cwInd_new = cwInd;
%                end
%                
%            elseif  (swInd==4 && strcmp(Lane(ii),'Right') ) 
%                if (cwInd==4 || cwInd==2) 
%                    if dist_cw2 <  dist_cw4
%                        cwInd_new = 2;
%                    else
%                        cwInd_new = 4;
%                    end
%                else
%                    cwInd_new = cwInd;
%                end
%                
%            elseif  (swInd==4 && strcmp(Lane(ii),'Left') ) 
%                if (cwInd==4 || cwInd==1) 
%                    if dist_cw1 <  dist_cw4
%                        cwInd_new = 1;
%                    else
%                        cwInd_new = 4;
%                    end
%                else
%                    cwInd_new = cwInd;
%                end
%                               
%            end
%        end
       
       %%%%%%%%%%%%%%%%%%%%%%5
       % longitudinal velocity (need to know closest CW to calculate this)
       if cwInd~=0 && cwInd~=inf
           theta = cw.theta(cwInd);
           rot = [cosd(theta), -sind(theta); sind(theta), cosd(theta)];
           velRot = rot*[x_vel; y_vel];
           lonVelocity = velRot(1);
           cwHeadingCloseCW = cwHeadingState(cwInd);
       end
      
       %%%%%%%%%%%%%%%%%%%%
       % b) is pedestrian walking?
       ped_vel = sqrt(tracksData.xVelocity(ii)^2 + tracksData.yVelocity(ii)^2);
       if ped_vel < stoppingThreshold
           walk_state = 'Stopping';
       else
           walk_state = 'Walking';
       end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% 2b) Hybrid state update    
    % run the hybrid state update only when needed (i.e. during update
    % stage, and when the hybrid state needs to be identified from the
    % continuous state predictions)
    if ~flag.hybridStatePred(trackletNo)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % conditions for Hybrid State of Pedestrians
        % (1) approach: if pedestrian is on sidewalk and headed towards a crosswalk
            if ( strcmp(Region(ii),"Sidewalk") && strcmp(cwHeadingCloseCW,'Headed') && strcmp(walk_state, 'Walking') )
               hybrid_state = 'Approach';
               prob_hybrid_state = [1, 0, 0, 0, 0];
%                annotatedImage_enhanced_w_tracks(-pixel_pos(2), pixel_pos(1)) = 60;
            end

        % (2) wait: if pedestrian is on sidewalk near a crosswalk and is walking very slowly. 
        % Note: Using absolute distance to center of crosswalk for now.
            if ( strcmp(Region(ii),"Sidewalk") && strcmp(cwHeadingCloseCW,'Headed') && strcmp(walk_state, 'Stopping') && (dist_cw < decZone) )
               hybrid_state = 'Wait'; 
               prob_hybrid_state = [0, 1, 0, 0, 0];
%                annotatedImage_enhanced_w_tracks(-pixel_pos(2), pixel_pos(1)) = 0;
               isCrossing = true;
            end

        % (3) cross: if pedestrian is on the road (note if they are crossing or
        % jaywalking)
            if (strcmp(Region(ii),"Crosswalk_Marked") || strcmp(Region(ii),"Crosswalk_UnMarked") )
               hybrid_state = 'Crossing';
               prob_hybrid_state = [0, 0, 1, 0, 0];
%                annotatedImage_enhanced_w_tracks(-pixel_pos(2), pixel_pos(1)) = 255;
               isCrossing = true;
            end    
            if (strcmp(Region(ii),"Road") )
%                hybrid_state = 'Jaywalking'; % neglect Jaywalking for now
                hybrid_state = 'Crossing';
               prob_hybrid_state = [0, 0, 0, 1, 0];
%                annotatedImage_enhanced_w_tracks(-pixel_pos(2), pixel_pos(1)) = 255;
               isCrossing = true;
            end

        % (4) walkaway: if pedestrian just crossed the street and is on the
        % sidewalk
            if ( strcmp(Region(ii),"Sidewalk") && strcmp(cwHeadingCloseCW,'Not_Headed') && strcmp(walk_state, 'Walking') )
               hybrid_state = 'Walk_away'; 
               prob_hybrid_state = [0, 0, 0, 0, 1];
%                annotatedImage_enhanced_w_tracks(-pixel_pos(2), pixel_pos(1)) = 120;
            end

    end % end of hybrid update (not prediction) loop
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %% debug
    if ii>1
        if (strcmp(hybrid_state, 'Approach') &&strcmp(tracksData.HybridState(ii-1,:),'Crossing') && (cwInd==tracksData.closestCW(ii-1)) )
            x=1;
        end
        
        if (strcmp(hybrid_state, 'Approach') &&strcmp(tracksData.HybridState(ii-1,:),'Walkway'))
            x=1;
        end
        
    end
    
    %% update variables
    tracksData.HybridState(ii) = hybrid_state;  
    tracksData.ProbHybridState(ii,:) = prob_hybrid_state;  
%     tracksData.isCrossing(ii) = isCrossing; 
%     tracksData.distCW(ii) = dist_cw;
    tracksData.closestCW(ii) = cwInd;
    tracksData.calcLonVelocity(ii) = lonVelocity;
%     tracksData.calcHeading(ii) = pedHead;
   

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
% tracksData.xCenterPix = xCenterPix;
% tracksData.yCenterPix = yCenterPix;

end

