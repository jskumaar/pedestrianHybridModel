%Note: make sure the enhanced annotataed background image is the input to
%this function
function [tracksData] = hybridState(tracksData, cw, flag, annotatedImage_enhanced, Params, trackletNo)
%% 1) setup
% fixed parameters
scaleDownFactor = Params.scaleFactor;
orthopxToMeter = Params.orthopxToMeter;
reSampleRate = Params.reSampleRate;
% parameters
headingThreshold = Params.headingThreshold; %45 degrees
stoppingThreshold = Params.stoppingThreshold; %speed m/s
walkingThreshold = Params.walkingThreshold; %speed m/s 
cwDistThreshold = Params.cwDistThreshold; %in pixels
cwCrossThreshold = Params.cwCrossThreshold; % approximate width of a lane
decZone = Params.decZone; %8 m radius (in pixels)
% initialize variables
% isCrossing = false;
lonVelocity = inf;
cwInd = tracksData.closestCW(end);
% initialize the hybrid state
% tracksData.HybridState(end) = strings;
% hybrid_state = 'None'; 

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
closestCW =  tracksData.closestCW;
% length of this particular pedestrian track    
% N_instances = size(tracksData.xCenter,1);
N_instances = 1;    % need to find only for the last time step
Region = strings(N_instances,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 2) Hybrid state loop    
if ~flag.outOfPlay
%for ii=1:N_instances   % need to find only for the last time step
       % initialize
       dist_cw_temp = [inf, inf, inf, inf];  
       % index based on the current pixel position of pedestrian
       posPixels = int32([xCenterPix(end), yCenterPix(end)]);
       %%%%%%%%%%%%%%%%%%%%
       %% 2a)check pedestrian position, heading, and velocity discrete states
       % a) is pedestrian heading towards a crosswalk?
       % heading angle of pedestrians (to reduce noise in the estimation of
       % heading because of noisy position and velocity data)
        y_vel = tracksData.yVelocity(end);
        if ( abs(y_vel) < walkingThreshold )
            y_vel = 0;
        end
        x_vel = tracksData.xVelocity(end);
        if ( abs(x_vel) < walkingThreshold )
            x_vel = 0;
        end
        % when pedestrian is stopped, maintain the previous heading
        % instead of saying it as zero.
        if x_vel~=0 && y_vel~=0
            pedHead = atan2(y_vel, x_vel)*180/pi; 
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
       % current region of pedestrian
       Region(end) = strings;
        if (annotatedImage_enhanced(-posPixels(2), posPixels(1))==200)
           Region(end) = 'Crosswalk_Marked';
           onRoad = true;
        elseif (annotatedImage_enhanced(-posPixels(2), posPixels(1))==100)
           Region(end) = 'Crosswalk_UnMarked';
           onRoad = true;
        elseif (annotatedImage_enhanced(-posPixels(2), posPixels(1))==50)
           Region(end) = 'Road';
           onRoad = true;
        elseif (annotatedImage_enhanced(-posPixels(2), posPixels(1))==150)
           Region(end) = 'Sidewalk';  
           onRoad = false;
        end 
       %%%%%%%%%%%%%%%%%%%%
       % when the heading angle is within +/- heading_threshold and pedestrian is close to a crosswalk, 
       % then pedestrian is approaching one of the crosswalks
       
       cwHeadingState = 'Not_Headed';
       if  ( ( abs(ped_cw1_angle - pedHead) <= headingThreshold && dist_cw1 < cwDistThreshold ) || (dist_cw1 < cwCrossThreshold && onRoad))
            cwHeadingState = 'Headed';
            dist_cw_temp(1) = dist_cw1;
       end
       if ( ( abs(ped_cw2_angle - pedHead) <= headingThreshold && dist_cw2 < cwDistThreshold ) || (dist_cw2 < cwCrossThreshold && onRoad))
            cwHeadingState = 'Headed';
            dist_cw_temp(2) = dist_cw2;
       end
       if ( ( abs(ped_cw3_angle - pedHead) <= headingThreshold && dist_cw3 < cwDistThreshold ) || (dist_cw3 < cwCrossThreshold && onRoad))
            cwHeadingState = 'Headed';
            dist_cw_temp(3) = dist_cw3;   
       end
       if ( ( abs(ped_cw4_angle - pedHead) <= headingThreshold && dist_cw4 < cwDistThreshold ) || (dist_cw4 < cwCrossThreshold && onRoad) )
            cwHeadingState = 'Headed';
            dist_cw_temp(4) = dist_cw4;
       end
       %%%%%%%%%%%%%%%%%%%%
       % dont update the close CW if the pedestrian is on the road
       if ~onRoad       
           % distance to the closest crosswalk when headed towards that
           % crosswalk
           [dist_cw, cwInd] = min(dist_cw_temp);
       end
       %%%%%%%%%%%%%%%%%%%%
       
       % longitudinal velocity (need to know closest CW to calculate this)
       if cwInd~=0 && cwInd~=inf
           theta = cw.theta(cwInd);
           rot = [cosd(theta), -sind(theta); sind(theta), cosd(theta)];
           velRot = rot*[x_vel; y_vel];
           lonVelocity = velRot(1);
       end
       
       %% debug
       if cwInd==3
           x=1;
       end
       
       if (length(closestCW)>1 && cwInd~=closestCW(end-1))
            x=1;
       end
       
       
       %%%%%%%%%%%%%%%%%%%%
       % b) is pedestrian walking?
       ped_vel = sqrt(tracksData.xVelocity(end)^2 + tracksData.yVelocity(end)^2);
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
            if ( strcmp(Region(end),'Sidewalk') && strcmp(cwHeadingState,'Headed') && strcmp(walk_state, 'Walking') )
               hybrid_state = 'Approach';
%                prob_hybrid_state = [1, 0, 0, 0, 0];
%                annotatedImage_enhanced_w_tracks(-pixel_pos(2), pixel_pos(1)) = 60;
            end

        % (2) wait: if pedestrian is on sidewalk near a crosswalk and is walking very slowly. 
        % Note: Using absolute distance to center of crosswalk for now.
            if ( strcmp(Region(end),'Sidewalk') && strcmp(cwHeadingState,'Headed') && strcmp(walk_state, 'Stopping') && (dist_cw < decZone) )
               hybrid_state = 'Wait'; 
%                prob_hybrid_state = [0, 1, 0, 0, 0];
%                annotatedImage_enhanced_w_tracks(-pixel_pos(2), pixel_pos(1)) = 0;
%                isCrossing = true;
            end

        % (3) cross: if pedestrian is on the road (note if they are crossing or
        % jaywalking)
            if (strcmp(Region(end),'Crosswalk_Marked') || strcmp(Region(end),'Crosswalk_UnMarked') )
               hybrid_state = 'Crossing';
%                prob_hybrid_state = [0, 0, 1, 0, 0];
%                annotatedImage_enhanced_w_tracks(-pixel_pos(2), pixel_pos(1)) = 255;
%                isCrossing = true;
            end    
            if (strcmp(Region(end),'Road') )
%                hybrid_state = 'Jaywalking'; % neglect Jaywalking for now
                hybrid_state = 'Crossing';
%                prob_hybrid_state = [0, 0, 0, 1, 0];
%                annotatedImage_enhanced_w_tracks(-pixel_pos(2), pixel_pos(1)) = 255;
%                isCrossing = true;
            end

        % (4) walkaway: if pedestrian just crossed the street and is on the
        % sidewalk
            if ( strcmp(Region(end),'Sidewalk') && strcmp(cwHeadingState,'Not_Headed') && strcmp(walk_state, 'Walking') )
               hybrid_state = 'Walk_away'; 
%                prob_hybrid_state = [0, 0, 0, 0, 1];
%                annotatedImage_enhanced_w_tracks(-pixel_pos(2), pixel_pos(1)) = 120;
            end

            % update the state variables not relevant for predictions
            tracksData.HybridState(end) = hybrid_state;  
            %tracksData.ProbHybridState(end,:) = prob_hybrid_state;  
            %tracksData.isCrossing(end) = isCrossing; 
            %tracksData.distCW(end) = dist_cw;

    end % end of hybrid update (not prediction) loop
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% update other variables
    tracksData.closestCW(end) = cwInd;
%     tracksData.lonVelocity(end) = lonVelocity;
    %tracksData.calcHeading(end) = pedHead;

    % plot the tracks (according to hybrid states) on the enhanced image    
    % annotatedImage_enhanced(index(1), index(2)) = 255;

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

