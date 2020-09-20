function [tracksData] = hybridState(tracksData, cw, flag, annotatedImage_enhanced, Params, trackletNo)
    %Note:
    %1) make sure the enhanced annotataed background image is the input to
    %this function

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
    isCrossing = false;
    pedHead = 0;
    
    % convert track data to pixels
    % if ~tracksData.xCenterPix
        tracksData.xCenterPix = (tracksData.xCenter/(scaleDownFactor*orthopxToMeter));
        tracksData.yCenterPix = (tracksData.yCenter/(scaleDownFactor*orthopxToMeter));
    % end

    % length of this particular pedestrian track    
    N_instances = size(tracksData,1);

    %% loop starts    
    for ii=1:N_instances 
           % initialize
           dist_cw_temp = [inf, inf, inf, inf];  
           cwInd = 0;
           
           % index based on the current pixel position of pedestrian
           pixelPos = int32([tracksData.xCenterPix(ii), tracksData.yCenterPix(ii)]);

           %% check pedestrian position, heading, and velocity discrete states
           % 2) is pedestrian heading towards a crosswalk?
           % heading angle of pedestrians      
           % ped_head = atan2(tracksData.yVelocity(ii),tracksData.xVelocity(ii))*180/pi;
           % to reduce noise in the estimation of heading because of noisy position and velocity data
            y_vel = tracksData.yVelocity(ii);
            if ( abs(y_vel) < walkingThreshold )
                y_vel = 0;
            end
            x_vel = tracksData.xVelocity(ii);
            if ( abs(x_vel) < walkingThreshold )
                x_vel = 0;
            end
            % when pedestrian is stopped, maintain the previpus heading
            % instead of saying it as zero.
            if x_vel~=0 || y_vel~=0
                pedHead = atan2(y_vel, x_vel)*180/pi; 
            end

           % angle between pedestrian and the crosswalks
            ped_cw1_angle = atan2(double([cw.center_y(1) - pixelPos(2)]), double([cw.center_x(1) - pixelPos(1)]))*180/pi;
            ped_cw2_angle = atan2(double([cw.center_y(2) - pixelPos(2)]), double([cw.center_x(2) - pixelPos(1)]))*180/pi;  
            ped_cw3_angle = atan2(double([cw.center_y(3) - pixelPos(2)]), double([cw.center_x(3) - pixelPos(1)]))*180/pi;  
            ped_cw4_angle = atan2(double([cw.center_y(4) - pixelPos(2)]), double([cw.center_x(4) - pixelPos(1)]))*180/pi;  

           % distance between pedestrian and crosswalk (in pixels)
           dist_cw1 = sqrt(double(cw.center_x(1) - pixelPos(1))^2 + double(cw.center_y(1) - pixelPos(2))^2);
           dist_cw2 = sqrt(double(cw.center_x(2) - pixelPos(1))^2 + double(cw.center_y(2) - pixelPos(2))^2); 
           dist_cw3 = sqrt(double(cw.center_x(3) - pixelPos(1))^2 + double(cw.center_y(3) - pixelPos(2))^2);
           dist_cw4 = sqrt(double(cw.center_x(4) - pixelPos(1))^2 + double(cw.center_y(4) - pixelPos(2))^2);

           % when the heading angle is within +/- heading_threshold and pedestrian is close to a crosswalk, 
           % then pedestrian is approaching one of the crosswalks 
           cwHeadingState = 'Not_Headed';
           if  ( ( abs(ped_cw1_angle - pedHead) <= headingThreshold && dist_cw1 < cwDistThreshold ) || dist_cw1 < cwCrossThreshold )
                cwHeadingState = 'Headed';
                dist_cw_temp(1) = dist_cw1;
           end
           if ( ( abs(ped_cw2_angle - pedHead) <= headingThreshold && dist_cw2 < cwDistThreshold ) || dist_cw2 < cwCrossThreshold )
                cwHeadingState = 'Headed';
                dist_cw_temp(2) = dist_cw2;
           end
           if ( ( abs(ped_cw3_angle - pedHead) <= headingThreshold && dist_cw3 < cwDistThreshold ) || dist_cw3 < cwCrossThreshold )
                cwHeadingState = 'Headed';
                dist_cw_temp(3) = dist_cw3;   
           end
           if ( ( abs(ped_cw4_angle - pedHead) <= headingThreshold && dist_cw4 < cwDistThreshold ) || dist_cw4 < cwCrossThreshold )
                cwHeadingState = 'Headed';
                dist_cw_temp(4) = dist_cw4;
           end

           % distance to the closest crosswalk when headed towards that
           % crosswalk
           [dist_cw, cwInd] = min(dist_cw_temp);
           
           % 3) is pedestrian walking?
           ped_vel = sqrt(tracksData.xVelocity(ii)^2 + tracksData.yVelocity(ii)^2);
           if ped_vel < stoppingThreshold
               walk_state = 'Stopping';
           else
               walk_state = 'Walking';
           end

        %% Hybrid state update    
        % run the hybrid state update only when needed (i.e. during update
        % stage, and when the hybrid state needs to be identified from the
        % continuous state predictions)
        if ~flag.pred(trackletNo)
            % 1) position of pedestrians (make sure the values match with the
            % enhanced values in annotated_plot.m)
            % initialize
            Region(ii) = strings;
            if (annotatedImage_enhanced(-pixelPos(2), pixelPos(1))==200)
               Region(ii) = 'Crosswalk_Marked';
            elseif (annotatedImage_enhanced(-pixelPos(2), pixelPos(1))==100)
               Region(ii) = 'Crosswalk_UnMarked';
            elseif (annotatedImage_enhanced(-pixelPos(2), pixelPos(1))==50)
               Region(ii) = 'Road';
            elseif (annotatedImage_enhanced(-pixelPos(2), pixelPos(1))==150)
               Region(ii) = 'Sidewalk';               
            end 
          
            % initialize the hybrid state
              tracksData.HybridState(ii) = strings;
              hybrid_state = 'None'; 
              prob_hybrid_state = [0, 0, 0, 0, 0];

            % conditions for Hybrid State of Pedestrians
            % (1) approach: if pedestrian is on sidewalk and headed towards a crosswalk
                if ( strcmp(Region(ii),'Sidewalk') & strcmp(cwHeadingState,'Headed') & strcmp(walk_state, 'Walking') )
                   hybrid_state = 'Approach';
                   prob_hybrid_state = [1, 0, 0, 0, 0];
                   %annotatedImage_enhanced_w_tracks(-pixel_pos(2), pixel_pos(1)) = 60;
                end

            % (2) wait: if pedestrian is on sidewalk near a crosswalk and is walking very slowly. 
            % Note: Using absolute distance to center of crosswalk for now.
                if ( strcmp(Region(ii),'Sidewalk') & strcmp(cwHeadingState,'Headed') & strcmp(walk_state, 'Stopping') & (dist_cw < decZone) )
                   hybrid_state = 'Wait'; 
                   prob_hybrid_state = [0, 1, 0, 0, 0];
                   %annotatedImage_enhanced_w_tracks(-pixel_pos(2), pixel_pos(1)) = 0;
                   isCrossing = true;
                end

            % (3) cross: if pedestrian is on the road (note if they are crossing or
            % jaywalking)
                if (strcmp(Region(ii),'Crosswalk_Marked') | strcmp(Region(ii),'Crosswalk_UnMarked') )
                   hybrid_state = 'Crossing';
                   prob_hybrid_state = [0, 0, 1, 0, 0];
                   %annotatedImage_enhanced_w_tracks(-pixel_pos(2), pixel_pos(1)) = 255;
                   isCrossing = true;
                end    
                if (strcmp(Region(ii),'Road') )
                   hybrid_state = 'Jaywalking'; 
                   prob_hybrid_state = [0, 0, 0, 1, 0];
                   %annotatedImage_enhanced_w_tracks(-pixel_pos(2), pixel_pos(1)) = 255;
                   isCrossing = true;
                end

            % (4) walkaway: if pedestrian just crossed the street and is on the
            % sidewalk
                if ( strcmp(Region(ii),'Sidewalk') & strcmp(cwHeadingState,'Not_Headed') & strcmp(walk_state, 'Walking') )
                   hybrid_state = 'Walk_away'; 
                   prob_hybrid_state = [0, 0, 0, 0, 1];
                   %annotatedImage_enhanced_w_tracks(-pixel_pos(2), pixel_pos(1)) = 120;
                end

                % update the state variables not relevant for predictions
                tracksData.HybridState(ii) = hybrid_state;  
                %tracksData.ProbHybridState(ii,:) = prob_hybrid_state;  
                %tracksData.isCrossing(ii) = isCrossing; 
                %tracksData.distCW(ii) = dist_cw;
                
        end % end of hybrid update (not prediction) loop
            
        % update other variables
        tracksData.closestCW(ii) = cwInd;
        tracksData.calcHeading(ii) = pedHead;

        % plot the tracks (according to hybrid states) on the enhanced image    
        % annotatedImage_enhanced(index(1), index(2)) = 255;

        % update wait time; wait time initialized to previous time step
        % wait time in 'updatePedContStates'
        if (strcmp(tracksData.HybridState(end), 'Wait') )
            if tracksData.waitTimeSteps(end)==-1
                tracksData.waitTimeSteps(end) = 0; % wait starts
            else
                tracksData.waitTimeSteps(end) = tracksData.waitTimeSteps(end) + reSampleRate;
            end
        end
             
    end
    
    % update region
    %tracksData.Region = Region;
    
    %% loop ends

end

