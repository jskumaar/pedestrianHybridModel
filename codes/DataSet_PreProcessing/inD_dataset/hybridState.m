function [tracksData] = hybridState(tracksData, cw, flag, orthopxToMeter, annotatedImage_enhanced)
    %Note:
    %1) make sure the enhanced annotataed background image is the input to
    %this function

    % fixed parameters
    scale_down_factor = 12;
    flag.pred = false; %temporarily fixing this parameter

    % parameters
    heading_threshold = 90; %45 degrees
    stopping_threshold = 0.5; %speed m/s
    walking_threshold = 0.1; %speed m/s 
    cw_dist_threshold = 240; %in pixels
    cw_cross_threshold = 50; % approximate width of a lane
    dec_zone = 8*scale_down_factor; %8 m radius (in pixels)

    % initialize variables
    isCrossing = false;
    ped_head = 0;
    
    % convert track data to pixels
    if ~tracksData.xCenterPix
        tracksData.xCenterPix = int32(tracksData.xCenter/(scale_down_factor*orthopxToMeter));
        tracksData.yCenterPix = int32(tracksData.yCenter/(scale_down_factor*orthopxToMeter));
    end

    % length of this particular pedestrian track    
    N_instances = size(tracksData,1);

    
    %% loop starts    
    for ii=1:N_instances
           % index based on the current pixel position of pedestrian
           pixel_pos = [tracksData.xCenterPix(ii), tracksData.yCenterPix(ii)];

           %% check pedestrian position, heading, and velocity discrete states
           % 1) position of pedestrians (make sure the values match with the
           % enhanced values in annotated_plot.m)
           % initialize
           tracksData.Region(ii) = strings;
           if (annotatedImage_enhanced(-pixel_pos(2), pixel_pos(1))==200)
               tracksData.Region(ii) = 'Crosswalk_Marked';
           elseif (annotatedImage_enhanced(-pixel_pos(2), pixel_pos(1))==100)
               tracksData.Region(ii) = 'Crosswalk_UnMarked';
           elseif (annotatedImage_enhanced(-pixel_pos(2), pixel_pos(1))==50)
               tracksData.Region(ii) = 'Road';
           elseif (annotatedImage_enhanced(-pixel_pos(2), pixel_pos(1))==150)
               tracksData.Region(ii) = 'Sidewalk';               
           end 

           % 2) is pedestrian heading towards a crosswalk?
           % heading angle of pedestrians      
           % ped_head = atan2(tracksData.yVelocity(ii),tracksData.xVelocity(ii))*180/pi;
           % to reduce noise in the estimation of heading because of noisy position and velocity data
            y_vel = tracksData.yVelocity(ii);
            if ( abs(y_vel) < walking_threshold )
                y_vel = 0;
            end
            x_vel = tracksData.xVelocity(ii);
            if ( abs(x_vel) < walking_threshold )
                x_vel = 0;
            end
            % when pedestrian is stopped, maintain the previpus heading
            % instead of saying it as zero.
            if x_vel~=0 || y_vel~=0
                ped_head = atan2(y_vel, x_vel)*180/pi; 
            end

           % angle between pedestrian and the crosswalks
            ped_cw1_angle = atan2(double([cw.center_y(1) - pixel_pos(2)]), double([cw.center_x(1) - pixel_pos(1)]))*180/pi;
            ped_cw2_angle = atan2(double([cw.center_y(2) - pixel_pos(2)]), double([cw.center_x(2) - pixel_pos(1)]))*180/pi;  
            ped_cw3_angle = atan2(double([cw.center_y(3) - pixel_pos(2)]), double([cw.center_x(3) - pixel_pos(1)]))*180/pi;  
            ped_cw4_angle = atan2(double([cw.center_y(4) - pixel_pos(2)]), double([cw.center_x(4) - pixel_pos(1)]))*180/pi;  

           % distance between pedestrian and crosswalk (in pixels)
           dist_cw1 = sqrt(double(cw.center_x(1) - pixel_pos(1))^2 + double(cw.center_y(1) - pixel_pos(2))^2);
           dist_cw2 = sqrt(double(cw.center_x(2) - pixel_pos(1))^2 + double(cw.center_y(2) - pixel_pos(2))^2); 
           dist_cw3 = sqrt(double(cw.center_x(3) - pixel_pos(1))^2 + double(cw.center_y(3) - pixel_pos(2))^2);
           dist_cw4 = sqrt(double(cw.center_x(4) - pixel_pos(1))^2 + double(cw.center_y(4) - pixel_pos(2))^2);
           dist_cw_temp = [inf, inf, inf, inf];  
          
           % when the heading angle is within +/- heading_threshold and pedestrian is close to a crosswalk, 
           % then pedestrian is approaching one of the crosswalks 
           cw_heading_state = 'Not_Headed';
           if  ( ( abs(ped_cw1_angle - ped_head) <= heading_threshold && dist_cw1 < cw_dist_threshold ) || dist_cw1 < cw_cross_threshold )
                cw_heading_state = 'Headed';
                dist_cw_temp(1) = dist_cw1;
           end
           if ( ( abs(ped_cw2_angle - ped_head) <= heading_threshold && dist_cw2 < cw_dist_threshold ) || dist_cw2 < cw_cross_threshold )
                cw_heading_state = 'Headed';
                dist_cw_temp(2) = dist_cw2;
           end
           if ( ( abs(ped_cw3_angle - ped_head) <= heading_threshold && dist_cw3 < cw_dist_threshold ) || dist_cw3 < cw_cross_threshold )
                cw_heading_state = 'Headed';
                dist_cw_temp(3) = dist_cw3;   
           end
           if ( ( abs(ped_cw4_angle - ped_head) <= heading_threshold && dist_cw4 < cw_dist_threshold ) || dist_cw4 < cw_cross_threshold )
                cw_heading_state = 'Headed';
                dist_cw_temp(4) = dist_cw4;
           end

           % distance to the closest crosswalk when headed towards that
           % crosswalk
           [dist_cw, cw_ind] = min(dist_cw_temp);
           % when the closest distance to a headed cw is 'inf', then change the
           % cw index to zero
           if dist_cw==inf
               cw_ind = 0;
           end

           % 3) is pedestrian walking?
           ped_vel = sqrt(tracksData.xVelocity(ii)^2 + tracksData.yVelocity(ii)^2);
           if ped_vel < stopping_threshold
               walk_state = 'Stopping';
           else
               walk_state = 'Walking';
           end

        %% Hybrid state update    
        % run the hybrid state update only during the update stage and not
        % during the prediction stage
        if ~flag.pred
            % initialize the hybrid state
              hybrid_state = 'None'; 
              prob_hybrid_state = [0, 0, 0, 0, 0];

            % conditions for Hybrid State of Pedestrians
            % (1) approach: if pedestrian is on sidewalk and headed towards a crosswalk
                if ( strcmp(tracksData.Region(ii),'Sidewalk') & strcmp(cw_heading_state,'Headed') & strcmp(walk_state, 'Walking') )
                   hybrid_state = 'Approach';
                   prob_hybrid_state = [1, 0, 0, 0, 0];
                   annotatedImage_enhanced_w_tracks(-pixel_pos(2), pixel_pos(1)) = 60;
                end

            % (2) wait: if pedestrian is on sidewalk near a crosswalk and is walking very slowly. 
            % Note: Using absolute distance to center of crosswalk for now.
                if ( strcmp(tracksData.Region(ii),'Sidewalk') & strcmp(cw_heading_state,'Headed') & strcmp(walk_state, 'Stopping') & (dist_cw < dec_zone) )
                   hybrid_state = 'Wait'; 
                   prob_hybrid_state = [0, 1, 0, 0, 0];
                   annotatedImage_enhanced_w_tracks(-pixel_pos(2), pixel_pos(1)) = 0;
                   isCrossing = true;
                end

            % (3) cross: if pedestrian is on the road (note if they are crossing or
            % jaywalking)
                if (strcmp(tracksData.Region(ii),'Crosswalk_Marked') | strcmp(tracksData.Region{ii},'Crosswalk_UnMarked') )
                   hybrid_state = 'Crossing';
                   prob_hybrid_state = [0, 0, 1, 0, 0];
                   annotatedImage_enhanced_w_tracks(-pixel_pos(2), pixel_pos(1)) = 255;
                   isCrossing = true;
                end    
                if (strcmp(tracksData.Region(ii),'Road') )
                   hybrid_state = 'Jaywalking'; 
                   prob_hybrid_state = [0, 0, 0, 1, 0];
                   annotatedImage_enhanced_w_tracks(-pixel_pos(2), pixel_pos(1)) = 255;
                   isCrossing = true;
                end

            % (4) walkaway: if pedestrian just crossed the street and is on the
            % sidewalk
                if ( strcmp(tracksData.Region(ii),'Sidewalk') & strcmp(cw_heading_state,'Not_Headed') & strcmp(walk_state, 'Walking') )
                   hybrid_state = 'Walk_away'; 
                   prob_hybrid_state = [0, 0, 0, 0, 1];
                   annotatedImage_enhanced_w_tracks(-pixel_pos(2), pixel_pos(1)) = 120;
                end

                % update the hybrid state
                tracksData.HybridState{ii} = hybrid_state;  
                tracksData.ProbHybridState{ii} = prob_hybrid_state;  
                tracksData.isCrossing(ii) = isCrossing; 
                tracksData.calcHeading{ii} = ped_head;
            
            
        end % end of hybrid update (not prediction) loop
            
            % update the close cw variables
            tracksData.closestCW(ii) = cw_ind;
            tracksData.distCW(ii) = dist_cw;

            % plot the tracks (according to hybrid states) on the enhanced image    
            % annotatedImage_enhanced(index(1), index(2)) = 255;


    end
    %% loop ends

end

