function tracksData = carState(tracksData, cw, orthopxToMeter, road_center)

% Assumption 1: when the car has crossed a crosswalk, then it is considered to be
% approaching the next crosswalk along the same direction.

%debugging
if tracksData.trackId(1) == 21
    x = 1;
end



% parameters
scale_down_factor = 12;
moving_threshold = 0.2;

% in pixels
tracksData.xCenterPix = int32(tracksData.xCenter/(scale_down_factor*orthopxToMeter));
tracksData.yCenterPix = int32(tracksData.yCenter/(scale_down_factor*orthopxToMeter));

%initialize
car_heading = 0;

%if vehicle is not parked
if mean(tracksData.lonVelocity) > 0.1   
%% Step 1: Find the lanes the car is travelling in
   for ii = 1:size(tracksData,1)
        
        car_pos = [tracksData.xCenterPix(ii), tracksData.yCenterPix(ii)];
        car_cw_heading = atan2(double([road_center(2) - car_pos(2)]), double([road_center(1) - car_pos(1)]))*180/pi;
        % to reduce noise in the estimation of heading because of noisy position and velocity data
        y_vel = tracksData.yVelocity(ii);
        if ( abs(y_vel) < moving_threshold )
            y_vel = 0;
        end
        x_vel = tracksData.xVelocity(ii);
        if ( abs(x_vel) < moving_threshold )
            x_vel = 0;
        end
        % when vehicle is stopped, the heading should
        % remain as the previous heading; else calculate
        % new heading
        if ~(x_vel==0 && y_vel==0)
            car_heading = atan2(y_vel, x_vel)*180/pi;
        end
        % car_heading = atan2(tracksData.yVelocity(ii), tracksData.xVelocity(ii))*180/pi;
       
        % distance between car and crosswalk (in pixels)
        dist_cw1 = sqrt(double(cw.center_x(1) - car_pos(1))^2 + double(cw.center_y(1) - car_pos(2))^2);
        dist_cw2 = sqrt(double(cw.center_x(2) - car_pos(1))^2 + double(cw.center_y(2) - car_pos(2))^2); 
        dist_cw3 = sqrt(double(cw.center_x(3) - car_pos(1))^2 + double(cw.center_y(3) - car_pos(2))^2);
        dist_cw4 = sqrt(double(cw.center_x(4) - car_pos(1))^2 + double(cw.center_y(4) - car_pos(2))^2);
       
        % heading between car and crosswalk (in degrees)
        car_cw1_angle = atan2(double([cw.center_y(1) - car_pos(2)]), double([cw.center_x(1) - car_pos(1)]))*180/pi;
        car_cw2_angle = atan2(double([cw.center_y(2) - car_pos(2)]), double([cw.center_x(2) - car_pos(1)]))*180/pi;  
        car_cw3_angle = atan2(double([cw.center_y(3) - car_pos(2)]), double([cw.center_x(3) - car_pos(1)]))*180/pi;  
        car_cw4_angle = atan2(double([cw.center_y(4) - car_pos(2)]), double([cw.center_x(4) - car_pos(1)]))*180/pi;  

        cw_dist = inf;
        cw_ind = 0;
        
        % conditions for lane
        % car heading is in east-west direction; (45 degrees buffer; also
        % the road is 10-15 tilted)
        if ( (car_heading > -45 && car_heading < 45) || (car_heading > -225 && car_heading < -135) )            
            % car is closer to crosswalk 1 or 2
            if dist_cw1 < dist_cw2              
                % car is heading towards the intersection or away from it
                % for crosswalk 1
                 %if abs(car_heading-car_cw_heading) < 90
                 if ( car_heading > -45 && car_heading < 45 )
                     car_lane{ii,1} = 'East_Left';
                 else
                     car_lane{ii,1} = 'East_Right';
                 end
                 % note the closest cw and its distance; note that the car
                 % when the difference in car heading and cw heading is
                 % greater than 90 degree, it means the car has crossed
                 % that crosswalk and is approaching the next crosswalk
                 if abs(car_heading-car_cw1_angle) < 110
                     cw_dist = dist_cw1;
                     cw_ind = 1;
                 elseif abs(car_heading-car_cw2_angle) < 110
                     cw_dist = dist_cw2;
                     cw_ind = 2;
                 end
            else
                % car is heading towards the intersection or away from it
                % for crosswalk 2
                 if ( car_heading > -45 && car_heading < 45 )
                     car_lane{ii,1} = 'West_Right';
                 else
                     car_lane{ii,1} = 'West_Left';
                 end
                 
                 if abs(car_heading-car_cw2_angle) < 110
                     cw_dist = dist_cw2;
                     cw_ind = 2;
                 elseif abs(car_heading-car_cw1_angle) < 110
                     cw_dist = dist_cw1;
                     cw_ind = 1;
                 end
            end           
        else
            % car is closer to crosswalk 3 or 4
            if dist_cw3 < dist_cw4              
                % car is heading towards the intersection or away from it
                % for crosswalk 3
                 if ( car_heading > 10 && car_heading < 135 )
                     car_lane{ii,1} = 'South_Right';
                 else
                     car_lane{ii,1} = 'South_Left';
                 end   
                 if abs(car_heading-car_cw3_angle) < 90
                     cw_dist = dist_cw3;
                     cw_ind = 3;
                 elseif abs(car_heading-car_cw4_angle) < 90
                     cw_dist = dist_cw4;
                     cw_ind = 4;
                 end
            else
                % car is heading towards the intersection or away from it
                % for crosswalk 4
                 %if abs(car_heading-car_cw_heading) < 90
                 if ( car_heading > -135 && car_heading < -45 )
                     car_lane{ii,1} = 'North_Right';
                 else
                     car_lane{ii,1} = 'North_Left';
                 end
                 if abs(car_heading-car_cw4_angle) < 90
                     cw_dist = dist_cw4;
                     cw_ind = 4;
                 elseif abs(car_heading-car_cw3_angle) < 90
                     cw_dist = dist_cw3;
                     cw_ind = 3;
                 end
            end
        end
        
        % update the cw data
        tracksData.distCW(ii) = cw_dist;
        tracksData.closestCW(ii) = cw_ind;
     
   end
   
   % update the lane data
   tracksData.car_lane = car_lane;
end   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%






end