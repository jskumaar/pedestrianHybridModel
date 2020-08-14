function carData = updateCarState(currentTSActiveCarData, del_t, reset, cw)

% parameters
turn_threshold = 0.7;
goal_dist_threshold = 10;

% initialize
N_cars = size(currentTSActiveCarData, 1);

for car_id = 1:N_cars       
    carData = currentTSActiveCarData(car_id,:);
    car_pos = [carData.xCenterPix, carData.yCenterPix];
    carHeading = carData.calcHeading;
        
    % use constant turn model if they are turning
    if carData.Turn
        % maintain the same acceleration (ideally only lateral accel must be the same) until reaching the new
        % lane
        carData.xVelocity = carData.xVelocity  + del_t*carData.xAcceleration ;
        carData.yVelocity = carData.yVelocity  + del_t*carData.yAcceleration ;
    end
    
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % reset states based on goal, when car is not turning
    % crosswalk 1
    if ~carData.Turn && carData.reachGoal
        if strcmp(carData.car_lane,'East_Right')
            head1Disp = reset.carLane.goal(1,:) - carPos;
            head2Disp = reset.carCW.goal(1,:) - carPos;
            head3Disp = reset.carCW.goal(4,:) - carPos;
            head1 = atan2(head1Disp(2), head1Disp(1));
            head2 = atan2(head2Disp(2), head2Disp(1)); 
            head3 = atan2(head3Disp(2), head3Disp(1)); 
            
            if abs(head1)> 90
                carHeading = head1;
            elseif abs(head2) > 90
                carHeading = head2;
            else
                carHeading = head3;
            end
        end
        
        if strcmp(carData.car_lane,'West_Right')
            head1Disp = reset.carLane.goal(3,:) - carPos;
            head2Disp = reset.carCW.goal(3,:) - carPos;
            head3Disp = reset.carCW.goal(2,:) - carPos;
            head1 = atan2(head1Disp(2), head1Disp(1));
            head2 = atan2(head2Disp(2), head2Disp(1)); 
            head3 = atan2(head3Disp(2), head3Disp(1)); 
            
            if abs(head1) < 90
                carHeading = head1;
            elseif abs(head2) < 90
                carHeading = head2;
            else
                carHeading = head3;
            end
        end
        
        if strcmp(carData.car_lane,'South_Right')
            head1Disp = reset.carLane.goal(5,:) - carPos;
            head2Disp = reset.carCW.goal(5,:) - carPos;
            head3Disp = reset.carCW.goal(8,:) - carPos;
            head1 = atan2(head1Disp(2), head1Disp(1));
            head2 = atan2(head2Disp(2), head2Disp(1)); 
            head3 = atan2(head3Disp(2), head3Disp(1)); 
            
            if head1 > 0
                carHeading = head1;
            elseif head2 > 0
                carHeading = head2;
            else
                carHeading = head3;
            end
        end
        
        if strcmp(carData.car_lane,'North_Right')
            head1Disp = reset.carLane.goal(7,:) - carPos;
            head2Disp = reset.carCW.goal(7,:) - carPos;
            head3Disp = reset.carCW.goal(6,:) - carPos;
            head1 = atan2(head1Disp(2), head1Disp(1));
            head2 = atan2(head2Disp(2), head2Disp(1)); 
            head3 = atan2(head3Disp(2), head3Disp(1)); 
            
            if head1 < 0
                carHeading = head1;
            elseif head2 < 0
                carHeading = head2;
            else
                carHeading = head3;
            end
        end
        
        if strcmp(carData.car_lane,'East_Left')            
            head1Disp = reset.carCW.goal(2,:) - carPos;
            head2Disp = reset.carLane.goal(2,:) - carPos;
            head1 = atan2(head1Disp(2), head1Disp(1));
            head2 = atan2(head2Disp(2), head2Disp(1)); 
            
            if abs(head1) < 90
                carHeading = head1;
            else
                carHeading = head2;
            end
        end
        
        if strcmp(carData.car_lane,'West_Left')            
            head1Disp = reset.carCW.goal(4,:) - carPos;
            head2Disp = reset.carLane.goal(4,:) - carPos;
            head1 = atan2(head1Disp(2), head1Disp(1));
            head2 = atan2(head2Disp(2), head2Disp(1)); 
            
            if abs(head1) > 90
                carHeading = head1;
            else
                carHeading = head2;
            end
        end

        if strcmp(carData.car_lane,'South_Left')            
            head1Disp = reset.carCW.goal(6,:) - carPos;
            head2Disp = reset.carLane.goal(6,:) - carPos;
            head1 = atan2(head1Disp(2), head1Disp(1));
            head2 = atan2(head2Disp(2), head2Disp(1)); 
            
            if abs(head1) < 0
                carHeading = head1;
            else
                carHeading = head2;
            end
        end
        
        if strcmp(carData.car_lane,'North_Left')            
            head1Disp = reset.carCW.goal(8,:) - carPos;
            head2Disp = reset.carLane.goal(8,:) - carPos;
            head1 = atan2(head1Disp(2), head1Disp(1));
            head2 = atan2(head2Disp(2), head2Disp(1)); 
            
            if abs(head1) > 0
                carHeading = head1;
            else
                carHeading = head2;
            end
        end

    end
    
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % check if the car is reaching a goal location
    distGoal(1) = norm(reset.carCW.goal(1,:) - carPos);
    distGoal(2) = norm(reset.carCW.goal(2,:) - carPos);
    distGoal(3) = norm(reset.carCW.goal(3,:) - carPos);
    distGoal(4) = norm(reset.carCW.goal(4,:) - carPos);
    distGoal(5) = norm(reset.carCW.goal(5,:) - carPos);
    distGoal(6) = norm(reset.carCW.goal(6,:) - carPos);
    distGoal(7) = norm(reset.carCW.goal(7,:) - carPos);
    distGoal(8) = norm(reset.carCW.goal(8,:) - carPos);

    distGoal(9) = norm(reset.carLane.goal(1,:) - carPos);
    distGoal(10) = norm(reset.carLane.goal(2,:) - carPos);
    distGoal(11) = norm(reset.carLane.goal(3,:) - carPos);
    distGoal(12) = norm(reset.carLane.goal(4,:) - carPos);
    distGoal(13) = norm(reset.carLane.goal(5,:) - carPos);
    distGoal(14) = norm(reset.carLane.goal(6,:) - carPos);
    distGoal(15) = norm(reset.carLane.goal(7,:) - carPos);
    distGoal(16) = norm(reset.carLane.goal(8,:) - carPos);
    
    goal_reach_ind = find(distGoal <= goal_dist_threshold, 1);
    if ~isempty(goal_reach_ind)
        carData.reachGoal = true;
    end
    
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %update the position (velocity remains constant if vehicle is not
    %turning)
    carData.xcenterPix = carData.xcenterPix + del_t*carData.xVelocity;
    carData.ycenterPix = carData.ycenterPix + del_t*carData.yVelocity;    
    %% check if the vehicle is turning and not changed lanes yet
    if abs(carData.latAcceleration) > turn_threshold && ~carData.changeLane
        carData.Turn = true;
    else
        carData.Turn = false;
        carData.latAcceleration = 0;
    end
    
    %% find the region of the car
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


     % car heading is in east-west direction; (45 degrees buffer; also
        % the road is 10-15 tilted)
        if ( (carHeading > -45 && carHeading < 45) || (carHeading > -225 && carHeading < -135) )            
            % car is closer to crosswalk 1 or 2
            if dist_cw1 < dist_cw2              
                % car is heading towards the intersection or away from it
                % for crosswalk 1
                 %if abs(carHeading-car_cw_heading) < 90
                 if ( carHeading > -45 && carHeading < 45 )
                     carData.car_lane = 'East_Left';
                 else
                     carData.car_lane = 'East_Right';
                 end
                 % note the closest cw and its distance; note that the car
                 % when the difference in car heading and cw heading is
                 % greater than 90 degree, it means the car has crossed
                 % that crosswalk and is approaching the next crosswalk
                 if abs(carHeading-car_cw1_angle) < 110
                     carData.closestCW = 1;
                 elseif abs(carHeading-car_cw2_angle) < 110
                     carData.closestCW = 2;
                 end
            else
                % car is heading towards the intersection or away from it
                % for crosswalk 2
                 if ( carHeading > -45 && carHeading < 45 )
                     carData.car_lane = 'West_Right';
                 else
                     carData.car_lane = 'West_Left';
                 end
                 
                 if abs(carHeading-car_cw2_angle) < 110
                     carData.closestCW  = 2;
                 elseif abs(carHeading-car_cw1_angle) < 110
                     carData.closestCW  = 1;
                 end
            end           
        else
            % car is closer to crosswalk 3 or 4
            if dist_cw3 < dist_cw4              
                % car is heading towards the intersection or away from it
                % for crosswalk 3
                 if ( carHeading > 10 && carHeading < 135 )
                     carData.car_lane = 'South_Right';
                 else
                     carData.car_lane = 'South_Left';
                 end   
                 if abs(carHeading-car_cw3_angle) < 90
                     carData.closestCW = 3;
                 elseif abs(carHeading-car_cw4_angle) < 90
                     carData.closestCW = 4;
                 end
            else
                % car is heading towards the intersection or away from it
                % for crosswalk 4
                 %if abs(carHeading-car_cw_heading) < 90
                 if ( carHeading > -135 && carHeading < -45 )
                     carData.car_lane = 'North_Right';
                 else
                     carData.car_lane = 'North_Left';
                 end
                 if abs(carHeading-car_cw4_angle) < 90
                     carData.closestCW = 4;
                 elseif abs(carHeading-car_cw3_angle) < 90
                     carData.closestCW = 3;
                 end
            end
        end
        
    
    
end


end