function nextTSActiveCarData = updateCarState(currentTSActiveCarData, AVStates, Params, reset, cw)

%% setup
% parameters
turnThreshold = Params.turnThreshold;
goalDistThreshold = Params.goalDistThreshold;
scaleFactor = Params.scaleFactor;
orthopxToMeter = Params.orthopxToMeter;
headingRateLimit = Params.headingRateLimit;
del_t =  Params.delta_T;
AVPosPixels = AVStates.carPosPixels;
AVHeading = AVStates.carHeading;

% initialize
N_cars = size(currentTSActiveCarData, 1);
nextTSActiveCarData = currentTSActiveCarData;
nextCarId = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% update states for all cars
for carId = 1:N_cars   
    % separate variables for parameters (fast computation)
    carPos = [currentTSActiveCarData.xCenter(carId), currentTSActiveCarData.yCenter(carId)];
    carPosPixels = carPos/(orthopxToMeter*scaleFactor);
    carVel =  [currentTSActiveCarData.xVelocity(carId), currentTSActiveCarData.yVelocity(carId)];
    carHeading = currentTSActiveCarData.calcHeading(carId);
    prevCarHeading = carHeading;
    carLane = currentTSActiveCarData.car_lane(carId);
    prevCarLane = carLane;    
    %% %%%%%%%%%%%%%%%%%%%%%%
    % reset states based on goal, when car is turning
    % crosswalk 1  
%     % use constant turn model if they are turning
%     if carData.turn
%         % maintain the same acceleration (ideally only lateral accel must be the same) until reaching the new
%         % lane
%         carData.xVelocity = carData.xVelocity  + del_t*carData.xAcceleration ;
%         carData.yVelocity = carData.yVelocity  + del_t*carData.yAcceleration ;
%     end
    %%%%%%%%%%%%%%%%%%%%%%
    % use a proportional controller if they are turning
    if currentTSActiveCarData.turn(carId)
        carVel(1) = carVel(1) + del_t*currentTSActiveCarData.xAcceleration(carId);
        carVel(2) = carVel(2) + del_t*currentTSActiveCarData.yAcceleration(carId);
    end
    %%%%%%%%%%%%%%%%%%%%%%
    
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % reset states based on goal, when car is not turning
    % crosswalk 1
    if ~currentTSActiveCarData.turn(carId) && ( currentTSActiveCarData.reachGoal(carId) || currentTSActiveCarData.changeLane(carId))
        if strcmp(carLane,'East_Right')
            head1Disp = reset.carLane.goal(1,:) - carPosPixels;
            head2Disp = reset.carCW.goal(1,:) - carPosPixels;
            head3Disp = reset.carCW.goal(4,:) - carPosPixels;
            head1 = atan2(head1Disp(2), head1Disp(1));
            head2 = atan2(head2Disp(2), head2Disp(1)); 
            head3 = atan2(head3Disp(2), head3Disp(1)); 
            
            if abs(head1)> 90      %[-180 to -90 and 90 to 180] 
                carHeading = head1;
            elseif abs(head2) > 90
                carHeading = head2;
            else
                carHeading = head3;
            end
        end
        
        if strcmp(carLane,'West_Right')
            head1Disp = reset.carLane.goal(3,:) - carPosPixels;
            head2Disp = reset.carCW.goal(3,:) - carPosPixels;
            head3Disp = reset.carCW.goal(2,:) - carPosPixels;
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
        
        if strcmp(carLane,'South_Right')
            head1Disp = reset.carLane.goal(5,:) - carPosPixels;
            head2Disp = reset.carCW.goal(5,:) - carPosPixels;
            head3Disp = reset.carCW.goal(8,:) - carPosPixels;
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
        
        if strcmp(carLane,'North_Right')
            head1Disp = reset.carLane.goal(7,:) - carPosPixels;
            head2Disp = reset.carCW.goal(7,:) - carPosPixels;
            head3Disp = reset.carCW.goal(6,:) - carPosPixels;
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
        
        if strcmp(carLane,'East_Left')            
            head1Disp = reset.carCW.goal(2,:) - carPosPixels;
            head2Disp = reset.carLane.goal(2,:) - carPosPixels;
            head1 = atan2(head1Disp(2), head1Disp(1));
            head2 = atan2(head2Disp(2), head2Disp(1)); 
            
            if abs(head1) < 90
                carHeading = head1;
            else
                carHeading = head2;
            end
        end
        
        if strcmp(carLane,'West_Left')            
            head1Disp = reset.carCW.goal(4,:) - carPosPixels;
            head2Disp = reset.carLane.goal(4,:) - carPosPixels;
            head1 = atan2(head1Disp(2), head1Disp(1));
            head2 = atan2(head2Disp(2), head2Disp(1)); 
            
            if abs(head1) > 90
                carHeading = head1;
            else
                carHeading = head2;
            end
        end

        if strcmp(carLane,'South_Left')            
            head1Disp = reset.carCW.goal(6,:) - carPosPixels;
            head2Disp = reset.carLane.goal(6,:) - carPosPixels;
            head1 = atan2(head1Disp(2), head1Disp(1));
            head2 = atan2(head2Disp(2), head2Disp(1)); 
            
            if head1 < 0
                carHeading = head1;
            else
                carHeading = head2;
            end
        end
        
        if strcmp(carLane,'North_Left')            
            head1Disp = reset.carCW.goal(8,:) - carPosPixels;
            head2Disp = reset.carLane.goal(8,:) - carPosPixels;
            head1 = atan2(head1Disp(2), head1Disp(1));
            head2 = atan2(head2Disp(2), head2Disp(1)); 
            
            if head1 > 0
                carHeading = head1;
            else
                carHeading = head2;
            end
        end
        
        % calculate new velocity
        vel_mag = norm(carVel);
        headings = [carHeading, prevCarHeading + del_t*headingRateLimit];
        [~,ind] = min(abs(headings));
        actualCarHeading = headings(ind);
        carVel = [vel_mag*cosd(actualCarHeading), vel_mag*sind(actualCarHeading)];
    end    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
    %check if the car is reaching a goal location
    distGoal(1) = norm(reset.carCW.goal(1,:) - carPosPixels);
    distGoal(2) = norm(reset.carCW.goal(2,:) - carPosPixels);
    distGoal(3) = norm(reset.carCW.goal(3,:) - carPosPixels);
    distGoal(4) = norm(reset.carCW.goal(4,:) - carPosPixels);
    distGoal(5) = norm(reset.carCW.goal(5,:) - carPosPixels);
    distGoal(6) = norm(reset.carCW.goal(6,:) - carPosPixels);
    distGoal(7) = norm(reset.carCW.goal(7,:) - carPosPixels);
    distGoal(8) = norm(reset.carCW.goal(8,:) - carPosPixels);

    distGoal(9) = norm(reset.carLane.goal(1,:) - carPosPixels);
    distGoal(10) = norm(reset.carLane.goal(2,:) - carPosPixels);
    distGoal(11) = norm(reset.carLane.goal(3,:) - carPosPixels);
    distGoal(12) = norm(reset.carLane.goal(4,:) - carPosPixels);
    distGoal(13) = norm(reset.carLane.goal(5,:) - carPosPixels);
    distGoal(14) = norm(reset.carLane.goal(6,:) - carPosPixels);
    distGoal(15) = norm(reset.carLane.goal(7,:) - carPosPixels);
    distGoal(16) = norm(reset.carLane.goal(8,:) - carPosPixels);
    
    goal_reach_ind = find(distGoal <= goalDistThreshold, 1);
    if ~isempty(goal_reach_ind)
        currentTSActiveCarData.reachGoal(carId) = true;
    end
    
    %update the position (velocity remains constant if vehicle is not
    %turning)
    carPos = carPos + del_t*carVel;   
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
    %do not consider the cars if they are out of range of the ego-vehicle or out of range of the environment
    distCarEgo = norm(carPosPixels-double(AVPosPixels));   
    if  (( distCarEgo < Params.sensingRange && abs(AVHeading-carHeading) < 90 ) &&...
          ( carPosPixels(1)>=100 && carPosPixels(1)<=950 && carPosPixels(2)>=-560 && carPosPixels(2)<=-100 ) )
        %flag.outOfRange = false;  
        %find the region of the car
        % distance between car and crosswalk (in pixels)
        dist_cw1 = sqrt(double(cw.center_x(1) - carPosPixels(1))^2 + double(cw.center_y(1) - carPosPixels(2))^2);
        dist_cw2 = sqrt(double(cw.center_x(2) - carPosPixels(1))^2 + double(cw.center_y(2) - carPosPixels(2))^2); 
        dist_cw3 = sqrt(double(cw.center_x(3) - carPosPixels(1))^2 + double(cw.center_y(3) - carPosPixels(2))^2);
        dist_cw4 = sqrt(double(cw.center_x(4) - carPosPixels(1))^2 + double(cw.center_y(4) - carPosPixels(2))^2);  
        % heading between car and crosswalk (in degrees)
        car_cw1_angle = atan2(double(cw.center_y(1) - carPosPixels(2)), double(cw.center_x(1) - carPosPixels(1)))*180/pi;
        car_cw2_angle = atan2(double(cw.center_y(2) - carPosPixels(2)), double(cw.center_x(2) - carPosPixels(1)))*180/pi;  
        car_cw3_angle = atan2(double(cw.center_y(3) - carPosPixels(2)), double(cw.center_x(3) - carPosPixels(1)))*180/pi;  
        car_cw4_angle = atan2(double(cw.center_y(4) - carPosPixels(2)), double(cw.center_x(4) - carPosPixels(1)))*180/pi;  

         % car heading is in east-west direction; (45 degrees buffer; also
         % the road is 10-15 tilted)
        if ( (carHeading > -45 && carHeading < 45) || (carHeading > -225 && carHeading < -135) )            
            % car is closer to crosswalk 1 or 2
            if dist_cw1 < dist_cw2              
                % car is heading towards the intersection or away from it
                % for crosswalk 1
                 %if abs(carHeading-car_cw_heading) < 90
                 if ( carHeading > -45 && carHeading < 45 )
                     carLane = 'East_Left';
                 else
                     carLane = 'East_Right';
                 end
                 % note the closest cw and its distance; note that the car
                 % when the difference in car heading and cw heading is
                 % greater than 90 degree, it means the car has crossed
                 % that crosswalk and is approaching the next crosswalk
                 if abs(carHeading-car_cw1_angle) < 110
                     currentTSActiveCarData.closestCW(carId) = 1;
                 elseif abs(carHeading-car_cw2_angle) < 110
                     currentTSActiveCarData.closestCW(carId) = 2;
                 end
            else
                % car is heading towards the intersection or away from it
                % for crosswalk 2
                 if ( carHeading > -45 && carHeading < 45 )
                     carLane = 'West_Right';
                 else
                     carLane = 'West_Left';
                 end

                 if abs(carHeading-car_cw2_angle) < 110
                     currentTSActiveCarData.closestCW(carId)  = 2;
                 elseif abs(carHeading-car_cw1_angle) < 110
                     currentTSActiveCarData.closestCW(carId)  = 1;
                 end
            end           
        else
            % car is closer to crosswalk 3 or 4
            if dist_cw3 < dist_cw4              
                % car is heading towards the intersection or away from it
                % for crosswalk 3
                 if ( carHeading > 10 && carHeading < 135 )
                     carLane = 'South_Right';
                 else
                     carLane = 'South_Left';
                 end   
                 if abs(carHeading-car_cw3_angle) < 90
                     currentTSActiveCarData.closestCW(carId) = 3;
                 elseif abs(carHeading-car_cw4_angle) < 90
                     currentTSActiveCarData.closestCW(carId) = 4;
                 end
            else
                % car is heading towards the intersection or away from it
                % for crosswalk 4
                 %if abs(carHeading-car_cw_heading) < 90
                 if ( carHeading > -135 && carHeading < -45 )
                     carLane = 'North_Right';
                 else
                     carLane = 'North_Left';
                 end
                 if abs(carHeading-car_cw4_angle) < 90
                     currentTSActiveCarData.closestCW(carId) = 4;
                 elseif abs(carHeading-car_cw3_angle) < 90
                     currentTSActiveCarData.closestCW(carId) = 3;
                 end
            end
        end
        currentTSActiveCarData.car_lane(carId) = carLane;

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % check if the vehicle has changed lanes
%             LaneChange_1 = (strcmp(prev_car_lane, 'East_Right') || strcmp(prev_car_lane, 'East_Left') ) && (strcmp(car_lane, 'North_Left') || strcmp(car_lane,'South_Left'));
%             LaneChange_2 = (strcmp(prev_car_lane,'West_Right') || strcmp(prev_car_lane, 'West_Left') ) && (strcmp(car_lane,'North_Left') || strcmp(car_lane,'South_Left'));
%             LaneChange_3 = (strcmp(prev_car_lane,'South_Right') || strcmp(prev_car_lane, 'South_Left') ) && (strcmp(car_lane,'East_Left') || strcmp(car_lane,'West_Left'));
%             LaneChange_4 = (strcmp(prev_car_lane,'North_Right') || strcmp(prev_car_lane, 'North_Left') ) && (strcmp(car_lane,'East_Left') || strcmp(car_lane,'West_Left'));
%             
        LaneChange_1 = ~(strcmp(prevCarLane, 'North_Left')) && (strcmp(carLane, 'North_Left') );
        LaneChange_2 = ~(strcmp(prevCarLane, 'South_Left') ) && (strcmp(carLane,'South_Left'));
        LaneChange_3 = ~(strcmp(prevCarLane,'East_Left') ) && (strcmp(carLane,'East_Left'));
        LaneChange_4 = ~(strcmp(prevCarLane, 'West_Left') ) && (strcmp(carLane,'West_Left'));

        if (LaneChange_1 || LaneChange_2 || LaneChange_3 || LaneChange_4)
            currentTSActiveCarData.changeLane(carId) = true;
        else
            currentTSActiveCarData.changeLane(carId) = false;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %check if the vehicle is turning and not changed lanes yet
        if abs(currentTSActiveCarData.latAcceleration(carId)) > turnThreshold && ~currentTSActiveCarData.changeLane(carId)
            currentTSActiveCarData.turn(carId) = true;
        else
            currentTSActiveCarData.turn(carId) = false;
            currentTSActiveCarData.latAcceleration(carId) = 0;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %update the values
        currentTSActiveCarData.xCenter(carId) = carPos(1);
        currentTSActiveCarData.yCenter(carId) = carPos(2);
        currentTSActiveCarData.xVelocity(carId) = carVel(1);
        currentTSActiveCarData.yVelocity(carId) = carVel(2);
        nextTSActiveCarData(nextCarId,:) = currentTSActiveCarData(carId);

        nextCarId = nextCarId + 1;
    end
    
end  % for all cars
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% remove un-updated initialized car predictions
nextTSActiveCarData(nextCarId:end,:) = [];

end