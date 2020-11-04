carData = carDataOr;
currentTSActiveCarData = currentTSActiveCarDataOr;

turnThreshold = Params.turnThreshold;
goalDistThreshold = Params.goalDistThreshold;
scaleFactor = Params.scaleFactor;
orthopxToMeter = Params.orthopxToMeter;
headingRateLimit = Params.headingRateLimit;
del_t =  Params.delta_T;

% a) is better
tic
for ii=1:1000
carPos = double([carData.xCenterPix, carData.yCenterPix]);
carVel =  [carData.xVelocity, carData.yVelocity];
carHeading = carData.calcHeading;
prevCarHeading = carHeading;
carLane = carData.car_lane;
prevCarLane = carLane;
       
% use a proportional controller if they are turning
if carData.turn
    % identify the lane
    carData.xVelocity = carData.xVelocity  + del_t*carData.xAcceleration ;
    carData.yVelocity = carData.yVelocity  + del_t*carData.yAcceleration ;
end
 
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% reset states based on goal, when car is not turning
% crosswalk 1
if ~carData.turn && ( carData.reachGoal || carData.changeLane)
    if strcmp(carLane,'East_Right')
        head1Disp = reset.carLane.goal(1,:) - carPos;
        head2Disp = reset.carCW.goal(1,:) - carPos;
        head3Disp = reset.carCW.goal(4,:) - carPos;
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

    if strcmp(carLane,'South_Right')
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

    if strcmp(carLane,'North_Right')
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

    if strcmp(carLane,'East_Left')            
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

    if strcmp(carLane,'West_Left')            
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

    if strcmp(carLane,'South_Left')            
        head1Disp = reset.carCW.goal(6,:) - carPos;
        head2Disp = reset.carLane.goal(6,:) - carPos;
        head1 = atan2(head1Disp(2), head1Disp(1));
        head2 = atan2(head2Disp(2), head2Disp(1)); 

        if head1 < 0
            carHeading = head1;
        else
            carHeading = head2;
        end
    end

    if strcmp(carLane,'North_Left')            
        head1Disp = reset.carCW.goal(8,:) - carPos;
        head2Disp = reset.carLane.goal(8,:) - carPos;
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
    carData.xVelocity = vel_mag*cosd(actualCarHeading);
    carData.yVelocity = vel_mag*sind(actualCarHeading);
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

goal_reach_ind = find(distGoal <= goalDistThreshold, 1);
if ~isempty(goal_reach_ind)
    carData.reachGoal = true;
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%update the position (velocity remains constant if vehicle is not
%turning)
carData.xCenterPix = carData.xCenterPix + del_t*carData.xVelocity/(orthopxToMeter*scaleFactor);
carData.yCenterPix = carData.yCenterPix + del_t*carData.yVelocity/(orthopxToMeter*scaleFactor);    

%% remove the cars if they are out of range of the ego-vehicle or out of range of the environment
distCarEgo = 1;   
AVHeading = carHeading;
if  (( distCarEgo < Params.sensingRange && abs(AVHeading-carHeading) < 90 ) &&...
      ( carPos(1)>=100 && carPos(1)<=950 && carPos(2)>=-560 && carPos(2)<=-100 ) )

    %flag.outOfRange = false;

    %% find the region of the car
    % distance between car and crosswalk (in pixels)
    dist_cw1 = sqrt(double(cw.center_x(1) - carPos(1))^2 + double(cw.center_y(1) - carPos(2))^2);
    dist_cw2 = sqrt(double(cw.center_x(2) - carPos(1))^2 + double(cw.center_y(2) - carPos(2))^2); 
    dist_cw3 = sqrt(double(cw.center_x(3) - carPos(1))^2 + double(cw.center_y(3) - carPos(2))^2);
    dist_cw4 = sqrt(double(cw.center_x(4) - carPos(1))^2 + double(cw.center_y(4) - carPos(2))^2);  

    % heading between car and crosswalk (in degrees)
    car_cw1_angle = atan2(double(cw.center_y(1) - carPos(2)), double(cw.center_x(1) - carPos(1)))*180/pi;
    car_cw2_angle = atan2(double(cw.center_y(2) - carPos(2)), double(cw.center_x(2) - carPos(1)))*180/pi;  
    car_cw3_angle = atan2(double(cw.center_y(3) - carPos(2)), double(cw.center_x(3) - carPos(1)))*180/pi;  
    car_cw4_angle = atan2(double(cw.center_y(4) - carPos(2)), double(cw.center_x(4) - carPos(1)))*180/pi;  


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
                     carData.closestCW = 1;
                 elseif abs(carHeading-car_cw2_angle) < 110
                     carData.closestCW = 2;
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
                     carLane = 'South_Right';
                 else
                     carLane = 'South_Left';
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
                     carLane = 'North_Right';
                 else
                     carLane = 'North_Left';
                 end
                 if abs(carHeading-car_cw4_angle) < 90
                     carData.closestCW = 4;
                 elseif abs(carHeading-car_cw3_angle) < 90
                     carData.closestCW = 3;
                 end
            end
        end
        carData.car_lane{1} = carLane;

        LaneChange_1 = ~(strcmp(prevCarLane, 'North_Left')) && (strcmp(carLane, 'North_Left') );
        LaneChange_2 = ~(strcmp(prevCarLane, 'South_Left') ) && (strcmp(carLane,'South_Left'));
        LaneChange_3 = ~(strcmp(prevCarLane,'East_Left') ) && (strcmp(carLane,'East_Left'));
        LaneChange_4 = ~(strcmp(prevCarLane, 'West_Left') ) && (strcmp(carLane,'West_Left'));

        if (LaneChange_1 || LaneChange_2 || LaneChange_3 || LaneChange_4)
            carData.changeLane = true;
        else
            carData.changeLane = false;
        end

        %% check if the vehicle is turning and not changed lanes yet
        if abs(carData.latAcceleration) > turnThreshold && ~carData.changeLane
            carData.turn = true;
        else
            carData.turn = false;
            carData.latAcceleration = 0;
        end
end
%% update the values
nextTSActiveCarData = carData;
end
disp('individual variables')
toc
%%%%%

%% b)
carId = 1;
tic
for ii=1:1000
carPos = double([currentTSActiveCarData.xCenterPix(carId), currentTSActiveCarData.yCenterPix(carId)]);
carVel =  [currentTSActiveCarData.xVelocity(carId), currentTSActiveCarData.yVelocity(carId)];
carAcc = [currentTSActiveCarData.xAcceleration(carId), currentTSActiveCarData.yAcceleration(carId)];
carHeading = currentTSActiveCarData.calcHeading(carId);
carTurn = currentTSActiveCarData.turn(carId);
reachGoal = currentTSActiveCarData.reachGoal(carId);
changeLane = currentTSActiveCarData.changeLane(carId);
prevCarHeading = carHeading;
carLane = currentTSActiveCarData.car_lane(carId);
prevCarLane = carLane;
       
% use a proportional controller if they are turning
if carTurn
    carVel = carVel + del_t*carAcc;
end
 
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% reset states based on goal, when car is not turning
% crosswalk 1
if ~carTurn && ( reachGoal || changeLane)
    if strcmp(carLane,'East_Right')
        head1Disp = reset.carLane.goal(1,:) - carPos;
        head2Disp = reset.carCW.goal(1,:) - carPos;
        head3Disp = reset.carCW.goal(4,:) - carPos;
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

    if strcmp(carLane,'South_Right')
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

    if strcmp(carLane,'North_Right')
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

    if strcmp(carLane,'East_Left')            
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

    if strcmp(carLane,'West_Left')            
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

    if strcmp(carLane,'South_Left')            
        head1Disp = reset.carCW.goal(6,:) - carPos;
        head2Disp = reset.carLane.goal(6,:) - carPos;
        head1 = atan2(head1Disp(2), head1Disp(1));
        head2 = atan2(head2Disp(2), head2Disp(1)); 

        if head1 < 0
            carHeading = head1;
        else
            carHeading = head2;
        end
    end

    if strcmp(carLane,'North_Left')            
        head1Disp = reset.carCW.goal(8,:) - carPos;
        head2Disp = reset.carLane.goal(8,:) - carPos;
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

goal_reach_ind = find(distGoal <= goalDistThreshold, 1);
if ~isempty(goal_reach_ind)
    reachGoal = true;
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%update the position (velocity remains constant if vehicle is not
%turning)
carPos = carPos + del_t*carVel/(orthopxToMeter*scaleFactor);    

%% remove the cars if they are out of range of the ego-vehicle or out of range of the environment
distCarEgo =1;   
AVHeading = carHeading;
if  (( distCarEgo < Params.sensingRange && abs(AVHeading-carHeading) < 90 ) &&...
      ( carPos(1)>=100 && carPos(1)<=950 && carPos(2)>=-560 && carPos(2)<=-100 ) )

    %flag.outOfRange = false;

    %% find the region of the car
    % distance between car and crosswalk (in pixels)
    dist_cw1 = sqrt(double(cw.center_x(1) - carPos(1))^2 + double(cw.center_y(1) - carPos(2))^2);
    dist_cw2 = sqrt(double(cw.center_x(2) - carPos(1))^2 + double(cw.center_y(2) - carPos(2))^2); 
    dist_cw3 = sqrt(double(cw.center_x(3) - carPos(1))^2 + double(cw.center_y(3) - carPos(2))^2);
    dist_cw4 = sqrt(double(cw.center_x(4) - carPos(1))^2 + double(cw.center_y(4) - carPos(2))^2);  

    % heading between car and crosswalk (in degrees)
    car_cw1_angle = atan2(double(cw.center_y(1) - carPos(2)), double(cw.center_x(1) - carPos(1)))*180/pi;
    car_cw2_angle = atan2(double(cw.center_y(2) - carPos(2)), double(cw.center_x(2) - carPos(1)))*180/pi;  
    car_cw3_angle = atan2(double(cw.center_y(3) - carPos(2)), double(cw.center_x(3) - carPos(1)))*180/pi;  
    car_cw4_angle = atan2(double(cw.center_y(4) - carPos(2)), double(cw.center_x(4) - carPos(1)))*180/pi;  


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
                     carData.closestCW = 1;
                 elseif abs(carHeading-car_cw2_angle) < 110
                     carData.closestCW = 2;
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
                     carLane = 'South_Right';
                 else
                     carLane = 'South_Left';
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
                     carLane = 'North_Right';
                 else
                     carLane = 'North_Left';
                 end
                 if abs(carHeading-car_cw4_angle) < 90
                     carData.closestCW = 4;
                 elseif abs(carHeading-car_cw3_angle) < 90
                     carData.closestCW = 3;
                 end
            end
        end
        carData.car_lane{1} = carLane;

        LaneChange_1 = ~(strcmp(prevCarLane, 'North_Left')) && (strcmp(carLane, 'North_Left') );
        LaneChange_2 = ~(strcmp(prevCarLane, 'South_Left') ) && (strcmp(carLane,'South_Left'));
        LaneChange_3 = ~(strcmp(prevCarLane,'East_Left') ) && (strcmp(carLane,'East_Left'));
        LaneChange_4 = ~(strcmp(prevCarLane, 'West_Left') ) && (strcmp(carLane,'West_Left'));

        if (LaneChange_1 || LaneChange_2 || LaneChange_3 || LaneChange_4)
            changeLane = true;
        else
            changeLane = false;
        end

        %% check if the vehicle is turning and not changed lanes yet
        if abs(currentTSActiveCarData.latAcceleration(carId)) > turnThreshold && ~changeLane
            currentTSActiveCarData.turn(carId) = true;
        else
            currentTSActiveCarData.turn(carId) = false;
            currentTSActiveCarData.latAcceleration(carId) = 0;
        end
        
       
        
end

 % add all variables back
currentTSActiveCarData.xCenterPix(carId) = carPos(1);
currentTSActiveCarData.yCenterPix(carId) = carPos(2);
currentTSActiveCarData.xVelocity(carId) = carVel(1);
currentTSActiveCarData.yVelocity(carId) = carVel(2);
currentTSActiveCarData.xAcceleration(carId) = carAcc(1);
currentTSActiveCarData.yAcceleration(carId) = carAcc(2);
currentTSActiveCarData.calcHeading(carId) = carHeading;
currentTSActiveCarData.reachGoal(carId) = reachGoal;
currentTSActiveCarData.changeLane(carId) = changeLane;

nextTSActiveCarData = currentTSActiveCarData.changeLane(carId);
end
disp('struct variables')
toc
% 
% disp('a is better')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
