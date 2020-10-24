%% nearLane calculator

if currentPedTrackData.closestCW(end) == 1

distPedRightLane = norm(reset.carCW.goal(1,:) - pedPos);
distPedLeftLane = norm(reset.carCW.goal(2,:) - pedPos);

    if distPedRightLane < distPedLeftLane
        Lane = "Right";

        if currentPedTrackData.closeCar_ind(end) ~=0 && currentPedTrackData.closeCar_ind(end) ~=inf
            if (strcmp(egoCarData.car_lane(egoCarTimeStep), 'East_Right') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'West_Left') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'North_Left'))
                currentPedTrackData.isNearLane(end) = true;
            else
                currentPedTrackData.isNearLane(end) = false;
            end
        end
    else
        Lane = "Left";

        if currentPedTrackData.closeCar_ind(end) ~=0 && currentPedTrackData.closeCar_ind(end) ~=inf
            if (strcmp(egoCarData.car_lane(egoCarTimeStep), 'East_Left') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'West_Right') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'North_Right') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'South_Right') )
                currentPedTrackData.isNearLane(end) = true;
            else
                currentPedTrackData.isNearLane(end) = false;
            end
        end
    end

elseif currentPedTrackData.closestCW(end) == 2

    distPedRightLane = norm(reset.carCW.goal(3,:) - pedPos);
    distPedLeftLane = norm(reset.carCW.goal(4,:) - pedPos);

    if distPedRightLane < distPedLeftLane
        Lane = "Right";
        if currentPedTrackData.closeCar_ind(end) ~=0 && currentPedTrackData.closeCar_ind(end) ~=inf
            if (strcmp(egoCarData.car_lane(egoCarTimeStep), 'East_Left') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'West_Right') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'North_Right'))
                currentPedTrackData.isNearLane(end) = true;
            else
                currentPedTrackData.isNearLane(end) = false;
            end
        end
    else
        Lane = "Left";
        if currentPedTrackData.closeCar_ind(end) ~=0 && currentPedTrackData.closeCar_ind(end) ~=inf
            if (strcmp(egoCarData.car_lane(egoCarTimeStep), 'East_Right') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'West_Left') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'North_Right') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'South_Right') )
                currentPedTrackData.isNearLane(end) = true;
            else
                currentPedTrackData.isNearLane(end) = false;
            end
        end
    end



elseif currentPedTrackData.closestCW(end) == 3

    distPedRightLane = norm(reset.carCW.goal(5,:) - pedPos);
    distPedLeftLane = norm(reset.carCW.goal(6,:) - pedPos);

    if distPedRightLane < distPedLeftLane
        Lane = "Right";
        if currentPedTrackData.closeCar_ind(end) ~=0 && currentPedTrackData.closeCar_ind(end) ~=inf
            if (strcmp(egoCarData.car_lane(egoCarTimeStep), 'East_Left') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'West_Right') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'South_Right'))
                currentPedTrackData.isNearLane(end) = true;
            else
                currentPedTrackData.isNearLane(end) = false;
            end
        end
    else
        Lane = "Left";
        if currentPedTrackData.closeCar_ind(end) ~=0 && currentPedTrackData.closeCar_ind(end) ~=inf
            if (strcmp(egoCarData.car_lane(egoCarTimeStep), 'East_Right') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'West_Right') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'North_Right') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'South_Left') )
                currentPedTrackData.isNearLane(end) = true;
            else
                currentPedTrackData.isNearLane(end) = false;
            end
        end
    end


elseif currentPedTrackData.closestCW(end) == 4
    
    distPedRightLane = norm(reset.carCW.goal(7,:) - pedPos);
    distPedLeftLane = norm(reset.carCW.goal(8,:) - pedPos);

    if distPedRightLane < distPedLeftLane
        Lane = "Right";
        if currentPedTrackData.closeCar_ind(end) ~=0 && currentPedTrackData.closeCar_ind(end) ~=inf
            if (strcmp(egoCarData.car_lane(egoCarTimeStep), 'East_Right') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'West_Left') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'North_Right'))
                currentPedTrackData.isNearLane(end) = true;
            else
                currentPedTrackData.isNearLane(end) = false;
            end
        end
    else
        Lane = "Left";
        if currentPedTrackData.closeCar_ind(end) ~=0 && currentPedTrackData.closeCar_ind(end) ~=inf
            if (strcmp(egoCarData.car_lane(egoCarTimeStep), 'East_Right') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'West_Right') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'North_Left') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'South_Right') )
                currentPedTrackData.isNearLane(end) = true;
            else
                currentPedTrackData.isNearLane(end) = false;
            end
        end
    end


end

% update lane
currentPedTrackData.Lane(end) = Lane;