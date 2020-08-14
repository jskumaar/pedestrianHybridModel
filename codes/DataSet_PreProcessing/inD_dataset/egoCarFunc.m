%% This script finds the index of the closest car (or the ego car)

function pedData = egoCarFunc(currentPedTrackData, activeCarTracksData, cw, annotatedImage_enhanced )

% Note: make sure only the current pedestrian data (and not history of
% data) is passed into the function.

%parameters
moving_threshold = 0.2; % m/s
walking_threshold = 0.1; % m/s
heading_threshold = 90; %90 degrees

img_size = size(annotatedImage_enhanced);
img_center = int32([img_size(2)/2; -img_size(1)/2]);

ped_heading = currentPedTrackData.calcHeading(end);

%% find the closest vehicle at every time step and its parameters
%rotation matrices for the different crosswalks
cw1_rot = [cosd(-14), -sind(-14); sind(-14), cosd(-14)];
cw2_rot = [cosd(-10), -sind(-10); sind(-10), cosd(-10)];
cw3_rot = [cosd(-23), -sind(-23); sind(-23), cosd(-23)];
cw4_rot = [cosd(-50), -sind(-50); sind(-50), cosd(-50)];

% intialize the pixel distances of the close car and the close car index
long_disp_car_pixels = inf;
lat_disp_cw_pixels = inf;
closeCar_ind = inf;
long_disp_ped_cw_pixels = inf;
isLooking = false;

% for every car that is interacting with the pedestrian at this
% time step     
for car_ind = 1:size(activeCarTracksData,1)
    cw_car = activeCarTracksData.closestCW(car_ind);
   
    
    carPosPixels = double([activeCarTracksData.xCenterPix(car_ind), activeCarTracksData.yCenterPix(car_ind)]);    
    pedPosPixels = double([currentPedTrackData.xCenterPix(end), currentPedTrackData.yCenterPix(end)]);    

    cw_ped = currentPedTrackData.closestCW(end);
    car_heading = activeCarTracksData.calcHeading(car_ind);
    
    % 1) car heading: to reduce noise in the estimation of heading because of noisy position and velocity data
    y_vel = activeCarTracksData.yVelocity(car_ind);
    if ( abs(y_vel) < moving_threshold )
        y_vel = 0;
    end
    x_vel = activeCarTracksData.xVelocity(car_ind);
    if ( abs(x_vel) < moving_threshold )
        x_vel = 0;
    end
    % if the car is stopped, maintain the previous heading
    if x_vel~=0 || y_vel~=0 
        car_heading = atan2(y_vel, x_vel)*180/pi;
    end

    % 2) pedestrian heading
    y_pedVel = currentPedTrackData.yVelocity(end);
    if ( abs(y_pedVel) < walking_threshold )
        y_pedVel = 0;
    end
    x_pedVel = currentPedTrackData.xVelocity(end);
    if ( abs(x_pedVel) < walking_threshold )
        x_pedVel = 0;
    end
    % if the pedestrian is stopped, maintain the previous heading
    if x_pedVel~=0 || y_pedVel~=0 
        ped_heading = atan2(y_pedVel, x_pedVel)*180/pi; 
    end
    
    
    
    % crosswalk position in pixels
    cw1_PosPixels = double([cw.center_x(1), cw.center_y(1)]);
    cw2_PosPixels = double([cw.center_x(2), cw.center_y(2)]);
    cw3_PosPixels = double([cw.center_x(3), cw.center_y(3)]);
    cw4_PosPixels = double([cw.center_x(4), cw.center_y(4)]);
    
    % angle between pedestrian and the crosswalks
    ped_cw1_angle = atan2(double([cw.center_y(1) - pedPosPixels(2)]), double([cw.center_x(1) - pedPosPixels(1)]))*180/pi;
    ped_cw2_angle = atan2(double([cw.center_y(2) - pedPosPixels(2)]), double([cw.center_x(2) - pedPosPixels(1)]))*180/pi;  
    ped_cw3_angle = atan2(double([cw.center_y(3) - pedPosPixels(2)]), double([cw.center_x(3) - pedPosPixels(1)]))*180/pi;  
    ped_cw4_angle = atan2(double([cw.center_y(4) - pedPosPixels(2)]), double([cw.center_x(4) - pedPosPixels(1)]))*180/pi;  


    % initialize some variables
    lat_disp_cw_pixels_temp = inf;
    long_disp_ped_car_pixels_temp = inf;
    
    
    % check if the pedestrian is looking at the car
     isLooking_temp = gazeCheck(carPosPixels, pedPosPixels, car_heading, ped_heading);
     isLooking = isLooking || isLooking_temp;
     
    % check if the pedestrian is heading in the same direction as the ego-vehicle or the opposite direction
    if abs(ped_heading - car_heading) < 45    
        isPedSameDirection = true;
    else
        isPedSameDirection = false;
    end
        
     
     
     
    % if the vehicle is close to crosswalk; % find the distance between the car and pedestrian based on the CW, the car is approaching
    if (cw_ped==1 && (cw_car==1 || cw_car==2) )
        carPosPixels_rot = int32(cw1_rot * (carPosPixels'  - double(img_center)) + double(img_center));
        pedPosPixels_rot = int32(cw1_rot * (pedPosPixels'  - double(img_center)) + double(img_center));
        cw1_PosPixels_rot = int32(cw1_rot * (cw1_PosPixels'  - double(img_center)) + double(img_center));
        disp_ped_cw_pixels = pedPosPixels_rot - cw1_PosPixels_rot;
        disp_ped_car_pixels = carPosPixels_rot - pedPosPixels_rot;
        
        % lateral distance to cw center
        lat_disp_cw_pixels_temp  = abs(disp_ped_cw_pixels(2)) - cw.center_lat_offset(1);
        % longitudinal displacement to ego-car; later cars that have passed the
        % pedestrian are discarded.
        if  ( strcmp(activeCarTracksData.car_lane{car_ind},'East_Right') || strcmp(activeCarTracksData.car_lane{car_ind},'West_Left') )
            long_disp_ped_car_pixels_temp = disp_ped_car_pixels(1);
        else
            long_disp_ped_car_pixels_temp = -disp_ped_car_pixels(1);
        end
        % longitudinal displacement to cw center depending on whether the
        % ped is approaching or walking away from the crosswalk
        if  ( abs(ped_cw1_angle - ped_heading) <= heading_threshold )
            long_disp_ped_cw_pixels_temp = abs(disp_ped_cw_pixels(1));
        else
            long_disp_ped_cw_pixels_temp = -abs(disp_ped_cw_pixels(1));
        end
    end

    if (cw_ped==2 && (cw_car==1 || cw_car==2) )
        carPosPixels_rot = int32(cw2_rot * (carPosPixels'  - double(img_center)) + double(img_center));
        pedPosPixels_rot = int32(cw2_rot * (pedPosPixels'  - double(img_center)) + double(img_center)); 
        cw2_PosPixels_rot = int32(cw2_rot * (cw2_PosPixels'  - double(img_center)) + double(img_center));
        disp_ped_cw_pixels = pedPosPixels_rot - cw2_PosPixels_rot;
        disp_ped_car_pixels = carPosPixels_rot - pedPosPixels_rot;

        lat_disp_cw_pixels_temp  = abs(disp_ped_cw_pixels(2)) - cw.center_lat_offset(2);

        if  ( strcmp(activeCarTracksData.car_lane{car_ind},'West_Right') || strcmp(activeCarTracksData.car_lane{car_ind},'East_Left') )
            long_disp_ped_car_pixels_temp = -disp_ped_car_pixels(1);
        else
            long_disp_ped_car_pixels_temp = disp_ped_car_pixels(1);
        end
        
        % longitudinal displacement to cw center depending on whether the
        % ped is approaching or walking away from the crosswalk
        if  ( abs(ped_cw2_angle - ped_heading) <= heading_threshold )
            long_disp_ped_cw_pixels_temp = abs(disp_ped_cw_pixels(1));
        else
            long_disp_ped_cw_pixels_temp = -abs(disp_ped_cw_pixels(1));
        end
    end

    if (cw_ped==3 && (cw_car==3 || cw_car==4) )
        carPosPixels_rot = int32(cw3_rot * (carPosPixels'  - double(img_center)) + double(img_center));
        pedPosPixels_rot = int32(cw3_rot * (pedPosPixels'  - double(img_center)) + double(img_center)); 
        cw3_PosPixels_rot = int32(cw3_rot * (cw3_PosPixels'  - double(img_center)) + double(img_center));
        disp_ped_cw_pixels = pedPosPixels_rot - cw3_PosPixels_rot;
        disp_ped_car_pixels = carPosPixels_rot - pedPosPixels_rot;

        lat_disp_cw_pixels_temp  = abs(disp_ped_cw_pixels(1)) - cw.center_lat_offset(3);

        if  ( strcmp(activeCarTracksData.car_lane{car_ind},'South_Right') || strcmp(activeCarTracksData.car_lane{car_ind},'North_Left') )
            long_disp_ped_car_pixels_temp = -disp_ped_car_pixels(2);
        else
            long_disp_ped_car_pixels_temp = disp_ped_car_pixels(2);
        end
        
        % longitudinal displacement to cw center depending on whether the
        % ped is approaching or walking away from the crosswalk
        if  ( abs(ped_cw3_angle - ped_heading) <= heading_threshold )
            long_disp_ped_cw_pixels_temp = abs(disp_ped_cw_pixels(2));
        else
            long_disp_ped_cw_pixels_temp = -abs(disp_ped_cw_pixels(2));
        end
        
    end

    if (cw_ped==4 && (cw_car==3 || cw_car==4) )
        carPosPixels_rot = int32(cw4_rot * (carPosPixels'  - double(img_center)) + double(img_center));
        pedPosPixels_rot = int32(cw4_rot * (pedPosPixels'  - double(img_center)) + double(img_center)); 
        cw4_PosPixels_rot = int32(cw4_rot * (cw4_PosPixels'  - double(img_center)) + double(img_center));
        disp_ped_cw_pixels = pedPosPixels_rot - cw4_PosPixels_rot;
        disp_ped_car_pixels = carPosPixels_rot - pedPosPixels_rot;

        lat_disp_cw_pixels_temp  = abs(disp_ped_cw_pixels(1)) - cw.center_lat_offset(4);

        if  ( strcmp(activeCarTracksData.car_lane{car_ind},'North_Right') || strcmp(activeCarTracksData.car_lane{car_ind},'South_Left') )
            long_disp_ped_car_pixels_temp = disp_ped_car_pixels(2);
        else
            long_disp_ped_car_pixels_temp = -disp_ped_car_pixels(2);
        end
               
        % longitudinal displacement to cw center depending on whether the
        % ped is approaching or walking away from the crosswalk
        if  ( abs(ped_cw4_angle - ped_heading) <= heading_threshold )
            long_disp_ped_cw_pixels_temp = abs(disp_ped_cw_pixels(2));
        else
            long_disp_ped_cw_pixels_temp = -abs(disp_ped_cw_pixels(2));
        end
    end

    % find the ego-vehicle:
    % Is the car approaching the pedestrian? Do not consider cars that have
    % crossed the pedestrian.
    % Is this car closer than the previous closest car?
    if ( (long_disp_ped_car_pixels_temp > 0) && (long_disp_ped_car_pixels_temp < long_disp_car_pixels) )
        long_disp_car_pixels = long_disp_ped_car_pixels_temp;
        lat_disp_cw_pixels = lat_disp_cw_pixels_temp;
        closeCar_ind = car_ind;
        long_disp_ped_cw_pixels = long_disp_ped_cw_pixels_temp;
    end                    

end  % end of loop for all active cars interacting with the pedestrian at this time step


% save the variables
pedData.closeCar_ind = closeCar_ind;
pedData.long_disp_ped_car_pixels = long_disp_car_pixels;
pedData.lat_disp_ped_cw_pixels = lat_disp_cw_pixels;
pedData.cw_car = cw_car;
pedData.long_disp_ped_cw_pixels = long_disp_ped_cw_pixels;
pedData.isLooking = isLooking;
pedData.calcPedHeading = ped_heading;
pedData.calcCarHeading = car_heading;
pedData.isPedSameDirection = isPedSameDirection;

end  % end of the function





