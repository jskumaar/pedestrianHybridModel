%% This script finds on which lane the vehicle is in

function formattedTracksData = findLaneFunc(formattedTracksData, tracks, annotatedImage_enhanced, N_scenes, orthopxToMeter, cw)


%% find the lane of the cars
scale_down_factor = 12;

%% center of the road
img_size = size(annotatedImage_enhanced);
road_pixels = [];
for ii = 1:img_size(1)
    for jj = 1:img_size(2)
        if annotatedImage_enhanced(ii,jj)==50
            road_pixels = [road_pixels; [jj, -ii]];
        end
    end
end

road_center = int32(mean(road_pixels));
annotatedImage_enhanced(-road_center(2), road_center(1)) = 255;
%imshow(annotatedImage_enhanced)

%% loop
for image_id = 1:N_scenes
    car_tracks = tracks{image_id}.car_tracks;
    car_lane = [];
    
for car_ind = 1:size(car_tracks,1)
    for ii = 1:size(formattedTracksData{image_id}{car_tracks(car_ind),1},1)
        
        % convert to pixels       
        formattedTracksData{image_id}{car_tracks(car_ind),1}.xCenterPix = int32(formattedTracksData{image_id}{car_tracks(car_ind),1}.xCenter/(scale_down_factor*orthopxToMeter));
        formattedTracksData{image_id}{car_tracks(car_ind),1}.yCenterPix = int32(formattedTracksData{image_id}{car_tracks(car_ind),1}.yCenter/(scale_down_factor*orthopxToMeter));

        car_pos = [formattedTracksData{image_id}{car_tracks(car_ind),1}.xCenterPix(ii), formattedTracksData{image_id}{car_tracks(car_ind),1}.yCenterPix(ii)];
        car_cw_heading = atan2(double([road_center(2) - car_pos(2)]), double([road_center(1) - car_pos(1)]))*180/pi;
        car_heading = atan2(formattedTracksData{image_id}{car_tracks(car_ind),1}.yVelocity(ii), formattedTracksData{image_id}{car_tracks(car_ind),1}.xVelocity(ii))*180/pi;
       
       % distance between car and crosswalk (in pixels)
       dist_cw1 = sqrt(double(cw.center_x(1) - car_pos(1))^2 + double(cw.center_y(1) - car_pos(2))^2);
       dist_cw2 = sqrt(double(cw.center_x(2) - car_pos(1))^2 + double(cw.center_y(2) - car_pos(2))^2); 
       dist_cw3 = sqrt(double(cw.center_x(3) - car_pos(1))^2 + double(cw.center_y(3) - car_pos(2))^2);
       dist_cw4 = sqrt(double(cw.center_x(4) - car_pos(1))^2 + double(cw.center_y(4) - car_pos(2))^2);
  
        % conditions for lane
        % car heading is in 1st or 3rd quadrant
        if ( (car_heading > 0 && car_heading < 90) || (car_heading > -180 && car_heading < -90) )            
            % car is closer to crosswalk 1 or 2
            if dist_cw1 < dist_cw2              
                % car is heading towards the intersection or away from it
                % for crosswalk 1
                 if abs(car_heading-car_cw_heading) < 45
                     car_lane{car_ind}{ii,1} = 'East_Right';
                 else
                     car_lane{car_ind}{ii,1} = 'East_Left';
                 end
                 % note the closest cw and its distance
                 cw_dist(car_ind) = dist_cw1;
                 cw_ind(car_ind) = 1;
            else
                % car is heading towards the intersection or away from it
                % for crosswalk 2
                 if abs(car_heading-car_cw_heading) < 45
                     car_lane{car_ind}{ii,1} = 'West_Right';
                 else
                     car_lane{car_ind}{ii,1} = 'West_Left';
                 end               
                 cw_dist(car_ind) = dist_cw2;
                 cw_ind(car_ind) = 2;
            end           
        else
            % car is closer to crosswalk 3 or 4
            if dist_cw3 < dist_cw4              
                % car is heading towards the intersection or away from it
                % for crosswalk 3
                 if abs(car_heading-car_cw_heading) < 45
                     car_lane{car_ind}{ii,1} = 'South_Right';
                 else
                     car_lane{car_ind}{ii,1} = 'South_Left';
                 end   
                 cw_dist(car_ind) = dist_cw3;
                 cw_ind(car_ind) = 3;
            else
                % car is heading towards the intersection or away from it
                % for crosswalk 4
                 if abs(car_heading-car_cw_heading) < 45
                     car_lane{car_ind}{ii,1} = 'North_Right';
                 else
                     car_lane{car_ind}{ii,1} = 'North_Left';
                 end
                 cw_dist(car_ind) = dist_cw4;
                 cw_ind(car_ind) = 4;
            end
        end
        
        
    end
    car_tracks(car_ind)
    formattedTracksData{image_id}{car_tracks(car_ind),1}.car_lane = car_lane{car_ind};
    
    
end

end     % loop ends for all scenes


end     % function ends