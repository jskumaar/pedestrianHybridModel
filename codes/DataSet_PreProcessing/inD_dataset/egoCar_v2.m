%% This script finds the descriptives of the tracks and the pedestrians


%formattedTracksData = tracksData;

%parameters
 images = [18:1:29];
delta_T = 1/25;
moving_threshold = 0.2;

annotatedImage = imread(strcat('annotated_',num2str(18),'_background.png'));        %using the same background image for all scenes as it roughly remains the same; also segmenting all images takes time.
% for better visualization, increase the grayscale value of the different regions
annotatedImage_enhanced = annotatedImage;
img_size = size(annotatedImage_enhanced);
for ii = 1:img_size(1)
    for jj = 1:img_size(2)
        if (annotatedImage(ii,jj)==2) %Road
            annotatedImage_enhanced(ii,jj) = 50;
        end
        if (annotatedImage(ii,jj)==4 || annotatedImage(ii,jj)==5 || annotatedImage(ii,jj)==6) %unmarked crosswalk
            annotatedImage_enhanced(ii,jj) = 100;
        end        
        if (annotatedImage(ii,jj)==3) %Sidewalk
            annotatedImage_enhanced(ii,jj) = 150;
        end        
        if (annotatedImage(ii,jj)==1) %marked crosswalk
            annotatedImage_enhanced(ii,jj) = 200;
        end
    end
end


img_center = int32([img_size(2)/2; -img_size(1)/2]);

%initialize
gap_ind = 1;
GapFeatures = table();
N_scenes = size(formattedTracksData, 2);

for scene_id = 1:N_scenes
    
    image_no = images(scene_id);
    
    car_tracks = tracks{scene_id}.car_tracks;
    car_parked_tracks = tracks{scene_id}.car_parked_tracks;
    ped_crossing_tracks = tracks{scene_id}.ped_crossing_tracks;
    ped_not_crossing_tracks = tracks{scene_id}.ped_not_crossing_tracks;
    car_moving_tracks = car_tracks;
    [~,ind] = intersect(car_tracks, car_parked_tracks);
    car_moving_tracks(ind) = [];
    all_ped_tracks = [ped_crossing_tracks; ped_not_crossing_tracks];
    
    tracksMetaDataScene = readtable(strcat(num2str(image_no),'_tracksMeta.csv'));
%    tracksMetaDataScene =  tracksMetaData{scene_id};
    max_track_size = max(tracksMetaDataScene.numFrames);

    %% 1) find the list of interacting cars for each pedestrian track
    for ped_ind = 1:size(all_ped_tracks,1)
        ped_track = all_ped_tracks(ped_ind);
        start_frame_ped =  tracksMetaDataScene.initialFrame(ped_track) + 1;
        end_frame_ped =  tracksMetaDataScene.finalFrame(ped_track) + 1;

        % find the indices of cars interacting with the pedestrian track
        interact_cars_temp = [];
        for car_ind = 1:size(car_moving_tracks,1)
            car_track = car_moving_tracks(car_ind);
            start_frame_car = tracksMetaDataScene.initialFrame(car_track) + 1;
            end_frame_car = tracksMetaDataScene.finalFrame(car_track) + 1;

            if (start_frame_car < end_frame_ped && end_frame_car > start_frame_ped)
                interact_cars_temp = [interact_cars_temp; car_track];
            end
        end
        interact_cars{ped_ind,1} = interact_cars_temp;
    end


    %% 2) find the closest vehicle at every time step and its parameters
    %rotation matrices for the different crosswalks
    cw1_rot = [cosd(-14), -sind(-14); sind(-14), cosd(-14)];
    cw2_rot = [cosd(-10), -sind(-10); sind(-10), cosd(-10)];
    cw3_rot = [cosd(-23), -sind(-23); sind(-23), cosd(-23)];
    cw4_rot = [cosd(-50), -sind(-50); sind(-50), cosd(-50)];

    for ped_ind = 1:size(all_ped_tracks,1)

        ped_track = all_ped_tracks(ped_ind);
        ego_veh_gap_hist = [];
        % for feature calculation
        pedData = formattedTracksData{scene_id}{ped_track};
        ped_vel = [pedData.xVelocity, pedData.yVelocity];
        % identify wait start time
        wait_start_time = find(strcmp(pedData.HybridState, 'Wait')==1,1,'first');

        % at every time step of the pedestrian trajectory
        for time_step = 1:size(formattedTracksData{scene_id}{ped_track},1)
            pedFrame = pedData.frame(time_step);
            pedPosPixels = double([pedData.xCenterPix(time_step), pedData.yCenterPix(time_step)]);
            cw_ped = pedData.closestCW(time_step);

            % intialize the pixel distances of the close car and the close car index
            long_disp_car_pixels = inf;
            lat_disp_cw_pixels = inf;
            closeCar_ind = inf;
            closeCar_ind_history = [];
            long_disp_ped_cw_pixels = inf;

            % for every car that is interacting with the pedestrian at this
            % time step     
            for car_ind = 1:size(interact_cars{ped_ind},1)

                interact_car_track = interact_cars{ped_ind}(car_ind);
                carData = formattedTracksData{scene_id}{interact_car_track};
                car_ts = find(carData.frame == pedFrame); 

                % if a common frame exists for both the car and the
                % pedestrian
                if ~isempty(car_ts)

                        cw_car = carData.closestCW(car_ts);
                        carPosPixels = double([carData.xCenterPix(car_ts), carData.yCenterPix(car_ts)]);    
                        
                        % to reduce noise in the estimation of heading because of noisy position and velocity data
                        y_vel = carData.yVelocity(car_ts);
                        if ( abs(y_vel) < moving_threshold )
                            y_vel = 0;
                        end
                        x_vel = carData.xVelocity(car_ts);
                        if ( abs(x_vel) < moving_threshold )
                            x_vel = 0;
                        end
                        % when vehicle is stopped, the heading should
                        % remain as the previous heading; else calculate
                        % new heading
                        if ~(x_vel==0 && y_vel==0)
                            car_heading = atan2(y_vel, x_vel)*180/pi;
                        end
                        %carVel = double([carData.xVelocity(car_ts), carData.yVelocity(car_ts)]);                                                          
                        %car_heading = atan2(double(carVel(2)), double(carVel(1)))*180/pi; 
                        
                        
                        % check if the pedestrian is heading in the same direction as the ego-vehicle or the opposite direction
                        if abs(ped_heading - car_heading) < 45    
                            isPedSameDirection = true;
                        else
                            isPedSameDirection = false;
                        end
                        

                        % crosswalk position in pixels
                        cw1_PosPixels = double([cw.center_x(1), cw.center_y(1)]);
                        cw2_PosPixels = double([cw.center_x(2), cw.center_y(2)]);
                        cw3_PosPixels = double([cw.center_x(3), cw.center_y(3)]);
                        cw4_PosPixels = double([cw.center_x(4), cw.center_y(4)]);

                        % distance between car and crosswalk (in pixels)
                        dist_cw1 = sqrt(double(cw.center_x(1) - carPosPixels(1))^2 + double(cw.center_y(1) - carPosPixels(2))^2);
                        dist_cw2 = sqrt(double(cw.center_x(2) - carPosPixels(1))^2 + double(cw.center_y(2) - carPosPixels(2))^2); 
                        dist_cw3 = sqrt(double(cw.center_x(3) - carPosPixels(1))^2 + double(cw.center_y(3) - carPosPixels(2))^2);
                        dist_cw4 = sqrt(double(cw.center_x(4) - carPosPixels(1))^2 + double(cw.center_y(4) - carPosPixels(2))^2);

                       % heading between car and crosswalk (in degrees)
                        car_cw1_angle = atan2(double([cw.center_y(1) - carPosPixels(2)]), double([cw.center_x(1) - carPosPixels(1)]))*180/pi;
                        car_cw2_angle = atan2(double([cw.center_y(2) - carPosPixels(2)]), double([cw.center_x(2) - carPosPixels(1)]))*180/pi;  
                        car_cw3_angle = atan2(double([cw.center_y(3) - carPosPixels(2)]), double([cw.center_x(3) - carPosPixels(1)]))*180/pi;  
                        car_cw4_angle = atan2(double([cw.center_y(4) - carPosPixels(2)]), double([cw.center_x(4) - carPosPixels(1)]))*180/pi;  

                        % initialize some variables
                        lat_disp_cw_pixels_temp = inf;
                        long_disp_ped_car_pixels_temp = inf;

                        % if the vehicle is close to crosswalk; % find the distance between the car and pedestrian based on the CW, the car is approaching
                        if (cw_ped==1 && (cw_car==1 || cw_car==2) )
                            carPosPixels_rot = int32(cw1_rot * (carPosPixels'  - double(img_center)) + double(img_center));
                            pedPosPixels_rot = int32(cw1_rot * (pedPosPixels'  - double(img_center)) + double(img_center));
                            cw1_PosPixels_rot = int32(cw1_rot * (cw1_PosPixels'  - double(img_center)) + double(img_center));
                            disp_ped_cw_pixels = pedPosPixels_rot - cw1_PosPixels_rot;
                            disp_ped_car_pixels = carPosPixels_rot - pedPosPixels_rot;
                            
                            lat_disp_cw_pixels_temp  = abs(disp_ped_cw_pixels(2)) - cw.center_lat_offset(1);
                            
                            if  ( strcmp(carData.car_lane{car_ts},'East_Right') || strcmp(carData.car_lane{car_ts},'West_Left') )
                                long_disp_ped_car_pixels_temp = disp_ped_car_pixels(1);
                                long_disp_ped_cw_pixels_temp = disp_ped_cw_pixels(1);
                            else
                                long_disp_ped_car_pixels_temp = -disp_ped_car_pixels(1);
                                long_disp_ped_cw_pixels_temp = -disp_ped_cw_pixels(1);
                            end
                        end

                        if (cw_ped==2 && (cw_car==1 || cw_car==2) )
                            carPosPixels_rot = int32(cw2_rot * (carPosPixels'  - double(img_center)) + double(img_center));
                            pedPosPixels_rot = int32(cw2_rot * (pedPosPixels'  - double(img_center)) + double(img_center)); 
                            cw2_PosPixels_rot = int32(cw2_rot * (cw2_PosPixels'  - double(img_center)) + double(img_center));
                            disp_ped_cw_pixels = pedPosPixels_rot - cw2_PosPixels_rot;
                            disp_ped_car_pixels = carPosPixels_rot - pedPosPixels_rot;
                            
                            lat_disp_cw_pixels_temp  = abs(disp_ped_cw_pixels(2)) - cw.center_lat_offset(2);
                            
                            if  ( strcmp(carData.car_lane{car_ts},'West_Right') || strcmp(carData.car_lane{car_ts},'East_Left') )
                                long_disp_ped_car_pixels_temp = -disp_ped_car_pixels(1);
                                long_disp_ped_cw_pixels_temp = -disp_ped_cw_pixels(1);
                            else
                                long_disp_ped_car_pixels_temp = disp_ped_car_pixels(1);
                                long_disp_ped_cw_pixels_temp = disp_ped_cw_pixels(1);
                            end
                        end

                        if (cw_ped==3 && (cw_car==3 || cw_car==4) )
                            carPosPixels_rot = int32(cw3_rot * (carPosPixels'  - double(img_center)) + double(img_center));
                            pedPosPixels_rot = int32(cw3_rot * (pedPosPixels'  - double(img_center)) + double(img_center)); 
                            cw3_PosPixels_rot = int32(cw3_rot * (cw3_PosPixels'  - double(img_center)) + double(img_center));
                            disp_ped_cw_pixels = pedPosPixels_rot - cw3_PosPixels_rot;
                            disp_ped_car_pixels = carPosPixels_rot - pedPosPixels_rot;
                            
                            lat_disp_cw_pixels_temp  = abs(disp_ped_cw_pixels(1)) - cw.center_lat_offset(3);
                            
                            if  ( strcmp(carData.car_lane{car_ts},'South_Right') || strcmp(carData.car_lane{car_ts},'North_Left') )
                                long_disp_ped_car_pixels_temp = -disp_ped_car_pixels(2);
                                long_disp_ped_cw_pixels_temp = -disp_ped_cw_pixels(2);
                            else
                                long_disp_ped_car_pixels_temp = disp_ped_car_pixels(2);
                                long_disp_ped_cw_pixels_temp = disp_ped_cw_pixels(2);
                            end
                        end

                        if (cw_ped==4 && (cw_car==3 || cw_car==4) )
                            carPosPixels_rot = int32(cw4_rot * (carPosPixels'  - double(img_center)) + double(img_center));
                            pedPosPixels_rot = int32(cw4_rot * (pedPosPixels'  - double(img_center)) + double(img_center)); 
                            cw4_PosPixels_rot = int32(cw4_rot * (cw4_PosPixels'  - double(img_center)) + double(img_center));
                            disp_ped_cw_pixels = pedPosPixels_rot - cw4_PosPixels_rot;
                            disp_ped_car_pixels = carPosPixels_rot - pedPosPixels_rot;
                            
                            lat_disp_cw_pixels_temp  = abs(disp_ped_cw_pixels(1)) - cw.center_lat_offset(4);
                            
                            if  ( strcmp(carData.car_lane{car_ts},'North_Right') || strcmp(carData.car_lane{car_ts},'South_Left') )
                                long_disp_ped_car_pixels_temp = disp_ped_car_pixels(2);
                                long_disp_ped_cw_pixels_temp = disp_ped_cw_pixels(2);
                            else
                                long_disp_ped_car_pixels_temp = -disp_ped_car_pixels(2);
                                long_disp_ped_cw_pixels_temp = -disp_ped_cw_pixels(2);
                            end
                        end
                       
                        % find the ego-vehicle:
                        % Is the car approaching the pedestrian?
                        % Is this car closer than the previous closest car?
                        if ( (long_disp_ped_car_pixels_temp > 0) && (long_disp_ped_car_pixels_temp < long_disp_car_pixels) )
                            long_disp_car_pixels = long_disp_ped_car_pixels_temp;
                            lat_disp_cw_pixels = lat_disp_cw_pixels_temp;
                            closeCar_ind = interact_car_track;
                            long_disp_ped_cw_pixels = long_disp_ped_cw_pixels_temp;
                        end                    

                        % save the variables
                        pedData.closeCar_ind(time_step) = closeCar_ind;
                        pedData.long_disp_ped_car_pixels(time_step) = long_disp_car_pixels;
                        pedData.lat_disp_ped_cw_pixels(time_step) = lat_disp_cw_pixels;
                        pedData.cw_car(time_step) = cw_car;
                        pedData.long_disp_ped_cw_pixels(time_step) = long_disp_ped_cw_pixels;
                        pedData.isPedSameDirection(time_step) = isPedSameDirection;
                        pedData.calcPedHeading(time_step) = ped_heading;
                        pedData.calcCarHeading(time_step) = car_heading;                     
                        
                        
                        
%                         % is this a new gap? i.e. a new car when the
%                         % pedestrian is approaching the crosswalk?
%                         if time_step > 1
%                             if ( closeCar_ind ~= pedData.closeCar_ind(time_step-1) && closeCar_ind ~= inf && ...
%                                  ( strcmp(pedData.HybridState{time_step},'Approach') || strcmp(pedData.HybridState{time_step},'Wait') ) )
%                                 
%                                 if isempty(intersect(closeCar_ind, ego_veh_gap_hist))
%                                         pedData.Gap_start(time_step) = true;
%                                         ego_veh_gap_hist = [ego_veh_gap_hist; closeCar_ind];
% 
%                                         %features for Gap
%                                         % calculate the features for the hybrid system model
%                                         % 1) speed of the pedestrian in the last one sec
%                                         if time_step > 1/ts
%                                             F_pedSpeed = mean(sqrt(ped_vel(time_step - 1/ts:time_step, 1).^2 + ped_vel(time_step-1/ts : time_step, 2).^2));
%                                         else
%                                             F_pedSpeed = mean(sqrt(ped_vel(1:time_step, 1).^2 + ped_vel(1:time_step, 2).^2));
%                                         end
% 
%                                         % 2) lateral distance to road - already calculated
% 
%                                         % 3) longitudinal distance to vehicle - already calculated
% 
%                                         % 4) cumulative waiting time
%                                         if ~isempty(wait_start_time)
%                                             if (strcmp(pedData.HybridState{time_step}, 'Wait'))
%                                                 F_cumWait = (time_step - wait_start_time)*delta_T;
%                                             else
%                                                 F_cumWait = 0;
%                                             end
%                                         else
%                                             F_cumWait = 0;
%                                         end
% 
%                                         % 5) vehicle speed
%                                         if ( pedData.closeCar_ind(time_step)~=inf)
%                                             F_vehVel = [formattedTracksData{image_id}{closeCar_ind}.lonVelocity(car_ts)];
%                                             F_vehAcc = [formattedTracksData{image_id}{closeCar_ind}.lonAcceleration(car_ts)];
%                                         else
%                                             F_vehVel = inf;
%                                             F_vehAcc = inf;
%                                         end
%                                         pedData.F_pedSpeed(time_step) = F_pedSpeed;
%                                         pedData.F_cumWait(time_step) = F_cumWait;
%                                         %pedData.F_vehVel(time_step) = F_vehVel;
%                                         
%                                         
%                                         % Need to include Gaze later!!
%                                         
%                                         % compile gap features
%                                         GapFeatures.recording(gap_ind,1) = pedData.recordingId(1);
%                                         GapFeatures.frame(gap_ind,1) = pedFrame;
%                                         GapFeatures.pedTrack(gap_ind,1) = ped_track;
%                                         GapFeatures.egoCarTrack(gap_ind,1) = closeCar_ind;
%                                         GapFeatures.F_pedSpeed(gap_ind,1) = F_pedSpeed;
%                                         GapFeatures.F_pedDistToCW(gap_ind,1) = long_disp_ped_cw_pixels;
%                                         GapFeatures.F_cumWait(gap_ind,1) = F_cumWait;
%                                         GapFeatures.F_pedDistToVeh(gap_ind,1) = long_disp_car_pixels;
%                                         GapFeatures.F_vehVel(gap_ind,1) = F_vehVel;
%                                         GapFeatures.F_pedDistToCurb(gap_ind,1) = lat_disp_cw_pixels;
%                                         GapFeatures.ped_close_cw(gap_ind,1) = cw_ped;
%                                         GapFeatures.F_vehAcc(gap_ind,1) = F_vehAcc;
%                                                                                                                         
%                                         %update Gap id
%                                         gap_ind = gap_ind + 1;
%                                 end
%                             end %end of inner gap checking loop
%                                                                       
%                             %has the pedestrian started crossing or started jaywalking?
%                             if ( strcmp(pedData.HybridState{time_step},'Crossing') && ~strcmp(pedData.HybridState{time_step-1},'Crossing')  || ...
%                                  strcmp(pedData.HybridState{time_step},'Jaywalking') && ~strcmp(pedData.HybridState{time_step-1},'Jaywalking')   )
% 
%                                 % if the pedestrian crossed for the ego-vehicle
%                                 % (closeCar_ind), whose gap started sometime
%                                 % back
%                                 GapFeatures_ind_temp1 = find(GapFeatures.recording == pedData.recordingId(1)); 
%                                 GapFeatures_ind_temp2 = find(GapFeatures.pedTrack  == ped_track); 
%                                 GapFeatures_ind_temp3 = find(GapFeatures.egoCarTrack == closeCar_ind); 
% 
%                                 GapFeatures_ind = intersect(intersect(GapFeatures_ind_temp1,GapFeatures_ind_temp2), GapFeatures_ind_temp3);
% 
%                                 if ~isempty(GapFeatures_ind)                                    
%                                     GapFeatures.CrossStart(GapFeatures_ind,1) = pedFrame;
%                                     GapFeatures.CrossCW(GapFeatures_ind,1) = cw_ped;
%                                     % check whether the pedestrian crossed
%                                     % the street or jaywalked during this
%                                     % particular gap
%                                     if ( strcmp(pedData.HybridState{time_step},'Crossing') && ~strcmp(pedData.HybridState{time_step-1},'Crossing') )
%                                         GapFeatures.CrossDecision(GapFeatures_ind,1) = true;
%                                         GapFeatures.JaywalkDecision(GapFeatures_ind,1) = false;
%                                     else
%                                         GapFeatures.CrossDecision(GapFeatures_ind,1) = false;
%                                         GapFeatures.JaywalkDecision(GapFeatures_ind,1) = true;
%                                     end
%                                 end
%                             end  % end of loop checking for crossing or jaywalking
%                         end % end of outer gap checking loop
                  

                end  %end of car loop for one car
      
            end % end of all car loops

        end % end of loop for all time steps of one pedestrians

        formattedTracksData{scene_id}{ped_track} = pedData;
    end  % end of loop for all pedestrians in a scene

end % end of loop for all scenes






