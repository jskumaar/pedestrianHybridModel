%% This script compiles the Gap features for batch processing

% parameters
dec_zone = 5; % in m
N_scenes=12;
AdjustedSampFreq = 5;

%initialize
gap_ind = 1;
GapFeatures = table();


for scene_id = 1:N_scenes

    tracksMetaDataScene = tracksMetaData{scene_id};
    
    car_tracks = tracks{scene_id}.car_tracks;
    car_parked_tracks = tracks{scene_id}.car_parked_tracks;
    ped_crossing_tracks = tracks{scene_id}.ped_crossing_tracks;
    ped_not_crossing_tracks = tracks{scene_id}.ped_not_crossing_tracks;
    car_moving_tracks = car_tracks;
    [~,ind] = intersect(car_tracks, car_parked_tracks);
    car_moving_tracks(ind) = [];
    all_ped_tracks = [ped_crossing_tracks; ped_not_crossing_tracks];
    
    
    for ped_ind = 1:size(all_ped_tracks,1)

        ped_track = all_ped_tracks(ped_ind);
        ego_veh_gap_hist = [];
        % for feature calculation
        pedData = formattedTracksData{scene_id}{ped_track};
        ped_vel = [pedData.xVelocity, pedData.yVelocity];

         % at every time step of the pedestrian trajectory
        for time_step = 1:size(formattedTracksData{scene_id}{ped_track},1)
            pedFrame = pedData.frame(time_step);
            pedPosPixels = double([pedData.xCenterPix(time_step), pedData.yCenterPix(time_step)]);
            cw_ped = pedData.closestCW(time_step);
            closeCar_ind = pedData.closeCar_ind(time_step);
            
            if closeCar_ind~=0 && closeCar_ind~=inf
            
                carData = formattedTracksData{scene_id}{closeCar_ind};
                car_ts = find(carData.frame == pedFrame); 

                % is this a new gap? i.e. a new car when the
                % pedestrian is approaching the crosswalk?

                if time_step > 1
    %                 if ( closeCar_ind ~= pedData.closeCar_ind(time_step-1) && closeCar_ind ~= inf && ...
    %                      ( strcmp(pedData.HybridState{time_step},'Approach') || strcmp(pedData.HybridState{time_step},'Wait') ) )

                GapCond_1 = pedData.closeCar_ind(time_step) ~= pedData.closeCar_ind(time_step-1) && ...
                            abs(pedData.long_disp_ped_cw(time_step)) < dec_zone && pedData.closeCar_ind(time_step) ~= inf && ...
                            ( strcmp(pedData.HybridState{time_step},'Approach') || strcmp(pedData.HybridState{time_step},'Wait') );
                % There is an ego-vehicle and pedestrian has just entered the decision zone
                GapCond_2 = abs(pedData.long_disp_ped_cw(time_step-1)) >= dec_zone && ...
                            abs(pedData.long_disp_ped_cw(time_step)) < dec_zone && ...
                            pedData.closeCar_ind(time_step) ~= inf && ( strcmp(pedData.HybridState{time_step},'Approach') || strcmp(pedData.HybridState{time_step},'Wait') );


                    if ( GapCond_1 || GapCond_2 ) 

                            if isempty(intersect(closeCar_ind, ego_veh_gap_hist))
                                pedData.Gap_start(time_step) = true;
                                ego_veh_gap_hist = [ego_veh_gap_hist; closeCar_ind];

                                %features for Gap
                                % calculate the features for the hybrid system model
                                % 1) speed of the pedestrian in the last one sec
                                if time_step >= AdjustedSampFreq
                                    F_pedSpeed = mean(sqrt(ped_vel(time_step - AdjustedSampFreq+1:time_step, 1).^2 + ped_vel(time_step-AdjustedSampFreq+1 : time_step, 2).^2));
                                else
                                    F_pedSpeed = mean(sqrt(ped_vel(1:time_step, 1).^2 + ped_vel(1:time_step, 2).^2));
                                end

                                % 2) lateral distance to road - already calculated

                                % 3) longitudinal distance to vehicle - already calculated

                                % 4) cumulative waiting time
                                F_cumWait = pedData.wait_time_steps(end);
                                % 5) vehicle speed
                                if ( pedData.closeCar_ind(time_step)~=inf)
                                    F_vehVel = [formattedTracksData{scene_id}{closeCar_ind}.lonVelocity(car_ts)];
                                    F_vehAcc = [formattedTracksData{scene_id}{closeCar_ind}.lonAcceleration(car_ts)];
                                else
                                    F_vehVel = inf;
                                    F_vehAcc = inf;
                                end
                                % 6) Gaze
                                F_gazeRatio = sum(pedData.isLooking(end-AdjustedSampFreq+1:end))/AdjustedSampFreq;


                                % Need to include Gaze later!!

                                % compile gap features
                                GapFeatures.recording(gap_ind,1) = pedData.recordingId(1);
                                GapFeatures.frame(gap_ind,1) = pedFrame;
                                GapFeatures.pedTrack(gap_ind,1) = ped_track;
                                GapFeatures.egoCarTrack(gap_ind,1) = closeCar_ind;
                                GapFeatures.ped_close_cw(gap_ind,1) = cw_ped;

                                GapFeatures.F_pedSpeed(gap_ind,1) = F_pedSpeed;
                                GapFeatures.F_cumWait(gap_ind,1) = F_cumWait;
                                GapFeatures.F_vehVel(gap_ind,1) = F_vehVel;
                                GapFeatures.F_vehAcc(gap_ind,1) = F_vehAcc;
                                GapFeatures.F_gazeRatio(gap_ind,1) = F_gazeRatio;
                                GapFeatures.F_pedDistToCurb(gap_ind,1) = pedData.lat_disp_ped_cw(time_step);                          
                                GapFeatures.F_pedDistToVeh(gap_ind,1) = pedData.long_disp_ped_car(time_step);
                                GapFeatures.F_pedDistToCW(gap_ind,1) = pedData.long_disp_ped_cw(time_step);                          
                                GapFeatures.F_isSameDirection(gap_ind,1) = pedData.isPedSameDirection(time_step);                          
                                
                                GapFeatures.F_isEgoNearLane(gap_ind,1) = pedData.isNearLane(time_step);

                                %update Gap id
                                gap_ind = gap_ind + 1;
                            end
                    end %end of inner gap checking loop

                    %has the pedestrian started crossing or started jaywalking?
                    if ( strcmp(pedData.HybridState{time_step},'Crossing') && ~strcmp(pedData.HybridState{time_step-1},'Crossing')  || ...
                         strcmp(pedData.HybridState{time_step},'Jaywalking') && ~strcmp(pedData.HybridState{time_step-1},'Jaywalking')   )

                        % if the pedestrian crossed for the ego-vehicle
                        % (closeCar_ind), whose gap started sometime
                        % back
                        GapFeatures_ind_temp1 = find(GapFeatures.recording == pedData.recordingId(1)); 
                        GapFeatures_ind_temp2 = find(GapFeatures.pedTrack  == ped_track); 
                        GapFeatures_ind_temp3 = find(GapFeatures.egoCarTrack == closeCar_ind); 

                        GapFeatures_ind = intersect(intersect(GapFeatures_ind_temp1,GapFeatures_ind_temp2), GapFeatures_ind_temp3);

                        if ~isempty(GapFeatures_ind)                                    
                            GapFeatures.CrossStart(GapFeatures_ind,1) = pedFrame;
                            % check whether the pedestrian crossed
                            % the street or jaywalked during this
                            % particular gap
                            if ( strcmp(pedData.HybridState{time_step},'Crossing') && ~strcmp(pedData.HybridState{time_step-1},'Crossing') )
                                GapFeatures.CrossDecision(GapFeatures_ind,1) = true;
                                GapFeatures.JaywalkDecision(GapFeatures_ind,1) = false;
                            else
                                GapFeatures.CrossDecision(GapFeatures_ind,1) = false;
                                GapFeatures.JaywalkDecision(GapFeatures_ind,1) = true;
                            end
                        end



                    end
                end
            end
        end
    end
    
end % end of outer gap checking loop