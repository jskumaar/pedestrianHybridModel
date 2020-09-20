function CrossFeatures =  compileCrossFeatures(currentPedData, ped_track_time_step, Params, varargin)
              
AdjustedSampFreq = Params.AdjustedSampFreq;
observationWindow = Params.observationWindow * AdjustedSampFreq;


start_ind_ped = max(ped_track_time_step-observationWindow+1,1);
% copy features               
gaze = currentPedData.isLooking(start_ind_ped : ped_track_time_step);
DTCurb = double(currentPedData.latDispPedCw(start_ind_ped:ped_track_time_step));
DTCW = double(currentPedData.longDispPedCw(start_ind_ped:ped_track_time_step));
ped_speed = currentPedData.lonVelocity(start_ind_ped:ped_track_time_step);           
veh_ped_dist = double(currentPedData.long_disp_ped_car(start_ind_ped:ped_track_time_step));
isSamedirection = currentPedData.isPedSameDirection(start_ind_ped:ped_track_time_step);


if size(varargin)~=0
    carData = varargin{1};
    % car time step
    ped_track_time = currentPedData.frame(ped_track_time_step);
    car_ts = find(carData.frame == ped_track_time);
    
    start_ind = max(car_ts-observationWindow+1, 1);
    veh_speed = carData.lonVelocity(start_ind: car_ts);
    veh_acc = carData.lonAcceleration(start_ind : car_ts);
    isNearLane = currentPedData.isNearLane(start_ind_ped:ped_track_time_step);

%     % time steps within observation window for which there is an ego-car
%     ego_car_ind = find(currentPedData.closeCar_ind(start_ind_ped : ped_track_time_step)~=0 |...
%                        currentPedData.closeCar_ind(start_ind_ped : ped_track_time_step)~=inf);

    % time steps within observation window for which there is an ego-car
    close_car_ind = find(currentPedData.closeCar_ind(start_ind_ped : ped_track_time_step)~=0 &...
                       currentPedData.closeCar_ind(start_ind_ped : ped_track_time_step)~=inf);
                   
    ego_car_ind = find(veh_ped_dist~=0 & veh_ped_dist~=inf);
                   
     
    % compile vehicle features

    % time steps when there is an ego-car
    CrossFeatures.duration_ego_vehicle = length(close_car_ind)/observationWindow;
    % time steps where the pedestrian and car is within the observation zone of
    % the AV (not implemented)

    CrossFeatures.mean_veh_speed = mean(veh_speed);
    CrossFeatures.std_veh_speed = std(veh_speed);                             

    CrossFeatures.mean_veh_acc = mean(veh_acc);
    CrossFeatures.std_veh_acc = std(veh_acc); 

    CrossFeatures.gaze_ratio = sum(gaze(close_car_ind))/length(close_car_ind);
    CrossFeatures.isSameDirection = mean(isSamedirection(end-AdjustedSampFreq+1:end))>0.5;
    CrossFeatures.isNearLane = mean(isNearLane(end-AdjustedSampFreq+1:end))>0.5;

    CrossFeatures.mean_veh_ped_dist = mean(veh_ped_dist(ego_car_ind));
    CrossFeatures.std_veh_ped_dist = std(veh_ped_dist(ego_car_ind));    
end




ped_ind = find(DTCurb~=0 & DTCW~=0);

% compile pedestrian features and scene data
CrossFeatures.mean_ped_speed = mean(ped_speed(ped_ind));
CrossFeatures.std_ped_speed = std(ped_speed(ped_ind));

CrossFeatures.mean_DTCurb = mean(DTCurb(ped_ind));
CrossFeatures.std_DTCurb = std(DTCurb(ped_ind));

CrossFeatures.mean_DTCW = mean(DTCW(ped_ind));
CrossFeatures.std_DTCW = std(DTCW(ped_ind));

CrossFeatures.scene_id = currentPedData.recordingId(1);
CrossFeatures.track_id = currentPedData.trackId(1);

end