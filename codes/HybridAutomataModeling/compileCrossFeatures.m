% Note: Features are calculted for the observation duration. If the
% prediction is somewhere after the 1st time step, the actual observations are
% augmented with the predictions made so far.

% Note: We assume that the No. of tracklets is not going to go above 5 for
% a given prediction horizon!


function [CrossFeatures, flag] =  compileCrossFeatures(currentPedData, trackletData, trackletNo, trackletStartNode, trackletEndNode, pedTrackTimeStep, predTimeStep, Params, flag, varargin)

%% 1) setup
% parameters
AdjustedSampFreq = Params.AdjustedSampFreq;
observationWindow = Params.observationWindow;
% indices
start_ind_ped_data = max(pedTrackTimeStep-observationWindow+predTimeStep ,1);
gaze_start_ind = max(pedTrackTimeStep-observationWindow+1, 1);


% copy the data based on the prediction tracklets
N_tracklets = size(trackletData,1);
% initialize trajectory
predictionTrajectory{1,1}(1,1) = trackletEndNode(1);

tempTraj = [trackletData{1}.xCenter, trackletData{1}.yCenter]';
predictionTrajectory{1,1} = [predictionTrajectory{1,1}, tempTraj(:)'];

% initialize variables with actual observed data till the time step
gaze{1} = currentPedData.isLooking(gaze_start_ind : pedTrackTimeStep);  %gaze uses only observed information and not predicted information and assumed to stay constant within the prediction horizon 
DTCurb{1} = currentPedData.latDispPedCw(start_ind_ped_data:pedTrackTimeStep);
DTCW{1} = currentPedData.longDispPedCw(start_ind_ped_data:pedTrackTimeStep);
ped_speed{1} = currentPedData.lonVelocity(start_ind_ped_data:pedTrackTimeStep);
veh_ped_dist{1} = currentPedData.long_disp_ped_car(start_ind_ped_data:pedTrackTimeStep);
isSamedirection{1} = currentPedData.isPedSameDirection(start_ind_ped_data:pedTrackTimeStep);
isNearLane{1} = currentPedData.isNearLane(start_ind_ped_data:pedTrackTimeStep);
closeCar_ind{1} = currentPedData.closeCar_ind(start_ind_ped_data:pedTrackTimeStep);

% compile trajector information from tracklets
% first tracklet
DTCurb{1} = [DTCurb{1}; trackletData{1}.latDispPedCw];
DTCW{1} = [DTCW{1}; trackletData{1}.longDispPedCw];
ped_speed{1} = [ped_speed{1}; trackletData{1}.lonVelocity];           
veh_ped_dist{1} = [veh_ped_dist{1}; trackletData{1}.long_disp_ped_car];
isSamedirection{1} = [isSamedirection{1}; trackletData{1}.isPedSameDirection];
isNearLane{1} =  [isNearLane{1}; trackletData{1}.isNearLane];
closeCar_ind{1} = [closeCar_ind{1}; trackletData{1}.closeCar_ind]; 

trajectory_id = 1;
if trackletNo > 1
    % compile all the different prediction futures
    N_new = 1;
    temp_predcopying = zeros(N_tracklets,1);
    predCopying = true;
    while(predCopying)
          for ii=1:size(predictionTrajectory,1)
                nextTracklet = find(trackletStartNode==predictionTrajectory{ii}(1,1));
                ind = find(nextTracklet==trackletNo, 1);
                % identify which trajectory is relevant for the current
                % tracklet
                if ~isempty(ind)
                    trajectory_id = ii;
                end
                if (~isempty(nextTracklet))
                    temp_predcopying(ii) = 1;
                    for jj = length(nextTracklet):-1:1 %the first tracklet gets added to the first prediction
                       trackletId = nextTracklet(jj);
                       if jj~=1
                            N_new = N_new + 1;                                                       
                            predictionTrajectory{N_new,1} = predictionTrajectory{ii};
                            predictionTrajectory{N_new}(1,1) = trackletEndNode(trackletId);
                            tempTraj = [trackletData{trackletId}.xCenter, trackletData{trackletId}.yCenter]';
                            predictionTrajectory{N_new} = [predictionTrajectory{N_new}, tempTraj(:)']; %the first entry i
                            
                            DTCurb{N_new} = [DTCurb{ii}; trackletData{trackletId}.latDispPedCw];
                            DTCW{N_new}  = [DTCW{ii}; trackletData{trackletId}.longDispPedCw];
                            ped_speed{N_new}  = [ped_speed{ii}; trackletData{trackletId}.lonVelocity];           
                            veh_ped_dist{N_new}  = [veh_ped_dist{ii}; trackletData{trackletId}.long_disp_ped_car];
                            isSamedirection{N_new}  = [isSamedirection{ii}; trackletData{trackletId}.isPedSameDirection];
                            isNearLane{N_new} =  [isNearLane{ii}; trackletData{trackletId}.isNearLane];
                            closeCar_ind{N_new}  = [closeCar_ind{ii}; trackletData{trackletId}.closeCar_ind];                             
                       else
                            tempTraj = [trackletData{trackletId}.xCenter(1:end), trackletData{trackletId}.yCenter(1:end)]';
                            predictionTrajectory{ii} = [predictionTrajectory{ii}, tempTraj(:)'];
                            predictionTrajectory{ii}(1,1) = trackletEndNode(trackletId);
                           
                            DTCurb{ii} = [DTCurb{ii}; trackletData{trackletId}.latDispPedCw];
                            DTCW{ii} = [DTCW{ii}; trackletData{trackletId}.longDispPedCw];
                            ped_speed{ii} = [ped_speed{ii}; trackletData{trackletId}.lonVelocity];           
                            veh_ped_dist{ii} = [veh_ped_dist{ii}; trackletData{trackletId}.long_disp_ped_car];
                            isSamedirection{ii} = [isSamedirection{ii}; trackletData{trackletId}.isPedSameDirection];
                            isNearLane{ii} =  [isNearLane{ii}; trackletData{trackletId}.isNearLane];
                            closeCar_ind{ii} = [closeCar_ind{ii}; trackletData{trackletId}.closeCar_ind]; 
                       end

                    end
                else
                    temp_predcopying(ii) = 0;
                end
                % check if there are no more tracklets for any of the paths
                if sum(temp_predcopying)==0
                    predCopying = false;
                end
          end 
    end
end

% update metrics
N = length(DTCurb{trajectory_id});
startIndex = max(N-observationWindow+1, 1);

DTCurb_data = DTCurb{trajectory_id}(startIndex:end);
DTCW_data = DTCW{trajectory_id}(startIndex:end);
ped_speed_data = ped_speed{trajectory_id}(startIndex:end);         
veh_ped_dist_data = veh_ped_dist{trajectory_id}(startIndex:end);
isSamedirection_data = isSamedirection{trajectory_id}(startIndex:end);
isNearLane_data = isNearLane{trajectory_id}(startIndex:end);
closeCar_ind_data = closeCar_ind{trajectory_id}(startIndex:end);
gaze_data = gaze{1};

% for Id = 1:N_tracklets
%     if Id==1
%         predPedData = trackletData{Id};
%         DTCurb = [DTCurb; predPedData.latDispPedCw(2:end)];
%         DTCW = [DTCW; predPedData.longDispPedCw(2:end)];
%         ped_speed = [ped_speed; predPedData.lonVelocity(2:end)];           
%         veh_ped_dist = [veh_ped_dist; predPedData.long_disp_ped_car(2:end) ];
%         isSamedirection = [isSamedirection; predPedData.isPedSameDirection(2:end) ];
%         closeCar_ind = [closeCar_ind; predPedData.closeCar_ind(2:end) ];            
%     end
%     
%     if ( (Id==2 || Id==3) && Id==trackletNo)
%         predPedData = trackletData{Id};
%         DTCurb = [DTCurb; predPedData.latDispPedCw(2:end)];
%         DTCW = [DTCW; predPedData.longDispPedCw(2:end)];
%         ped_speed = [ped_speed; predPedData.lonVelocity(2:end)];           
%         veh_ped_dist = [veh_ped_dist; predPedData.long_disp_ped_car(2:end) ];
%         isSamedirection = [isSamedirection; predPedData.isPedSameDirection(2:end) ];
%         closeCar_ind = [closeCar_ind; predPedData.closeCar_ind(2:end) ];  
%     end
%     
%     
%     if ( (Id==4 || Id==5) && Id==trackletNo)
%         
%         if length(trackletData{2}.trackLifetime) < length(trackletData{3}.trackLifetime)
%             predPedData = trackletData{2};
%             DTCurb = [DTCurb; predPedData.latDispPedCw(2:end)];
%             DTCW = [DTCW; predPedData.longDispPedCw(2:end)];
%             ped_speed = [ped_speed; predPedData.lonVelocity(2:end)];           
%             veh_ped_dist = [veh_ped_dist; predPedData.long_disp_ped_car(2:end) ];
%             isSamedirection = [isSamedirection; predPedData.isPedSameDirection(2:end) ];
%             closeCar_ind = [closeCar_ind; predPedData.closeCar_ind(2:end) ];  
%         else
%             predPedData = trackletData{3};
%             DTCurb = [DTCurb; predPedData.latDispPedCw(2:end)];
%             DTCW = [DTCW; predPedData.longDispPedCw(2:end)];
%             ped_speed = [ped_speed; predPedData.lonVelocity(2:end)];           
%             veh_ped_dist = [veh_ped_dist; predPedData.long_disp_ped_car(2:end) ];
%             isSamedirection = [isSamedirection; predPedData.isPedSameDirection(2:end) ];
%             closeCar_ind = [closeCar_ind; predPedData.closeCar_ind(2:end) ]; 
%         end
%         
%             predPedData = trackletData{Id};
%             DTCurb = [DTCurb; predPedData.latDispPedCw(2:end)];
%             DTCW = [DTCW; predPedData.longDispPedCw(2:end)];
%             ped_speed = [ped_speed; predPedData.lonVelocity(2:end)];           
%             veh_ped_dist = [veh_ped_dist; predPedData.long_disp_ped_car(2:end) ];
%             isSamedirection = [isSamedirection; predPedData.isPedSameDirection(2:end) ];
%             closeCar_ind = [closeCar_ind; predPedData.closeCar_ind(2:end) ]; 
% 
%     end
%     
% end
% 
% if N_tracklets>5
%     x=1;
% end

% use only the last observation window data


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 2) compile vehicle data (if available)
if size(varargin)~=0
    carData = varargin{1};
    carTrackCurrentTimeStep = varargin{2};
    
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
    % when the car observations are not available for the time steps in the
    % prediction, use the last N observations where N is the no. of time
    % steps that falls within the observation window for which actual
    % observed car data is available.

    % easier way
    carTimeStep = carTrackCurrentTimeStep + predTimeStep -1;
    start_ind = max(carTimeStep-observationWindow+1, 1);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
    % copy car data to separate variables
    veh_speed = carData.lonVelocity(start_ind: carTimeStep);
    veh_acc = carData.lonAcceleration(start_ind : carTimeStep);
    % time steps within observation window for which there is an ego-car
    close_car_data_indices = find(closeCar_ind_data~=0 & closeCar_ind_data~=inf);                  
    ego_car_ind = find(veh_ped_dist_data~=0 & veh_ped_dist_data~=inf);                  
    %%%%%%%%%%%%%%%%%%%%
    % compile vehicle features

    % time steps when there is an ego-car
    CrossFeatures.duration_ego_vehicle = length(close_car_data_indices)/observationWindow;
    %%%%%%%%%%%%%%%%%%%%
    % time steps where the pedestrian and car is within the observation zone of
    % the AV (not implemented)
    %%%%%%%%%%%%%%%%%%%%
    CrossFeatures.mean_veh_speed = mean(veh_speed);
%     CrossFeatures.std_veh_speed = std(veh_speed);                             
    CrossFeatures.mean_veh_acc = mean(veh_acc);
%     CrossFeatures.std_veh_acc = std(veh_acc); 
    CrossFeatures.gaze_ratio = sum(gaze_data)/length(gaze_data);
    CrossFeatures.isSameDirection = mean(isSamedirection_data)>0.5;
    CrossFeatures.isNearLane = mean(isNearLane_data)>0.5;
    CrossFeatures.mean_veh_ped_dist = mean(veh_ped_dist_data(ego_car_ind));
%     CrossFeatures.std_veh_ped_dist = std(veh_ped_dist(ego_car_ind));
    
    if isnan(mean(veh_ped_dist_data(ego_car_ind))) 
        flag.checkIntentWOEgo(trackletNo) = true;
    end
else
    CrossFeatures.mean_veh_speed = [];
    CrossFeatures.mean_veh_acc = [];
    CrossFeatures.mean_veh_ped_dist = [];
    CrossFeatures.gaze_ratio = [];
    CrossFeatures.isSameDirection = [];
    CrossFeatures.isNearLane = [];
    CrossFeatures.duration_ego_vehicle = [];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 3) compile pedestrian features and scene data
ped_ind = find(DTCurb_data~=0 & DTCW_data~=0);

CrossFeatures.mean_ped_speed = mean(ped_speed_data(ped_ind));
% CrossFeatures.std_ped_speed = std(ped_speed(ped_ind));

CrossFeatures.mean_DTCurb = mean(DTCurb_data(ped_ind));
% CrossFeatures.std_DTCurb = std(DTCurb(ped_ind));

CrossFeatures.mean_DTCW = mean(DTCW_data(ped_ind));
% CrossFeatures.std_DTCW = std(DTCW(ped_ind));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% debug
if isnan(mean(ped_speed_data(ped_ind))) || isnan(mean(DTCurb_data(ped_ind))) || isnan(mean(DTCW_data(ped_ind))) 
    x=1;
end
 


end