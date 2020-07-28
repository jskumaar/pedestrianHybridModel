function dataset_x = cvm(dataset_x)
    
    H = dataset_x.pred_horizon;
    N_tracks = size(dataset_x.trackData,1);
    
    ADE_allTracks = zeros(N_tracks,1);
    FDE_allTracks = zeros(N_tracks,1);

    for ii = 1:N_tracks
       trackData = dataset_x.trackData{ii};
       
       if size(trackData,1) > H          
           ADE_temp = zeros( size(trackData,1) - H, 1);
           FDE_temp = zeros( size(trackData,1) - H, 1);

           for jj = 1: size(trackData,1) - H
               obs_pos = [trackData.xCenter(jj), trackData.yCenter(jj)];
               obs_vel = [trackData.xVelocity(jj), trackData.yVelocity(jj)];
               A = tril(ones(H),-1) + eye(H);
               
               % prediction
               pred_pos = [ones(H,1).*obs_pos] + [A*[ones(H,1)*obs_vel(1)*dataset_x.deltaT], A*[ones(H,1)*obs_vel(2)*dataset_x.deltaT]];
               % ground truth
               gt_pos = [trackData.xCenter(jj+1:jj+H), trackData.yCenter(jj+1:jj+H)];
               % error
               error_pos = pred_pos - gt_pos;
               del_T = ceil(0.04/dataset_x.deltaT);
               error_pos = error_pos(del_T:del_T:end,:);
               
               ADE_temp(jj) = mean(sqrt(sum(error_pos.^2,2)));
               FDE_temp(jj) = sqrt(sum(error_pos(end).^2));

               
           end
           
           
%            % error metrics               
%            if strcmp(dataset_x.class{ii}{1}, 'car')
%                ADE_car = ADE_temp;
%                FDE_car = FDE_temp;
%            elseif strcmp(dataset_x.class{ii}{1}, 'truck')
%                ADE_truck = ADE_temp;
%                FDE_truck = FDE_temp;
%            elseif strcmp(dataset_x.class{ii}, 'pedestrian')
%                ADE_ped = ADE_temp;
%                FDE_ped = FDE_temp;
%            elseif strcmp(dataset_x.class{ii}, 'bicycle')
%                ADE_cycle = ADE_temp;
%                FDE_cycle = FDE_temp;
%            end
           
           dataset_x.cvm_ADE(ii) = mean(ADE_temp);
           dataset_x.cvm_FDE(ii) = mean(FDE_temp);
%            
%            dataset_x.cvm_ADE_car(ii) = mean(ADE_car);
%            dataset_x.cvm_FDE_car(ii) = mean(FDE_car);
%            dataset_x.cvm_ADE_truck(ii) = mean(ADE_truck);
%            dataset_x.cvm_FDE_truck(ii) = mean(FDE_truck);
%            dataset_x.cvm_ADE_ped(ii) = mean(ADE_ped);
%            dataset_x.cvm_FDE_ped(ii) = mean(FDE_ped);
%            dataset_x.cvm_ADE_cycle(ii) = mean(ADE_cycle);
%            dataset_x.cvm_FDE_cycle(ii) = mean(FDE_cycle);
%            
           ADE_allTracks(ii) = mean(ADE_temp);
           FDE_allTracks(ii) = mean(FDE_temp);
%            ADE_allTracks_truck = [ADE_allTracks_truck, mean(ADE_truck)];
%            FDE_allTracks_truck = [FDE_allTracks_truck, mean(FDE_truck)];
%            ADE_allTracks_ped = [ADE_allTracks_ped, mean(ADE_ped)];
%            FDE_allTracks_ped = [FDE_allTracks_ped, mean(FDE_ped)];
%            ADE_allTracks_cycle = [ADE_allTracks_cycle, mean(ADE_cycle)];
%            FDE_allTracks_cycle = [FDE_allTracks_cycle, mean(FDE_cycle)];
           
           
           
           
       end
       
    end
    
    dataset_x.mean_cvm_ADE = mean(ADE_allTracks);
    dataset_x.mean_cvm_FDE = mean(FDE_allTracks);
    
%     dataset_x.mean_cvm_ADE_car = mean(ADE_allTracks_car);
%     dataset_x.mean_cvm_FDE_car = mean(FDE_allTracks_car);
%     dataset_x.mean_cvm_ADE_truck = mean(ADE_allTracks_truck);
%     dataset_x.mean_cvm_FDE_truck = mean(FDE_allTracks_truck);
%     dataset_x.mean_cvm_ADE_ped = mean(ADE_allTracks_ped);
%     dataset_x.mean_cvm_FDE_ped = mean(FDE_allTracks_ped);
%     dataset_x.mean_cvm_ADE_cycle = mean(ADE_allTracks_cycle);
%     dataset_x.mean_cvm_FDE_cycle = mean(FDE_allTracks_cycle);


end


