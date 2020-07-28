%% Created 06/10/2020: This file runs a constant velocity model on the heterogeneous datasets

       

% 
clear all
datasets = initialize_cvm();
N_datasets = size(datasets,1);

for ii=1:1
   % read dataset data
   datasets(ii) = read_data(datasets(ii));
   
   % run cvm predictions and calculate error
   datasets(ii) = cvm(datasets(ii));
   
   % display results
   disp(datasets(ii).name)
   disp(strcat('ADE:', num2str(datasets(ii).mean_cvm_ADE)));
   disp(strcat('FDE:', num2str(datasets(ii).mean_cvm_FDE)));
end


%% find errors based on agent type, movement, etc.
% this code needs to be moved inside a function later.


N_tracks = size(datasets.trackData,1);

%% error for tracks less than 120 (4.8 s) must be NaN

for ii=1:N_tracks
   if size(datasets(1).trackData{ii},1) < 120 
       datasets(1).cvm_ADE(ii) = NaN;
       datasets(1).cvm_FDE(ii) = NaN;
   end   
end







N_car = 1;
N_truck = 1;
N_ped = 1;
N_cycle = 1;
for ii=1:N_tracks
    if strcmp(datasets(1).class{ii},'car')
        datasets(1).cvm_ADE_agent_car(N_car,:) = [ii,datasets(1).cvm_ADE(ii), mean(datasets(1).trackData{ii}.lonVelocity(1:end)), mean(datasets(1).trackData{ii}.lonAcceleration(1:end))];
        N_car = N_car +1;
    end
    
    if strcmp(datasets(1).class{ii},'pedestrian')
        datasets(1).cvm_ADE_agent_ped(N_ped,:) = [ii,datasets(1).cvm_ADE(ii), mean(datasets(1).trackData{ii}.lonVelocity(1:end)), mean(datasets(1).trackData{ii}.lonAcceleration(1:end))];
        N_ped = N_ped +1;
    end   
end


% figures
ms = 6;


start_ind = 1;
end_ind = 50;

figure(1)
plot(datasets(1).cvm_ADE_agent_ped(start_ind:end_ind,2),'k--'); hold on;
plot(datasets(1).cvm_ADE_agent_ped(start_ind:end_ind,3),'b--'); hold on;
plot(datasets(1).cvm_ADE_agent_ped(start_ind:end_ind,4),'r--');
legend('error','velocity','acceleration')



