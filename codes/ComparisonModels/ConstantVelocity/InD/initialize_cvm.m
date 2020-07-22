function datasets = initialize_cvm()

%parameters
global pred_horizon obs_horizon;

pred_horizon = 4.8; % seconds
obs_horizon  = 0.1;  % seconds


%include path of datasets
datasets(1).path = 'G:\My Drive\Research\MAVRIC_Summer_2020\Publicly available Data Sets\inD\inD-dataset-v1.0\data\';
datasets(1).name = 'inD'; 
datasets(1).deltaT = 0.04;
datasets(1).pred_horizon = [];
datasets(1).obs_horizon = [];
datasets(1).trackData = {};
datasets(1).class = '';

datasets(1).cvm_ADE = [];
datasets(1).cvm_FDE = [];
% datasets(1).cvm_ADE_truck = [];
% datasets(1).cvm_FDE_truck = [];
% datasets(1).cvm_ADE_ped = [];
% datasets(1).cvm_FDE_ped = [];
% datasets(1).cvm_ADE_cycle = [];
% datasets(1).cvm_FDE_cycle = [];

datasets(1).mean_cvm_ADE = [];
datasets(1).mean_cvm_FDE = [];
% datasets(1).mean_cvm_ADE_truck = [];
% datasets(1).mean_cvm_FDE_truck = [];
% datasets(1).mean_cvm_ADE_ped = [];
% datasets(1).mean_cvm_FDE_ped = [];
% datasets(1).mean_cvm_ADE_cycle = [];
% datasets(1).mean_cvm_FDE_cycle = [];

% datasets(2).path = 'G:\My Drive\Research\MAVRIC_Summer_2020\Publicly available Data Sets\INTERACTION\recorded_trackfiles';
% datasets(2).name = 'INTERACTION'; 
% datasets(2).deltaT = 0.1;

end