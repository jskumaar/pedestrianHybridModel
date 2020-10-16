%% plot descriptives

% % a) addpath of necessary directories
p1 = genpath('G:\My Drive\Research\Projects\pedestrianHybridModel\codes');
p2 = genpath('G:\My Drive\Research\Projects\pedestrianHybridModel\datasets');

addpath(p1)
addpath(p2)

load('GapData_12Scenes_v6.mat');

variable = GapFeatures.ped_close_cw;
variableName = 'ped_close_cw';


figure()





