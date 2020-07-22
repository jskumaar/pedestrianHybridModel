%% temp_for learning the t start and starting speed distributions
% this file finds the distributions of starting speed and starting time
% after waiting from the data


%% Updated 10/28/2019
% The distrbutions are calculated only from the data that is used for the
% SVM data (no high deceleration, no extreme waiting outliers).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all

%add path to the MATLAB variables and excel sheets
addpath('G:\my drive\Research\Pedestrian Modelling Project\Modelling Scripts and Results\Mat Data\')
addpath('G:\My Drive\Research\Pedestrian Modelling Project\Study I Data for Modeling\Compiled Data\');

%Read data
load('AllFeaturesCrossingWise_PW_5_SpeedHist.mat')
EventIndices = xlsread('DiscreteStateEventIndicesW5.xlsx');

%identify waiting indices
indexWaiting = [1:540]';
indexWaiting(EventIndices(:,6)==0)=[];

%remove the indices of high deceleration and extreme waiting times
high_deceleration_data = xlsread('High_acceleration_data.xlsx',1);  %sheet no.1 has dec_limt >10 m/s2 and distance to ped <3m










for ii=1:403   
    index = EventIndices(indexWaiting(ii),8) - EventIndices(indexWaiting(ii),4)+5;
    StartingVelocity(ii,1) = DataPredict{indexWaiting(ii)}.PedestrianAbsoluteVelocityAverage(index,1);
    StartingVelocity(ii,2) = DataPredict{indexWaiting(ii)}.PedestrianAbsoluteVelocityAverage(index+5,1); 
end
mean(StartingVelocity)

save('StartingVelocityDistribution.mat','StartingSpeed_05_Normal','StartSpeed_10_Normal')


