%% This file recalculate the test train indices after removing the high deceleraion data

clear all

%add path to the MATLAB variables and excel sheets
addpath('G:\my drive\Research\Pedestrian Modelling Project\Modelling Scripts and Results\2. Mat Data\')
addpath('G:\My Drive\Research\Pedestrian Modelling Project\Modelling Scripts and Results\1. Study I Data for Modeling\Compiled Data\');


%load data
load('HybridModelTestTrainIndices_wHighDec_old.mat')
load('ExpectedGapData_W5_NewDTCurbDTCW_StartingTimeforCrossingGaps.mat')
[high_dec_data,~] = xlsread('High_acceleration_data.xlsx',1);
[EventIndices,~] = xlsread('DiscreteStateEventIndicesW5.xlsx');
[GapData,~] = xlsread('VehicleGapTimesV6.xlsx');   %changed the -0.1 gap duratiosn to 0.1
S = vartype('numeric');


% identify the crossing indices that have high deceleration
high_dec_index_to_remove = [];
for ii=1:size(high_dec_data,1)
    high_dec_index_to_remove = [high_dec_index_to_remove, 18*(high_dec_data(ii,1)-1) + 6*(high_dec_data(ii,2)-1) + [high_dec_data(ii,3):6]]; 
end
high_dec_index_to_remove = high_dec_index_to_remove';


%% 1) Identify test and train crossing indices
[~,ind_to_remove,] = intersect(TrainIndices_woExtremeOutlier, high_dec_index_to_remove);
TrainIndices_woExtremeOutlier_woHighDec = TrainIndices_woExtremeOutlier;
TrainIndices_woExtremeOutlier_woHighDec(ind_to_remove) = [];

[~,ind_to_remove,] = intersect(TrainIndices_woMildOutlier, high_dec_index_to_remove);
TrainIndices_woMildOutlier_woHighDec = TrainIndices_woMildOutlier;
TrainIndices_woMildOutlier_woHighDec(ind_to_remove) = [];

[~,ind_to_remove,] = intersect(TestIndices_woExtremeOutlier, high_dec_index_to_remove);
TestIndices_woExtremeOutlier_woHighDec = TestIndices_woExtremeOutlier;
TestIndices_woExtremeOutlier_woHighDec(ind_to_remove) = [];
 
[~,ind_to_remove,] = intersect(TestIndices_woMildOutlier, high_dec_index_to_remove);
TestIndices_woMildOutlier_woHighDec = TestIndices_woMildOutlier;
TestIndices_woMildOutlier_woHighDec(ind_to_remove) = [];



%find gap indices that have the crossing number of high_dec
EG_CrossingNumber = 18*(ExpectedGapData.VehicleGapTimes(:,1)-1) + 6*(ExpectedGapData.VehicleGapTimes(:,2)-1) + ExpectedGapData.VehicleGapTimes(:,3);


EG_high_dec_index = [];
for ii=1:length(high_dec_index_to_remove)
    EG_high_dec_index = [EG_high_dec_index; find(EG_CrossingNumber == high_dec_index_to_remove(ii))];
end

TrainGapIndices_extreme = [];
for ii=1:length(TrainIndices_woExtremeOutlier)
    TrainGapIndices_extreme = [TrainGapIndices_extreme; find(EG_CrossingNumber == TrainIndices_woExtremeOutlier(ii))];
end

TrainGapIndices_mild = [];
for ii=1:length(TrainIndices_woMildOutlier)
    TrainGapIndices_mild = [TrainGapIndices_mild; find(EG_CrossingNumber == TrainIndices_woMildOutlier(ii))];
end

[~,to_rem_ind,~] = intersect(TrainGapIndices_extreme,EG_high_dec_index);
TrainGapIndices_extreme_noHigh_dec = TrainGapIndices_extreme;
TrainGapIndices_extreme_noHigh_dec(to_rem_ind) = [];

[~,to_rem_ind,~] = intersect(TrainGapIndices_mild,EG_high_dec_index);
TrainGapIndices_mild_noHigh_dec = TrainGapIndices_mild;
TrainGapIndices_mild_noHigh_dec(to_rem_ind) = [];


TestGapIndices_extreme = [];
for ii=1:length(TestIndices_woExtremeOutlier)
    TestGapIndices_extreme = [TestGapIndices_extreme; find(EG_CrossingNumber == TestIndices_woExtremeOutlier(ii))];
end
 
TestGapIndices_mild = [];
for ii=1:length(TestIndices_woMildOutlier)
    TestGapIndices_mild = [TestGapIndices_mild; find(EG_CrossingNumber == TestIndices_woMildOutlier(ii))];
end
 
[~,to_rem_ind,~] = intersect(TestGapIndices_extreme,EG_high_dec_index);
TestGapIndices_extreme_noHigh_dec = TestGapIndices_extreme;
TestGapIndices_extreme_noHigh_dec(to_rem_ind) = [];
 
[~,to_rem_ind,~] = intersect(TestGapIndices_mild,EG_high_dec_index);
TestGapIndices_mild_noHigh_dec = TestGapIndices_mild;
TestGapIndices_mild_noHigh_dec(to_rem_ind) = [];

clearvars ExpectedGapData
save('HybridModelTestTrainIndices.mat')


