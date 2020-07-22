%% Created: 10/17/2019

% This function finds the crossing indices for which the there was a very high
% deceleration

function high_dec_data = HighDeceleration_data_separate(dec_limit,stop_dist_limit)

%addpath (also check if the data is read from the correct excel files)
addpath('G:\My Drive\Research\Pedestrian Modelling Project\Modelling Scripts and Results\1. Study I Data for Modeling\Compiled Data')


high_deceleration_data = [];

for kk=1:30
    for jj=1:3
        
        %Read events data
        [EventsData,~] = xlsread('DiscreteStateEventIndicesW5.xlsx');
        
        %Read nearest vehicle DTC, position, next vehicle position,
        %adjacent vehicle position data
        [vehicleData,~] = xlsread('VehicleDTCSpeedData_allFourVehicles.xlsx',6*(kk-1)+jj);
        sameLane_acc = vehicleData(50:end-20,8);
        adjLane_acc = vehicleData(50:end-20,14);
        samelane_dist = vehicleData(50:end-20,6);
        adjLane_dist = vehicleData(50:end-20,12);
                
        dec_ind = find((abs(sameLane_acc)>dec_limit & samelane_dist<stop_dist_limit) | (abs(adjLane_acc)>dec_limit & adjLane_dist<stop_dist_limit),1,'first');
        
        if ~isempty(dec_ind)
            high_deceleration_data = [high_deceleration_data;kk,jj,dec_ind+50 ];
        end
        
    end
end


%data including the crossing number
high_dec_data = zeros(length(high_deceleration_data),4);
high_dec_data(:,1:3) =  high_deceleration_data;
crossing_number = [];
% include crossing number to the high deceleration interactions
for ii = 1: length(high_deceleration_data)
    sub = high_deceleration_data(ii,1);
    scene = high_deceleration_data(ii,2);
    index = high_deceleration_data(ii,3);
    
     for jj = 1:6
        cross_index = 18*(sub-1) + 3*(scene-1) + jj; 
        if  (index >= EventsData(cross_index,4) && index <= EventsData(cross_index,11))
            high_dec_data(ii,4) = jj;  
        end
     end
end

high_dec_data(high_dec_data(:,4)==0,:) = [];

save('high_dec_data.mat','high_dec_data')


end

