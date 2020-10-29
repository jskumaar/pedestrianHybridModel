%% Computation performance

% find the number of pedestrians and the no of time instances

total_comp_time = 11010.151;

N_scenes = size(predictedPedTraj_MHP,1);
total_N_timeSteps = 0;
total_N_car = 0;
total_N_ped = 0;

for sceneId  = 1:N_scenes
    scenePred = predictedPedTraj_MHP{N_scenes};
    N_cars = size(scenePred,1);
    for carId = 1:N_cars
       carPred = scenePred{carId};
       if ~isempty(carPred)
          total_N_car = total_N_car + 1;
          N_ped = size(carPred,1) ;
          for pedId = 1:N_ped
             pedPred =  carPred{pedId};
             if ~isempty(pedPred)
                 N_Ts = size(pedPred.data,1);
                 total_N_timeSteps = total_N_timeSteps + N_Ts;
                 total_N_ped = total_N_ped + 1;
             end
          end
       end
    end
    
end


% time per pedestrian per time step (entire prediction horizon, N = 6 s, 30
% steps)
comp_time = total_comp_time/total_N_timeSteps


