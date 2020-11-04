%% constant velocity predictions

function [pedPredictions, kfPred] = func_HPed_CV(kf, currentPedData, Params, ped_track_time_step)

del_t = Params.delta_T;
pedPos = [currentPedData.xCenter(ped_track_time_step), currentPedData.yCenter(ped_track_time_step)];
pedVel = [currentPedData.xVelocity(ped_track_time_step), currentPedData.yVelocity(ped_track_time_step)];

pedPredictions{1}(1,1) = -1; % if end Node is -1, it represents that predictions were made using the constant velocity model
pedPredictions{1}(1,2) = 1;  % probability of predicted path

% 1st prediction
pedPredictions{1} = [pedPredictions{1}, pedPos + del_t*pedVel];
kf = kalmanPredict(kf);
kfPred = [kf.x', diag(kf.P)'];

for pred_time_step = 2:Params.predHorizon
    pedPredictions{1}(end+1:end+2) = pedPredictions{1}(end-1:end) + del_t*pedVel;
    kf = kalmanPredict(kf);
    kfPred(end+1, :) = [kf.x', diag(kf.P)'];
end
