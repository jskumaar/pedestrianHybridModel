% temp

p1 = genpath('G:\My Drive\Research\Projects\pedestrianHybridModel\codes');
p2 = genpath('G:\My Drive\Research\Projects\pedestrianHybridModel\datasets');

addpath(p1)
addpath(p2)

load('GapData_12Scenes_v7.mat');

% gaps with zero wait
indices_noWait = find(GapFeatures.F_cumWait==-1);


for ii = 1:length(indices_noWait)
    gapInd = indices_noWait(ii);
    sceneId = GapFeatures_SVM.recording(gapInd)-17;
    pedTrackId = GapFeatures_SVM.pedTrack(gapInd);
    carTrackId = GapFeatures_SVM.egoCarTrack(gapInd);
    frame = GapFeatures_SVM.frame(gapInd);
    
    % check if the pedestrian did not wait at all, especially for the particular
    % ego car
    pedData = formattedTracksData{sceneId}{pedTrackId};
    ind_wait_ego = find(pedData.waitTimeSteps==-1 & pedData.closeCar_ind==carTrackId);
    if ~isempty(ind_wait_ego)
        x=1;
    end
    

    
end
