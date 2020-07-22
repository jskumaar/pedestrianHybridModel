%% compile Gap features

for ped_ind = 1:size(ped_tracks,1)
    pedData = formattedTracksData{ped_tracks(ped_ind)};
    gap_ind = find(pedData.Gap_start == true);
    Features
    
    
end

% run the SVM model
load('SVM_ExtremeOutlier_NoHighDeceleration_7Features_noTimeGap.mat')










