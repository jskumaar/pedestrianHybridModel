%% This function creates and initializes a new prediction tracklet
% Inputs: existing set of tracklets, current node no and (independent) probability of
% this tracklet
% Output: set of tracklets with additionally a new tracklet

function predTracklet = newTracklet(predTracklet, node_no, prob_tracklet)
    % current number of tracklets
    N_tracklets = size(predTracklet,1);
    % create a new tracklet and initialize it
    predTracklet.data{N_tracklets+1} = predTracklet.data{N_tracklets}(end, :);
    predTracklet.probability(N_tracklets+1) = prob_tracklet; 
    predTracklet.startNode(N_tracklets+1) = node_no; 
    predTracklet.isActive(N_tracklets+1) = true; 
    predTracklet.eventFlag(N_tracklets+1) = false;
    predTracklet.endNode(N_tracklets+1) = -1; 
    predTracklet.Goal = 'NA';
end