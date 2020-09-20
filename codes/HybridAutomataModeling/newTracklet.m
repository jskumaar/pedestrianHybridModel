%% This function creates and initializes a new prediction tracklet
% Inputs: existing set of tracklets, current node no and (independent) probability of
% this tracklet
% Output: set of tracklets with additionally a new tracklet

function [predTracklet, flag] = newTracklet(flag, predTracklet, node_no, prob_tracklet)
    % current number of tracklets
    N_tracklets = size(predTracklet.data,1);
    % create a new tracklet and initialize it
    predTracklet.data{N_tracklets+1,1} = predTracklet.data{N_tracklets}(end, :);
    predTracklet.probability(N_tracklets+1,1) = prob_tracklet; 
    predTracklet.startNode(N_tracklets+1,1) = node_no; 
    predTracklet.isActive(N_tracklets+1,1) = true; 
    predTracklet.eventFlag(N_tracklets+1,1) = false;
    predTracklet.endNode(N_tracklets+1,1) = -1; 
    predTracklet.Goal(N_tracklets+1,:) = 'NA';
    
    predTracklet.kfData{N_tracklets+1,1} = predTracklet.kfData{N_tracklets}(end, :);
    
    if N_tracklets>=3
        x=1;
    end
    
    % initialize all flags for the new tracklet
    flag.dataCompile(N_tracklets+1) = false;
    flag.pred(N_tracklets+1) = false;
    flag.EgoCar(N_tracklets+1) = false;
    flag.GapStart(N_tracklets+1) = false;
    flag.sampleWaitTime(N_tracklets+1) = false;
    flag.startCross(N_tracklets+1) = false;
    flag.finishedCrossing(N_tracklets+1) = false;
    flag.reachCrosswalk(N_tracklets+1) = false;    
    flag.checkIntent(N_tracklets+1) = false;
    flag.atCrosswalk(N_tracklets+1) = false;
    
end