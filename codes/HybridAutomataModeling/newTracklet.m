%% This function creates and initializes a new prediction tracklet
% Inputs: existing set of tracklets, current node no and (independent) probability of
% this tracklet
% Output: set of tracklets with additionally a new tracklet

function [predTracklet, newTrackletId, flag] = newTracklet(flag, predTracklet, node_no, prob_tracklet, newTrackletId)
    % current number of tracklets
    % N_tracklets = size(predTracklet.data,1);  
    newTrackletId = newTrackletId + 1;
    
    % create a new tracklet and initialize it
    predTracklet.data{newTrackletId,1} = predTracklet.data{newTrackletId-1}(end, :);
    predTracklet.probability(newTrackletId,1) = prob_tracklet; 
    predTracklet.startNode(newTrackletId,1) = node_no; 
    predTracklet.isActive(newTrackletId,1) = true; 
    predTracklet.eventFlag(newTrackletId,1) = false;
    predTracklet.endNode(newTrackletId,1) = -1; 
    predTracklet.Goal(newTrackletId,:) = 'NA';   
    predTracklet.kfData{newTrackletId,1} = predTracklet.kfData{newTrackletId-1}(end, :);
        
    % initialize all flags for the new tracklet
    flag.dataCompile(newTrackletId) = false;
    flag.pred(newTrackletId) = false;
    flag.EgoCar(newTrackletId) = false;
    flag.GapStart(newTrackletId) = false;
    flag.sampleWaitTime(newTrackletId) = false;
    flag.startCross(newTrackletId) = false;
    flag.finishedCrossing(newTrackletId) = false;
    flag.reachCrosswalk(newTrackletId) = false;    
    flag.checkIntent(newTrackletId) = false;
    flag.atCrosswalk(newTrackletId) = false;
end