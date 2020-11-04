indices_interest = [];
for sceneId = 1:12
    N_tracks = size(formattedTracksData{sceneId},1);
    for track_id = 1:N_tracks
        if strcmp(formattedTracksData{sceneId}{track_id}.class(1), 'pedestrian')
            trackData = formattedTracksData{sceneId}{track_id};
            N_ts = length(trackData.frame);
            crossStart = [];
            crossedCW = [];
            for ii=2:N_ts
                if strcmp(trackData.HybridState(ii),'Crossing')&& ~strcmp(trackData.HybridState(ii-1),'Crossing')
                    crossStart = [crossStart; ii];
                    indices_interest = [indices_interest; [sceneId, track_id, ii, 1] ];
                end
                
                if (trackData.longDispPedCw(ii)<0 && trackData.longDispPedCw(ii-1)>0 ) ||...
                   (trackData.longDispPedCw(ii)>0 && trackData.longDispPedCw(ii-1)<0 )     
                    crossedCW  = [crossedCW; ii];
                    indices_interest = [indices_interest; [sceneId, track_id, ii, 2] ];
                end
            end
            crossStartTimeSteps{sceneId}{track_id} = crossStart;
            crossedCWTimeSteps{sceneId}{track_id} = crossedCW;
        else
            crossStartTimeSteps{sceneId}{track_id} = [];
            crossedCWTimeSteps{sceneId}{track_id} = [];
        end
    end
end

%% filter the indices of interest
% if there is a crossing event closer in time to a crossed CW event, use
% only the cross crosswalk event

indices_interest_filtered = indices_interest;
rem_ind = [];
for ii=1:length(indices_interest)
    if indices_interest_filtered(ii, 4)==2
       cross_ind = find(indices_interest_filtered(:,4)==1 & indices_interest_filtered(:,1)==indices_interest_filtered(ii,1) & ...
                 indices_interest_filtered(:,2)==indices_interest_filtered(ii,2) );
       
       ts = indices_interest_filtered(cross_ind,3);
       for jj=1:length(ts)
            if abs(ts(jj) - indices_interest_filtered(ii,3)) < 20
                rem_ind = [rem_ind; ii];
            end
       end
        
    end
end

indices_interest_filtered(rem_ind,:) = [];


