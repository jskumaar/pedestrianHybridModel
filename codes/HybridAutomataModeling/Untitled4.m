%         %has the pedestrian started crossing?
%         if ( strcmp(currentPedData.HybridState{time_step},'Crossing') && ~strcmp(currentPedData.HybridState{time_step-1},'Crossing') )
% 
%             % if the pedestrian crossed for the ego-vehicle
%             % (closeCar_ind), whose gap started sometime
%             % back
%             GapFeatures_ind_temp1 = find(GapFeatures.recording == currentPedData.recordingId(1)); 
%             GapFeatures_ind_temp2 = find(GapFeatures.pedTrack  == ped_track); 
%             GapFeatures_ind_temp3 = find(GapFeatures.egoCarTrack == closeCar_ind); 
% 
%             GapFeatures_ind = intersect(intersect(GapFeatures_ind_temp1,GapFeatures_ind_temp2), GapFeatures_ind_temp3);
% 
%             if ~isempty(GapFeatures_ind)
%                 GapFeatures.CrossDecision(GapFeatures_ind,1) = true;
%                 GapFeatures.CrossStart(GapFeatures_ind,1) = pedFrame;
%                 GapFeatures.CrossCW(GapFeatures_ind,1) = cw_ped;
%             end
% 
%         end
%         
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
%         
% start_ind = find(diff(GapFeatures.recording)~=0);
% start_ind = [1; start_ind + 1];
% end_ind = [start_ind(2:end) - 1; size(GapFeatures,1)];
% 
% 
% for ii=1:12
%     temp = GapFeatures(start_ind(ii):end_ind(ii),:);
%     pedTrack = temp.pedTrack;
%     a = unique(pedTrack);
%     Ncount = histc(pedTrack, a);
%     max(Ncount)
% end
% 
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% 
% % load the Gap features
% % 
% 
% AcceptedGaps = GapFeatures(GapFeatures.CrossDecision==1, :);
% CrossDelay = (AcceptedGaps.CrossStart - AcceptedGaps.frame)/25;
% figure()
% h = histogram(CrossDelay,'BinWidth',2,'Normalization','probability')
% 
% % curve fit
% y = h.Values;
% x = (h.BinEdges(1:end-1) + h.BinEdges(2:end))/2;
% figure()
% plot(x,y)
% % fit curve using the curve fit toolbox GUI
% 
% x = x';
% y = y';
% 
% g = fittype('a-b*exp(-c*x)');
% f0 = fit(x,y,g,'StartPoint',[[ones(size(x)), -exp(-x)]\y; 1]);
% 
% figure()
% a = [0:0.1:50];
% plot(a, f0(a)); hold on;
% plot(x, y, '*')
% 
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % check the distribution of wait when an approach gap is accepted
% 
% 
% 
% 
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % calculate the heading of the tracks
% N_scenes = size(tracksData,2);
% moving_threshold = 0.1;
% 
% for scene_id = 1: N_scenes
%    N_tracks = size(tracksData{scene_id}, 1);
%    
%    
%    for track_id = 1:N_tracks
%        
%        % initialize heading
%        heading = 0;
%        N_instances = size(tracksData{scene_id}{track_id}, 1);
%        
%        for track_time_step = 1:N_instances
%           % calculate heading
%             y_vel = tracksData{scene_id}{track_id}.yVelocity(track_time_step);
%             if ( abs(y_vel) < moving_threshold )
%                 y_vel = 0;
%             end
%             x_vel = tracksData{scene_id}{track_id}.xVelocity(track_time_step);
%             if ( abs(x_vel) < moving_threshold )
%                 x_vel = 0;
%             end
%             % if the agent is stopped, maintain the previous heading
%             if x_vel~=0 || y_vel~=0 
%                 heading = atan2(y_vel, x_vel)*180/pi;
%             end
%             
%             % copy the heading
%             tracksData{scene_id}{track_id}.calcHeading(track_time_step) = heading;
%            
%        end
% 
%    end
% 
%     
% end
% 
% 
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% % plot the pedestrian vision range and the vehicle box
% 
% figure()
% plot(carEdgePoints(:,1), carEdgePoints(:,2), '*', 'MarkerSize', 12); hold on;
% plot(pedVisionPoints(:,1), pedVisionPoints(:,2), 'r*','MarkerSize', 12); hold on;
% 
% 
% 
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% % compile the pedestrian tracks, crossing and not crossing
% N_scenes = size(tracks,2);
% 
% for scene_id = 1:N_scenes
%     ped_notCrossing_tracks = tracks{scene_id}.ped_tracks;
%     
%     % remove crossing tracks from this
%     ped_crossing_tracks = tracks{scene_id}.ped_crossing_tracks;
%     [~,ind,~] = intersect(ped_notCrossing_tracks, ped_crossing_tracks);
%     ped_notCrossing_tracks(ind) = [];
%     
%     % remove jaywalking tracks
%     ped_jaywalking_tracks = tracks{scene_id}.ped_jaywalking_tracks;
%     [~,ind,~] = intersect(ped_notCrossing_tracks, ped_jaywalking_tracks);
%     ped_notCrossing_tracks(ind) = [];
%    
%     tracks{scene_id}.ped_notCrossing_tracks = ped_notCrossing_tracks;
%     
% end
% 
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % write compiled track data to hdf5 file
% 
% for scene_id=1:12
% 
% filename = strcat('compiledData_inD_','scene_',num2str(scene_id),'.dat');
% 
% writecell(formattedTracksData{scene_id},filename)
% 
% end
% 
% 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% change data format from cell to double
ped_close = [];
for scene_id=1:12
    
    ped_tracks = [tracks_updated{scene_id}.ped_crossing_tracks; tracks_updated{scene_id}.ped_not_crossing_tracks];
    
    N_tracks = length(ped_tracks);
    
    for ii = 1:N_tracks
        
        isPedClose = false;
        track_id = ped_tracks(ii);
        N_instances = size(formattedTracksData{scene_id}{track_id}, 1);
        
        for jj=1:N_instances
            if ( abs( formattedTracksData{scene_id}{track_id}.long_disp_ped_cw_pixels(jj)) < 0.2 && strcmp(formattedTracksData{scene_id}{track_id}.HybridState(jj), 'Approach') )
                formattedTracksData{scene_id}{track_id}.pedClose(jj) = true;
                isPedClose = true;
            else
                formattedTracksData{scene_id}{track_id}.pedClose(jj) = false;
            end
        end
        
        if isPedClose
            ped_close = [ped_close; [scene_id, track_id] ];
        end
    end


end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% read tracks Meta Data

for jj=1:12
    scene_id = 17+jj;
   tracksMetaData{jj} = readtable(strcat(num2str(scene_id),'_tracksMeta.csv')) ;

end
