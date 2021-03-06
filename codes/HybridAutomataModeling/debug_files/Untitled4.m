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
% for sceneId = 1: N_scenes
%    N_tracks = size(tracksData{sceneId}, 1);
%    
%    
%    for track_id = 1:N_tracks
%        
%        % initialize heading
%        heading = 0;
%        N_instances = size(tracksData{sceneId}{track_id}, 1);
%        
%        for track_time_step = 1:N_instances
%           % calculate heading
%             y_vel = tracksData{sceneId}{track_id}.yVelocity(track_time_step);
%             if ( abs(y_vel) < moving_threshold )
%                 y_vel = 0;
%             end
%             x_vel = tracksData{sceneId}{track_id}.xVelocity(track_time_step);
%             if ( abs(x_vel) < moving_threshold )
%                 x_vel = 0;
%             end
%             % if the agent is stopped, maintain the previous heading
%             if x_vel~=0 || y_vel~=0 
%                 heading = atan2(y_vel, x_vel)*180/pi;
%             end
%             
%             % copy the heading
%             tracksData{sceneId}{track_id}.calcHeading(track_time_step) = heading;
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
% for sceneId = 1:N_scenes
%     ped_notCrossing_tracks = tracks{sceneId}.ped_tracks;
%     
%     % remove crossing tracks from this
%     ped_crossing_tracks = tracks{sceneId}.ped_crossing_tracks;
%     [~,ind,~] = intersect(ped_notCrossing_tracks, ped_crossing_tracks);
%     ped_notCrossing_tracks(ind) = [];
%     
%     % remove jaywalking tracks
%     ped_jaywalking_tracks = tracks{sceneId}.ped_jaywalking_tracks;
%     [~,ind,~] = intersect(ped_notCrossing_tracks, ped_jaywalking_tracks);
%     ped_notCrossing_tracks(ind) = [];
%    
%     tracks{sceneId}.ped_notCrossing_tracks = ped_notCrossing_tracks;
%     
% end
% 
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % write compiled track data to hdf5 file
% 
% for sceneId=1:12
% 
% filename = strcat('compiledData_inD_','scene_',num2str(sceneId),'.dat');
% 
% writecell(formattedTracksData{sceneId},filename)
% 
% end
% 
% 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% change data format from cell to double
% ped_close = [];
% for sceneId=1:12
%     
%     ped_tracks = [tracks_updated{sceneId}.ped_crossing_tracks; tracks_updated{sceneId}.ped_not_crossing_tracks];
%     
%     N_tracks = length(ped_tracks);
%     
%     for ii = 1:N_tracks
%         
%         isPedClose = false;
%         sceneId = ped_tracks(ii);
%         N_instances = size(formattedTracksData{sceneId}{sceneId}, 1);
%         
%         for jj=1:N_instances
%             if ( abs( formattedTracksData{sceneId}{sceneId}.long_disp_ped_cw_pixels(jj)) < 0.2 && strcmp(formattedTracksData{sceneId}{sceneId}.HybridState(jj), 'Approach') )
%                 formattedTracksData{sceneId}{sceneId}.pedClose(jj) = true;
%                 isPedClose = true;
%             else
%                 formattedTracksData{sceneId}{sceneId}.pedClose(jj) = false;
%             end
%         end
%         
%         if isPedClose
%             ped_close = [ped_close; [sceneId, sceneId] ];
%         end
%     end
% 
% 
% end
% 
% 
% 
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% read tracks Meta Data

% for jj=1:12
%     sceneId = 17+jj;
%     tracksMetaData{jj} = readtable(strcat(num2str(sceneId),'_tracksMeta.csv')) ;
% 
% end
% % 
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % create a wait time 
% 
% for sceneId = 1:12
%     N_tracks = size(formattedTracksData{sceneId},1);
%     for track_id = 1:N_tracks
%         act_wt = [];
%         N_instances = size(formattedTracksData{sceneId}{track_id},1);
%         if strcmp(formattedTracksData{sceneId}{track_id}.class(1), 'pedestrian')
%            wait_times  = find(strcmp(formattedTracksData{sceneId}{track_id}.HybridState, 'Wait'));
%            del_wt = diff(wait_times);
%            if ~isempty(del_wt)
%                act_wt = wait_times(1);
%                for ii = 1:size(del_wt)-1
%                    if (del_wt(ii+1) - del_wt(ii)) > 2                   
%                        act_wt = [act_wt, wait_times(ii+2)];
%                    end
%                end
%            end
%            
%         
%         wait_time_steps = [];
%         ind = 1;
%         loop = 1;
%         if ~isempty(act_wt)
%             while(loop)
%                 if ind<=length(act_wt)
%                     if length(wait_time_steps)+1==act_wt(ind)
%                         wait_time_steps = [wait_time_steps; act_wt(ind)];
%                     else
%                         if ~isempty(wait_time_steps)
%                             wait_time_steps = [wait_time_steps; wait_time_steps(end)];
%                         else
%                             wait_time_steps = 0;
%                         end
%                     end
%                 else
%                     wait_time_steps = [wait_time_steps; wait_time_steps(end)];
%                 end
%                 
%                 ind = ind+1;
% 
%                 if length(wait_time_steps)==N_instances
%                     loop = 0;
%                 end
%             end
%         end
%         
%         if ~isempty(wait_time_steps)
%             formattedTracksData{sceneId}{track_id}.wait_time_steps = wait_time_steps;
%         else
%             formattedTracksData{sceneId}{track_id}.wait_time_steps = -1*ones(N_instances,1);
%         end
% 
%         end
%     end
% end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Aliter%%%%%%%%%%%%%%%%%
% skip_ts = 5;
% for sceneId = 1:12
%     N_tracks = size(formattedTracksData{sceneId},1);
%     
%     
%     
%     for track_id = 1:N_tracks
% 
%         N_instances = size(formattedTracksData{sceneId}{track_id},1);
%         if strcmp(formattedTracksData{sceneId}{track_id}.class(1), 'pedestrian')
%            wait_time_steps  = cumsum(formattedTracksData{sceneId}{track_id}.ProbHybridState(:,2));
% %             if (sum(wait_time_steps~=0)~=0)
% %                 formattedTracksData{sceneId}{track_id}.wait_time_steps = wait_time_steps*skip_ts;
% %             else
% %                 formattedTracksData{sceneId}{track_id}.wait_time_steps = -1*ones(N_instances,1);
% %             end
%             formattedTracksData{sceneId}{track_id}.wait_time_steps = wait_time_steps*skip_ts;
%         end
% 
%     end
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% for sceneId = 1:12
%    tracks{sceneId}.pedCrossingTracks = tracks{sceneId}.ped_crossing_tracks;
%    tracks{sceneId}.pedNotCrossingTracks = tracks{sceneId}.ped_not_crossing_tracks;  
%    tracks{sceneId}.pedJaywalkingTracks = tracks{sceneId}.ped_jaywalking_tracks;
%    tracks{sceneId}.pedWaitingTracks = tracks{sceneId}.ped_waiting_tracks;
%    tracks{sceneId}.pedTracks = tracks{sceneId}.ped_tracks;
%    tracks{sceneId}.carTracks = tracks{sceneId}.car_tracks;
%    tracks{sceneId}.carParkedTracks = tracks{sceneId}.car_parked_tracks;
%    tracks{sceneId}.carMovingTracks = tracks{sceneId}.car_moving_tracks;
%    
%    
%    tracks{sceneId}.ped_crossing_tracks = [];
%    tracks{sceneId}.ped_not_crossing_tracks = [];  
%    tracks{sceneId}.ped_jaywalking_tracks = [];
%    tracks{sceneId}.ped_waiting_tracks = [];
%    tracks{sceneId}.ped_tracks = [];
%    tracks{sceneId}.car_tracks = [];
%    tracks{sceneId}.car_parked_tracks = [];
%    tracks{sceneId}.car_moving_tracks = [];
%    
%    tracksUpdated{sceneId}.pedCrossingTracks = tracks_updated{sceneId}.ped_crossing_tracks;
%    tracksUpdated{sceneId}.pedNotCrossingTracks = tracks_updated{sceneId}.ped_not_crossing_tracks;  
%    tracksUpdated{sceneId}.pedJaywalkingTracks = tracks_updated{sceneId}.ped_jaywalking_tracks;
%    tracksUpdated{sceneId}.pedWaitingTracks = tracks_updated{sceneId}.ped_waiting_tracks;
%    tracksUpdated{sceneId}.pedTracks = tracks_updated{sceneId}.ped_tracks;
%    tracksUpdated{sceneId}.carTracks = tracks_updated{sceneId}.car_tracks;
%    tracksUpdated{sceneId}.carParkedTracks = tracks_updated{sceneId}.car_parked_tracks;
%    tracksUpdated{sceneId}.carMovingTracks = tracks_updated{sceneId}.car_moving_tracks;
%    
%    tracks_updated{sceneId}.ped_crossing_tracks = [];
%    tracks_updated{sceneId}.ped_not_crossing_tracks = [];  
%    tracks_updated{sceneId}.ped_jaywalking_tracks = [];
%    tracks_updated{sceneId}.ped_waiting_tracks = [];
%    tracks_updated{sceneId}.ped_tracks = [];
%    tracks_updated{sceneId}.car_tracks = [];
%    tracks_updated{sceneId}.car_parked_tracks = [];
%    tracks_updated{sceneId}.car_moving_tracks = [];
% end
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% for sceneId = 1:12
%     N_tracks = size(formattedTracksData{sceneId},1);
%     for track_id = 1:N_tracks
%         formattedTracksDataStruct{sceneId,1}{track_id,1} = table2struct(formattedTracksData{sceneId}{track_id}, 'ToScalar',true);
%         formattedTracksDataStruct{sceneId,1}{track_id,1}.class  = string( formattedTracksDataStruct{sceneId,1}{track_id,1}.class);
%     end
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% load('tracksData_reSampled_correctDisCW_v8.mat')

% for sceneId = 1:12
%     N_tracks = size(formattedTracksData{sceneId},1);
%     for track_id = 1:N_tracks
%         formattedTracksData{sceneId,1}{track_id,1}.trackId = formattedTracksData{sceneId,1}{track_id,1}.trackId+1;
%     end
% end

% 
% % load('tracksData_v10.mat')
% tic
% zenoCW.sceneId =[];
% zenoCW.trackId = [];
% zenoCW.N_instances = [];
% 
% zenoHybrid.sceneId =[];
% zenoHybrid.trackId = [];
% zenoHybrid.N_instances = [];
% 
% flag.hybridStatePred = false;
% for sceneId = 1:12
%     N_tracks = size(formattedTracksData{sceneId},1);
%     for track_id = 1:N_tracks
%            
%            if strcmp(formattedTracksData{sceneId}{track_id,1}.class{1}, 'pedestrian') 
%                if track_id==5
%                    x=1;
%                end
%                 [formattedTracksData{sceneId}{track_id,1}] = ....
%                         hybridStateCopy_v3(formattedTracksData{sceneId}{track_id,1}, cw, flag, annotatedImageEnhanced, Params, 1, resetStates);    
%                                    
% %                   [formattedTracksData{sceneId}{track_id,1}] = ....
% %                         hybridStateCopy(formattedTracksData{sceneId}{track_id,1}, cw, flag, annotatedImageEnhanced, Params);    
% %                          
%                % check for anomalies in close CW calculation
%                pedData = formattedTracksData{sceneId}{track_id,1};
%                diff_closeCw = diff(pedData.closestCW);
%                ind = find(diff_closeCw~=0);
%                diff_ind = diff(ind);
%                diff_ind_2 = diff_ind;
%                diff_ind_2(diff_ind_2 > 5) = [];
%                if ~isempty(diff_ind_2)
%                    zenoCW.sceneId = [zenoCW.sceneId;sceneId];
%                    zenoCW.trackId = [zenoCW.trackId;track_id];
%                    zenoCW.N_instances = [zenoCW.N_instances; length(diff_ind_2)];
%                end
%                
%                % check for anomalies in hybrid state calculation
%                pedData = formattedTracksData{sceneId}{track_id,1};
%                
%                diff_hybrid = [];
%                for ii = 2:length(pedData.frame)
%                    if (~strcmp(pedData.HybridState(ii), pedData.HybridState(ii-1)) && ii>5 && ~(strcmp(pedData.HybridState(ii-1),'Wait')) )
%                        diff_hybrid = [diff_hybrid; ii];
%                    end
%                end
%                diff_ind_hybrid = diff(diff_hybrid);
%                diff_ind_hybrid_2 = diff_ind_hybrid;
%                diff_ind_hybrid_2(diff_ind_hybrid_2 > 5) = [];
%                if ~isempty(diff_ind_hybrid_2)
%                    zenoHybrid.sceneId = [zenoHybrid.sceneId;sceneId];
%                    zenoHybrid.trackId = [zenoHybrid.trackId;track_id];
%                    zenoHybrid.N_instances = [zenoHybrid.N_instances; length(diff_ind_hybrid_2)];
%                end
%                x=1;
%                     
%            end
%     end
% end
% toc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% for sceneId = 1:12
%     N_tracks = size(formattedTracksData{sceneId},1);
%     for trackId = 1:N_tracks
%         trackData = formattedTracksData{sceneId}{trackId};
%         fn = fieldnames(trackData);
%         x=1;
%         clear variable_new
%         for fieldId = 1:length(fn)
%             fni = string(fn(fieldId));
%             if ( strcmp(fni, 'latDispPedCw') || strcmp(fni, 'longDispPedCw') || strcmp(fni, 'long_disp_ped_car') || strcmp(fni, 'isPedSameDirection') ||....
%                  strcmp(fni, 'isLooking') || strcmp(fni, 'closeCar_ind')  ) 
%                 variable = trackData.(fni);
%                 if size(variable,1)==1
%                     variable_new(:,1) = variable(1,1:end);
%                 else
%                     variable_new(:,1) = variable(1:end,1);
%                 end
%                 trackData.(fni) = variable_new; 
%             end
%             x=1;
%         end
%     
%         formattedTracksData{sceneId}{trackId} = trackData;
%     end
%     
%     
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% plot
car_t0 = 94;
car_tf = 104;
ped_t0 = 1;
ped_tf = 162;
sf = 0.0978;

annotatedImageEnhanced_2 = annotatedImageEnhanced;
pedData = formattedTracksData{1}{299};

for car_ts = car_t0:car_tf
   carPosPixels = int32([carData.xCenter(car_ts), carData.yCenter(car_ts)]/sf);   
   annotatedImageEnhanced_2(-carPosPixels(2), carPosPixels(1)) = 0;
end


for ped_ts = ped_t0:ped_tf
   pedPosPixels = int32([pedData.xCenter(ped_ts), pedData.yCenter(ped_ts)]/sf);   
   annotatedImageEnhanced_2(-pedPosPixels(2), pedPosPixels(1)) = 255;
end

figure()
imshow(annotatedImageEnhanced_2)



