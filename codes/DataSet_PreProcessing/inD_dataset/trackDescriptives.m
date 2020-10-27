%% This script finds the descriptives of the tracks

%% a) Compile the descriptives

% N_scenes=12;
%     for image_id = 1:N_scenes
% 
%         N_tracks = size(formattedTracksData{image_id},1);
% 
%         % initialize the variables
%         trackDescriptivesData.approachSpeed = zeros(N_tracks,1);
%         trackDescriptivesData.crossSpeed = zeros(N_tracks,1);
%         trackDescriptivesData.walkawaySpeed = zeros(N_tracks,1);
%         trackDescriptivesData.overallWaitDuration = zeros(N_tracks,1);
%         trackDescriptivesData.waitDuration = zeros(N_tracks,1);
%         trackDescriptivesData.overallCrossDuration = zeros(N_tracks,1);
%         
% 
%         %note the indices of pedestrians, crossing or waiting
%         pedCrossingTracks = [];
%         pedNotCrossingTracks = [];
%         pedWaitingTracks = [];
%         pedTracks = [];
%         carTracks = [];
%         carParkedTracks = [];
%         carMovingTracks = [];
%         pedJaywalkingTracks = [];
% 
%         % track loop starts
%         for ii=1:N_tracks
%             if ( strcmp(formattedTracksData{image_id}{ii,1}.class(1,:),'pedestrian'))       
%                 pedTracks = [pedTracks; ii];
% 
%                 if ( sum(formattedTracksData{image_id}{ii,1}.isCrossing) > 0 ) 
% 
%                     jaywalk_ind_temp = find(strcmp(formattedTracksData{image_id}{ii,1}.HybridState, 'Jaywalking') == 1);
%                     crossing_ind_temp = find(strcmp(formattedTracksData{image_id}{ii,1}.HybridState, 'Crossing') == 1);
%                     if ~isempty(crossing_ind_temp)
%                         pedCrossingTracks = [pedCrossingTracks; ii];
%                     else
%                         pedJaywalkingTracks = [pedJaywalkingTracks; ii];
%                     end
%                 else
%                     pedNotCrossingTracks = [pedNotCrossingTracks; ii];
%                 end
% 
%                 if ( find(strcmp(formattedTracksData{image_id}{ii,1}.HybridState, 'Wait') ==1 ) ) 
%                     pedWaitingTracks = [pedWaitingTracks; ii];
%                 end       
%             elseif( strcmp(formattedTracksData{image_id}{ii,1}.class(1,:),'car'))
%                 carTracks = [carTracks; ii];
% 
%                 if (mean(formattedTracksData{image_id}{ii,1}.lonVelocity) < 0.1 )
%                     carParkedTracks = [carParkedTracks; ii];
%                 else
%                     carMovingTracks = [carMovingTracks; ii];
%                 end       
%             end
% 
%             % track descriptives
%             trackDescriptivesData.duration(ii,1) = size(formattedTracksData{image_id}{ii,1},1);
%             trackDescriptivesData.avgLonVelocity(ii,1) = mean(formattedTracksData{image_id}{ii,1}.lonVelocity);
%             trackDescriptivesData.avgLonAcc(ii,1) = mean(formattedTracksData{image_id}{ii,1}.lonAcceleration);
%         end %track loop ends
% 
%         % pedestrians' loop starts
%         for ii=1:length(pedTracks)
%             ind = pedTracks(ii);
% 
%             % time indices for the various hybrid states
%             cross_time_ind = find(strcmp(formattedTracksData{image_id}{ind,1}.HybridState, 'Crossing') == 1);
%             jaywalk_time_ind = find(strcmp(formattedTracksData{image_id}{ind,1}.HybridState, 'Jaywalking') == 1);
%             wait_time_ind = find(strcmp(formattedTracksData{image_id}{ind,1}.HybridState, 'Wait') == 1);
%             approach_time_ind = find(strcmp(formattedTracksData{image_id}{ind,1}.HybridState, 'Approach') == 1);
%             walkaway_time_ind = find(strcmp(formattedTracksData{image_id}{ind,1}.HybridState, 'Walk_away') == 1);
% 
%             %velocity
%             velocity = sqrt(formattedTracksData{image_id}{ind,1}.lonVelocity.^2 + formattedTracksData{image_id}{ind,1}.latVelocity.^2);
% 
%             % durations
%             trackDescriptivesData.overallWaitDuration(ind,1) = size(wait_time_ind,1);
%             trackDescriptivesData.ovarellCrossDuration(ind,1) = size(cross_time_ind,1);
%             if ~isempty(approach_time_ind)
%                 trackDescriptivesData.approachSpeed(ind,1) = mean(velocity(approach_time_ind));
%             end
%             if ~isempty(cross_time_ind)
%                 trackDescriptivesData.crossSpeed(ind,1) = mean(velocity(cross_time_ind));
%             end
%             if ~isempty(walkaway_time_ind)
%                 trackDescriptivesData.walkawaySpeed(ind,1) = mean(velocity(walkaway_time_ind));
%             end
%             if ~isempty(jaywalk_time_ind)
%                 trackDescriptivesData.jaywalkSpeed(ind,1) = mean(velocity(jaywalk_time_ind));
%             end
%             
%             
%             % wait duration for each crossing
%             wait_changes = diff(wait_time_ind);
%             wait_start_indices = find(wait_changes~=1) + 1;
%             if ~isempty(wait_time_ind)
%                 wait_start_indices = [1; wait_start_indices];
%             end
%             % find corresponding crossing indices if any
%             cross_jaywalk_time_ind = sort([cross_time_ind; jaywalk_time_ind]);
%             cross_changes = diff(cross_jaywalk_time_ind);
%             cross_start_indices = find(cross_changes~=1) + 1;
%             if ~isempty(cross_jaywalk_time_ind)
%                 cross_start_indices = [1; cross_start_indices];
%             end
%             % approach
%             approach_changes = diff(approach_time_ind);
%             wait_end_indices = find(approach_changes~=1) + 1;
%             if ~isempty(approach_time_ind)
%                 wait_end_indices = [1; wait_end_indices];
%             end
% 
%             ww = 1;
%             cc = 1;
%             nextWaitInstance = false;
%             
%             wait_time = [];
%             if ~isempty(wait_time_ind)               
%                 for ww = 1:length(wait_start_indices)
%                     if ~isempty(wait_end_indices)
%                         while( cc <= length(wait_end_indices) && ~nextWaitInstance)                        
%                             if approach_time_ind(wait_end_indices(cc)) > wait_time_ind(wait_start_indices(ww))
%                                 wait_time = [wait_time; approach_time_ind(wait_end_indices(cc))- wait_time_ind(wait_start_indices(ww))];
%                                 nextWaitInstance = true;
%                             end
%                             cc = cc+1;
%                         end
%                     else
%                          wait_time = [wait_time, length(wait_time_ind)];
%                     end
%                 end
%             end
% 
% %                 for ww = 1:length(wait_start_indices)
% %                     if ~isempty(cross_start_indices)
% %                         for cc = 1:length(wait_end_indices)
% %                             if cross_time_ind(wait_end_indices(cc)) > wait_time_ind(wait_start_indices(ww))
% %                                 wait_time = [wait_time; approach_time_ind(wait_end_indices(cc))- wait_time_ind(wait_start_indices(ww))];
% %                                 break;
% %                             end
% %                         end
% %                     else
% %                         wait_time = length(wait_time_ind);
% %                     end
% %                 end
% %             end
%             
%             trackDescriptivesData.waitDuration(ind,1) = mean(wait_time);
%             
%            
% 
%         end %pedestrians' loop ends
% 
%         % save the tracks data for this scene
%         trackDescriptivesComplete{image_id} = trackDescriptivesData;
%         tracks{image_id}.pedCrossingTracks = pedCrossingTracks;
%         tracks{image_id}.pedNotCrossingTracks = pedNotCrossingTracks;
%         tracks{image_id}.pedJaywalkingTracks = pedJaywalkingTracks;
%         tracks{image_id}.pedWaitingTracks = pedWaitingTracks;
%         tracks{image_id}.pedTracks = pedTracks;
%         tracks{image_id}.carTracks = carTracks;
%         tracks{image_id}.carParkedTracks = carParkedTracks;
%         tracks{image_id}.carMovingTracks = carMovingTracks;
% 
%     end
% 
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     
%     
% %% compile the track descriptives for the 12 scenes
% compiledDescriptives.pedApproachSpeed = [];
% compiledDescriptives.pedCrossSpeed = [];
% compiledDescriptives.pedWalkAwaySpeed = [];
% compiledDescriptives.waitDuration = [];
% 
% for ii=1:length(tracks)
%     
%     pedTracks = tracks{ii}.pedTracks;
%     pedWaitingTracks = tracks{ii}.pedWaitingTracks;
%     trackDescriptivesData  = trackDescriptivesComplete{ii};
%     
%     compiledDescriptives.pedApproachSpeed = [compiledDescriptives.pedApproachSpeed; trackDescriptivesData.approachSpeed(pedTracks)];
%     compiledDescriptives.pedCrossSpeed   = [compiledDescriptives.pedCrossSpeed; trackDescriptivesData.crossSpeed(pedTracks)];
%     compiledDescriptives.pedWalkAwaySpeed = [compiledDescriptives.pedWalkAwaySpeed; trackDescriptivesData.walkawaySpeed(pedTracks)];
%     compiledDescriptives.waitDuration = [compiledDescriptives.waitDuration; trackDescriptivesData.waitDuration(pedWaitingTracks)/25];
%     
% end
% 
% 
% % compile all the tracks for the 12 scenes
% NoOfTracks.N_ped = 0;
% NoOfTracks.N_ped_crossing_intent = 0;
% NoOfTracks.N_ped_jaywalk = 0;
% NoOfTracks.N_ped_cross = 0;
% NoOfTracks.N_ped_wait_jaywalk = 0;
% NoOfTracks.N_ped_wait_cross = 0;
% NoOfTracks.N_car = 0;
% NoOfTracks.N_parked_cars = 0;
% 
% for ii=1:length(tracks)
%     NoOfTracks.N_ped = NoOfTracks.N_ped + size(tracks{ii}.pedTracks,1);
%     NoOfTracks.N_ped_jaywalk = NoOfTracks.N_ped_jaywalk + size(tracks{ii}.pedJaywalkingTracks,1);
%     NoOfTracks.N_ped_cross = NoOfTracks.N_ped_cross + size(tracks{ii}.pedCrossingTracks,1);
%     
%     NoOfTracks.N_ped_wait_jaywalk = NoOfTracks.N_ped_wait_jaywalk + size(intersect(tracks{ii}.pedWaitingTracks, tracks{ii}.pedJaywalkingTracks),1);
%     NoOfTracks.N_ped_wait_cross = NoOfTracks.N_ped_wait_cross + size(intersect(tracks{ii}.pedWaitingTracks, tracks{ii}.pedCrossingTracks),1);
% 
%     NoOfTracks.N_car = NoOfTracks.N_car + size(tracks{ii}.carTracks,1);
%     NoOfTracks.N_parked_cars = NoOfTracks.N_parked_cars + size(tracks{ii}.carParkedTracks,1);
%     
%     
% end
% 
% 
% NoOfTracks.N_moving_cars = NoOfTracks.N_car - NoOfTracks.N_parked_cars;
% NoOfTracks.N_ped_crossing_intent = NoOfTracks.N_ped_jaywalk + NoOfTracks.N_ped_cross;
% NoOfTracks.N_ped_no_crossing_intent = NoOfTracks.N_ped - NoOfTracks.N_ped_crossing_intent;
% 
% 
% %% save necessary variables
% % check the name of the file
% save('inD_trackDescriptives_v4.mat','NoOfTracks','tracks','trackDescriptivesComplete','compiledDescriptives')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    



%% calculate wait distribution from compiled descriptives
load('inD_trackDescriptives_v4.mat')
wait_times = [];
for sceneId = 1:12
    pedCrossingTracks = [tracks{sceneId}.pedCrossingTracks; tracks{sceneId}.pedJaywalkingTracks];
    
    for track_index = 1:length(pedCrossingTracks)
        isEgoCarPresent = false;
        pedTrackId = pedCrossingTracks(track_index);
        pedData = formattedTracksData{sceneId}{pedTrackId};
        wait_indices_in_track = find(strcmp(pedData.HybridState,'Wait'));
        for ii = 1:length(wait_indices_in_track)
            pedTrackTimeStep = wait_indices_in_track(ii);           
            if pedData.closeCar_ind(pedTrackTimeStep)~=0 && pedData.closeCar_ind(pedTrackTimeStep)~=inf
               isEgoCarPresent = true;
            end
        end
        
        if isEgoCarPresent
            wait_times = [wait_times; trackDescriptivesComplete{sceneId}.waitDuration(pedTrackId)];
        end
    end

end
wait_times(isnan(wait_times)) = 0;
wait_times = wait_times*0.2;
wait_times_no_outliers = wait_times;
wait_times_no_outliers(wait_times_no_outliers>14.4) = [];

% plot histogram
figure()
h3 = histogram(wait_times_no_outliers,'Normalization','probability','BinWidth',1);
xlabel('Wait[s]')
ylabel('Probability')
title('Waitdistribution')

boxplot(wait_times)