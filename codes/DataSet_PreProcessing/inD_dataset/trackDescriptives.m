% %% This script finds the descriptives of the tracks and the pedestrians
% 
% 
% for image_id = 1:length(images)
% 
% N_tracks = size(formattedTracksData{image_id},1);
% 
% % initialize the variables
% trackDescriptivesData.approachSpeed = zeros(N_tracks,1);
% trackDescriptivesData.crossSpeed = zeros(N_tracks,1);
% trackDescriptivesData.walkawaySpeed = zeros(N_tracks,1);
% trackDescriptivesData.waitDuration = zeros(N_tracks,1);
% trackDescriptivesData.crossDuration = zeros(N_tracks,1);
% 
% 
% %note the indices of pedestrians, crossing or waiting
% ped_crossing_tracks = [];
% ped_waiting_tracks = [];
% ped_tracks = [];
% car_tracks = [];
% car_parked_tracks = [];
% ped_jaywalking_tracks = [];
% 
% track_id = 1;
% 
% for ii=1:N_tracks
%     if ( strcmp(formattedTracksData{image_id}{ii,1}.class(1,:),'pedestrian'))       
%         ped_tracks = [ped_tracks; ii];
%         
%         if ( sum(formattedTracksData{image_id}{ii,1}.isCrossing) > 0 ) 
%             
%             jaywalk_ind_temp = find(strcmp(formattedTracksData{image_id}{ii,1}.HybridState, 'Jaywalking') == 1);
%             crossing_ind_temp = find(strcmp(formattedTracksData{image_id}{ii,1}.HybridState, 'Crossing') == 1);
%             if ~isempty(crossing_ind_temp)
%                 ped_crossing_tracks = [ped_crossing_tracks; ii];
%             else
%                 ped_jaywalking_tracks = [ped_jaywalking_tracks; ii];
%             end
%         end
%         
%         if ( find(strcmp(formattedTracksData{image_id}{ii,1}.HybridState, 'Wait') ==1 ) ) 
%             ped_waiting_tracks = [ped_waiting_tracks; ii];
%         end       
%     elseif( strcmp(formattedTracksData{image_id}{ii,1}.class(1,:),'car'))
%         car_tracks = [car_tracks; ii];
%      
%         if (mean(formattedTracksData{image_id}{ii,1}.lonVelocity) < 0.1 )
%             car_parked_tracks = [car_parked_tracks; ii];
%         end       
%     end
%     
%     % track descriptives
%     trackDescriptivesData.duration(ii,1) = size(formattedTracksData{image_id}{ii,1},1);
%     trackDescriptivesData.avgLonVelocity(ii,1) = mean(formattedTracksData{image_id}{ii,1}.lonVelocity);
%     trackDescriptivesData.avgLonAcc(ii,1) = mean(formattedTracksData{image_id}{ii,1}.lonAcceleration);
%     
% 
% end
% 
% for ii=1:length(ped_tracks)
%     ind = ped_tracks(ii);
%     
%     % time indices for the various hybrid states
%     cross_time_ind = find(strcmp(formattedTracksData{image_id}{ind,1}.HybridState, 'Crossing') == 1);
%     jaywalk_time_ind = find(strcmp(formattedTracksData{image_id}{ind,1}.HybridState, 'Jaywalking') == 1);
%     wait_time_ind = find(strcmp(formattedTracksData{image_id}{ind,1}.HybridState, 'Wait') == 1);
%     approach_time_ind = find(strcmp(formattedTracksData{image_id}{ind,1}.HybridState, 'Approach') == 1);
%     walkaway_time_ind = find(strcmp(formattedTracksData{image_id}{ind,1}.HybridState, 'Walk_away') == 1);
%     
%     %velocity
%     velocity = sqrt(formattedTracksData{image_id}{ind,1}.lonVelocity.^2 + formattedTracksData{image_id}{ind,1}.latVelocity.^2);
%     
% %    temp_ind = find();
%     
%     % durations
%     trackDescriptivesData.waitDuration(ind,1) = size(wait_time_ind,1);
%     trackDescriptivesData.crossDuration(ind,1) = size(cross_time_ind,1);
%     if ~isempty(approach_time_ind)
%         trackDescriptivesData.approachSpeed(ind,1) = mean(velocity(approach_time_ind));
%     end
%     if ~isempty(cross_time_ind)
%         trackDescriptivesData.crossSpeed(ind,1) = mean(velocity(cross_time_ind));
%     end
%     if ~isempty(walkaway_time_ind)
%         trackDescriptivesData.walkawaySpeed(ind,1) = mean(velocity(walkaway_time_ind));
%     end
% 
% end
% 
% %
% trackDescriptivesComplete{image_id} = trackDescriptivesData;
% tracks{image_id}.ped_crossing_tracks = ped_crossing_tracks;
% tracks{image_id}.ped_jaywalking_tracks = ped_jaywalking_tracks;
% tracks{image_id}.ped_waiting_tracks = ped_waiting_tracks;
% tracks{image_id}.ped_tracks = ped_tracks;
% tracks{image_id}.car_tracks = car_tracks;
% tracks{image_id}.car_parked_tracks = car_parked_tracks;
% 
% 
% 
% 
% 
% 
% end


pedApproachSpeed = [];
pedCrossSpeed = [];
pedWalkAwaySpeed = [];
waitDuration = [];

for ii=1:12
    
    ped_tracks = tracks{ii}.ped_tracks;
    ped_waiting_tracks = tracks{ii}.ped_waiting_tracks;
    trackDescriptivesData  = trackDescriptivesComplete{ii};
    
    pedApproachSpeed = [pedApproachSpeed; trackDescriptivesData.approachSpeed(ped_tracks)];
    pedCrossSpeed   = [pedCrossSpeed; trackDescriptivesData.crossSpeed(ped_tracks)];
    pedWalkAwaySpeed = [pedWalkAwaySpeed; trackDescriptivesData.walkawaySpeed(ped_tracks)];
    waitDuration = [waitDuration; trackDescriptivesData.waitDuration(ped_waiting_tracks)/25];
    
    
    
end







waitDuration_2 = waitDuration;
waitDuration_2(waitDuration_2>12.6) = [];


boxplot(waitDuration_2)




disp('Average approach speed of pedestrians... \n')
mean( pedApproachSpeed)

disp('Average cross speed of pedestrians... \n')
mean( pedCrossSpeed)

disp('Average walkaway speed of pedestrians... \n')
mean( pedWalkAwaySpeed)


figure()
plotbar = bar([mean( pedApproachSpeed), mean( pedCrossSpeed), mean( pedWalkAwaySpeed)]);

figure()
subplot(2,2,1)
histogram(pedApproachSpeed(pedApproachSpeed>0.4),'Normalization','probability');
ylabel('Probability')
xlabel('Approach speed')

subplot(2,2,2)
histogram(pedCrossSpeed(pedCrossSpeed>0.2), 'Normalization','probability');
ylabel('Probability')
xlabel('Cross speed')

subplot(2,2,3)
histogram(pedWalkAwaySpeed(pedWalkAwaySpeed>0.3), 'Normalization','probability');
ylabel('Probability')
xlabel('Walkaway speed')

subplot(2,2,4)
histogram(waitDuration_2, 'Normalization','probability');
ylabel('Probability')
xlabel('Wait Time')