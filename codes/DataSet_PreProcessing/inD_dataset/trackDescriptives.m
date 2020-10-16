%% This script finds the descriptives of the tracks

%% Note:
% [1] Check the file name of the track descriptives mat file so that any
% existing data file is not replaced.


% Inputs:
% 1) formattedTracksData: A cell of all the individual tracks from all the
% scenes
% 2) N_scenes: The no. of the individual scenes

% function [tracks, trackDescriptivesData] = trackDescriptives(formattedTracksData, N_scenes)


    for image_id = 1:N_scenes

        N_tracks = size(formattedTracksData{image_id},1);

        % initialize the variables
        trackDescriptivesData.approachSpeed = zeros(N_tracks,1);
        trackDescriptivesData.crossSpeed = zeros(N_tracks,1);
        trackDescriptivesData.walkawaySpeed = zeros(N_tracks,1);
        trackDescriptivesData.waitDuration = zeros(N_tracks,1);
        trackDescriptivesData.crossDuration = zeros(N_tracks,1);
        

        %note the indices of pedestrians, crossing or waiting
        pedCrossingTracks = [];
        pedNotCrossingTracks = [];
        pedWaitingTracks = [];
        pedTracks = [];
        carTracks = [];
        carParkedTracks = [];
        carMovingTracks = [];
        pedJaywalkingTracks = [];

        % track loop starts
        for ii=1:N_tracks
            if ( strcmp(formattedTracksData{image_id}{ii,1}.class(1,:),'pedestrian'))       
                pedTracks = [pedTracks; ii];

                if ( sum(formattedTracksData{image_id}{ii,1}.isCrossing) > 0 ) 

                    jaywalk_ind_temp = find(strcmp(formattedTracksData{image_id}{ii,1}.HybridState, 'Jaywalking') == 1);
                    crossing_ind_temp = find(strcmp(formattedTracksData{image_id}{ii,1}.HybridState, 'Crossing') == 1);
                    if ~isempty(crossing_ind_temp)
                        pedCrossingTracks = [pedCrossingTracks; ii];
                    else
                        pedJaywalkingTracks = [pedJaywalkingTracks; ii];
                    end
                else
                    pedNotCrossingTracks = [pedNotCrossingTracks; ii];
                end

                if ( find(strcmp(formattedTracksData{image_id}{ii,1}.HybridState, 'Wait') ==1 ) ) 
                    pedWaitingTracks = [pedWaitingTracks; ii];
                end       
            elseif( strcmp(formattedTracksData{image_id}{ii,1}.class(1,:),'car'))
                carTracks = [carTracks; ii];

                if (mean(formattedTracksData{image_id}{ii,1}.lonVelocity) < 0.1 )
                    carParkedTracks = [carParkedTracks; ii];
                else
                    carMovingTracks = [carMovingTracks; ii];
                end       
            end

            % track descriptives
            trackDescriptivesData.duration(ii,1) = size(formattedTracksData{image_id}{ii,1},1);
            trackDescriptivesData.avgLonVelocity(ii,1) = mean(formattedTracksData{image_id}{ii,1}.lonVelocity);
            trackDescriptivesData.avgLonAcc(ii,1) = mean(formattedTracksData{image_id}{ii,1}.lonAcceleration);
        end %track loop ends

        % pedestrians' loop starts
        for ii=1:length(pedTracks)
            ind = pedTracks(ii);

            % time indices for the various hybrid states
            cross_time_ind = find(strcmp(formattedTracksData{image_id}{ind,1}.HybridState, 'Crossing') == 1);
            jaywalk_time_ind = find(strcmp(formattedTracksData{image_id}{ind,1}.HybridState, 'Jaywalking') == 1);
            wait_time_ind = find(strcmp(formattedTracksData{image_id}{ind,1}.HybridState, 'Wait') == 1);
            approach_time_ind = find(strcmp(formattedTracksData{image_id}{ind,1}.HybridState, 'Approach') == 1);
            walkaway_time_ind = find(strcmp(formattedTracksData{image_id}{ind,1}.HybridState, 'Walk_away') == 1);

            %velocity
            velocity = sqrt(formattedTracksData{image_id}{ind,1}.lonVelocity.^2 + formattedTracksData{image_id}{ind,1}.latVelocity.^2);

            % durations
            trackDescriptivesData.waitDuration(ind,1) = size(wait_time_ind,1);
            trackDescriptivesData.crossDuration(ind,1) = size(cross_time_ind,1);
            if ~isempty(approach_time_ind)
                trackDescriptivesData.approachSpeed(ind,1) = mean(velocity(approach_time_ind));
            end
            if ~isempty(cross_time_ind)
                trackDescriptivesData.crossSpeed(ind,1) = mean(velocity(cross_time_ind));
            end
            if ~isempty(walkaway_time_ind)
                trackDescriptivesData.walkawaySpeed(ind,1) = mean(velocity(walkaway_time_ind));
            end
            if ~isempty(jaywalk_time_ind)
                trackDescriptivesData.jaywalkSpeed(ind,1) = mean(velocity(jaywalk_time_ind));
            end

        end %pedestrians' loop ends

        % save the tracks data for this scene
        trackDescriptivesComplete{image_id} = trackDescriptivesData;
        tracks{image_id}.pedCrossingTracks = pedCrossingTracks;
        tracks{image_id}.pedNotCrossingTracks = pedNotCrossingTracks;
        tracks{image_id}.pedJaywalkingTracks = pedJaywalkingTracks;
        tracks{image_id}.pedWaitingTracks = pedWaitingTracks;
        tracks{image_id}.pedTracks = pedTracks;
        tracks{image_id}.carTracks = carTracks;
        tracks{image_id}.carParkedTracks = carParkedTracks;
        tracks{image_id}.carMovingTracks = carMovingTracks;

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
%% compile the track descriptives for the 12 scenes
compiledDescriptives.pedApproachSpeed = [];
compiledDescriptives.pedCrossSpeed = [];
compiledDescriptives.pedWalkAwaySpeed = [];
compiledDescriptives.waitDuration = [];

for ii=1:length(tracks)
    
    pedTracks = tracks{ii}.pedTracks;
    pedWaitingTracks = tracks{ii}.pedWaitingTracks;
    trackDescriptivesData  = trackDescriptivesComplete{ii};
    
    compiledDescriptives.pedApproachSpeed = [compiledDescriptives.pedApproachSpeed; trackDescriptivesData.approachSpeed(pedTracks)];
    compiledDescriptives.pedCrossSpeed   = [compiledDescriptives.pedCrossSpeed; trackDescriptivesData.crossSpeed(pedTracks)];
    compiledDescriptives.pedWalkAwaySpeed = [compiledDescriptives.pedWalkAwaySpeed; trackDescriptivesData.walkawaySpeed(pedTracks)];
    compiledDescriptives.waitDuration = [compiledDescriptives.waitDuration; trackDescriptivesData.waitDuration(pedWaitingTracks)/25];
    
end


% compile all the tracks for the 12 scenes
NoOfTracks.N_ped = 0;
NoOfTracks.N_ped_crossing_intent = 0;
NoOfTracks.N_ped_jaywalk = 0;
NoOfTracks.N_ped_cross = 0;
NoOfTracks.N_ped_wait_jaywalk = 0;
NoOfTracks.N_ped_wait_cross = 0;
NoOfTracks.N_car = 0;
NoOfTracks.N_parked_cars = 0;

for ii=1:length(tracks)
    NoOfTracks.N_ped = NoOfTracks.N_ped + size(tracks{ii}.pedTracks,1);
    NoOfTracks.N_ped_jaywalk = NoOfTracks.N_ped_jaywalk + size(tracks{ii}.pedJaywalkingTracks,1);
    NoOfTracks.N_ped_cross = NoOfTracks.N_ped_cross + size(tracks{ii}.pedCrossingTracks,1);
    
    NoOfTracks.N_ped_wait_jaywalk = NoOfTracks.N_ped_wait_jaywalk + size(intersect(tracks{ii}.pedWaitingTracks, tracks{ii}.pedJaywalkingTracks),1);
    NoOfTracks.N_ped_wait_cross = NoOfTracks.N_ped_wait_cross + size(intersect(tracks{ii}.pedWaitingTracks, tracks{ii}.pedCrossingTracks),1);

    NoOfTracks.N_car = NoOfTracks.N_car + size(tracks{ii}.carTracks,1);
    NoOfTracks.N_parked_cars = NoOfTracks.N_parked_cars + size(tracks{ii}.carParkedTracks,1);
    
    
end


NoOfTracks.N_moving_cars = NoOfTracks.N_car - NoOfTracks.N_parked_cars;
NoOfTracks.N_ped_crossing_intent = NoOfTracks.N_ped_jaywalk + NoOfTracks.N_ped_cross;
NoOfTracks.N_ped_no_crossing_intent = NoOfTracks.N_ped - NoOfTracks.N_ped_crossing_intent;


%% save necessary variables
% check the name of the file
save('inD_trackDescriptives_v3.mat','NoOfTracks','tracks','trackDescriptivesComplete','compiledDescriptives')

    

% end