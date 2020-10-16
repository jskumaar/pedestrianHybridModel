%% This script is the main script needed to compile the data from inD intersection dataset

function [formattedTracksData] = inD_compile(Params, annotatedImage_enhanced, cw, reset)
%function [allTracksMetaData, N_scenes, annotatedImage_enhanced, cw] = inD_compile(SampFreq, AdjustedSampFreq)

% set of images to load
images = [18:1:29];
% images = 18;
N_scenes = length(images);
skip_ts = Params.reSampleRate;
orthopxToMeter = Params.orthopxToMeter;

%identify the centers of the crosswalks; use the knowledge of the
%scene; we know that there are two crosswalks and they are on the
%opposite ends of the intersection; use x = 500 as the mid-point for x to
%differentiate the two crosswalks       
cw.center_lat_offset = int32([27, 24, 30, 27]);     %this was calculated manually from the images

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% loop starts
for scene_id = 1:length(images)
    image_no = images(scene_id);
    
    % Process tracks corresponding to the image no
    tracksData = (readtable(strcat(num2str(image_no),'_tracks.csv')));
    tracksMetaData = readtable(strcat(num2str(image_no),'_tracksMeta.csv'));
%     recordingMetaData = readtable(strcat(num2str(image_no),'_recordingMeta.csv'));
    tempTrackDiff = diff(tracksData.trackId); 
    % indices for track (is same as trackId when the number of scenes is just 1)
    new_track_end_ind = find(tempTrackDiff~=0);
    new_track_start_ind = [1; new_track_end_ind+1];
    new_track_end_ind = [new_track_end_ind; size(tracksData,1)];
    %initialize additional tracks' data
    tracksData.xCenterPix([1:size(tracksData,1)]) = 0;
    tracksData.yCenterPix([1:size(tracksData,1)]) = 0;
    tracksData.distCW([1:size(tracksData,1)]) = inf;
    tracksData.closestCW([1:size(tracksData,1)]) = 0;
    %loop count
    track_ind = 1;
    N_tracks = size(tracksMetaData,1);

% inner loop starts
for track_id = 1:N_tracks
    % add class data, i.e. whether the track is a car or a pedestrian
    tracksData.class([new_track_start_ind(track_id):new_track_end_ind(track_id)]) = tracksMetaData.class(track_id);   
    % frame indices (note that this is before resampling)
    track_start_frame = new_track_start_ind(track_id);
    track_end_frame = new_track_end_ind(track_id);

    % if frame does not start from initial time step or other sampling
    % time, move it to the next sampling time
    if  tracksData.frame(track_start_frame)~=0 
        if mod(tracksData.frame(track_start_frame),skip_ts)~=0 
            track_start_frame = track_start_frame + (skip_ts - mod(tracksData.frame(track_start_frame),skip_ts) );
        end
    end
        
    % split the data according to its track index and use that for
    % subsequent processing   
    formattedTracksData{scene_id}{track_ind,1} = table2struct(formattedTracksData{scene_id}{track_ind,1} , 'ToScalar', true);
%     formattedTracksData{scene_id}{track_ind,1} = tracksData([track_start_frame : skip_ts : track_end_frame], :);
    
    % a) identify the hybrid state of the pedestrians and distance to closest CW (if the track is a
    % pedestrian track)
    % check if track is pedestrian
    flag.hybridStatePred = false;
    if strcmp(formattedTracksData{scene_id}{track_ind,1}.class{1}, 'pedestrian')   
        [formattedTracksData{scene_id}{track_ind,1}] = ....
                hybridStateCopy_v3(formattedTracksData{scene_id}{track_ind,1}, cw, flag, annotatedImage_enhanced, Params, 1);    
        % plot background with all pedestrian tracks for this scene
        % imshow(annotatedImage_enhanced_w_tracks)  
    end   
    % b) identify the lane and the distance to the closest CW (if the track is a car track)  
    if strcmp(formattedTracksData{scene_id}{track_ind,1}.class{1}, 'car')  
        formattedTracksData{scene_id}{track_ind,1}  = carState(formattedTracksData{scene_id}{track_ind,1}, cw, road_center, Params);
    end
     
    % update loop id
    track_ind = track_ind+1;
end % inner loop ends for the image
% 
allTracksMetaData{scene_id} = tracksMetaData;
% 
end % loop ends for all images
% 
end % function end