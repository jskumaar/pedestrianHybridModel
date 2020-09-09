%% This script is the main script needed to compile the data from inD intersection dataset

function [formattedTracksData, allTracksMetaData, N_scenes, annotatedImage_enhanced] = inD_compile(Params, reset)
%function [allTracksMetaData, N_scenes, annotatedImage_enhanced, cw] = inD_compile(SampFreq, AdjustedSampFreq)

% set of images to load
images = [18:1:29];
% images = 18;
N_scenes = length(images);
skip_ts = Params.reSampleRate;

% load background images
annotatedImage = imread(strcat('annotated_',num2str(18),'_background.png'));        %using the same background image for all scenes as it roughly remains the same; also segmenting all images takes time.
img_size = size(annotatedImage);

% for better visualization, increase the grayscale value of the different regions
annotatedImage_enhanced = annotatedImage;
for ii = 1:img_size(1)
    for jj = 1:img_size(2)
        if (annotatedImage(ii,jj)==2) %Road
            annotatedImage_enhanced(ii,jj) = 50;
        end
        if (annotatedImage(ii,jj)==4 || annotatedImage(ii,jj)==5 || annotatedImage(ii,jj)==6) %unmarked crosswalk
            annotatedImage_enhanced(ii,jj) = 100;
        end        
        if (annotatedImage(ii,jj)==3) %Sidewalk
            annotatedImage_enhanced(ii,jj) = 150;
        end        
        if (annotatedImage(ii,jj)==1) %marked crosswalk
            annotatedImage_enhanced(ii,jj) = 200;
        end
    end
end

%% center of the road
img_size = size(annotatedImage_enhanced);
road_pixels = [];
for ii = 1:img_size(1)
    for jj = 1:img_size(2)
        if annotatedImage_enhanced(ii,jj)==50
            road_pixels = [road_pixels; [jj, -ii]];
        end
    end
end
road_center = int32(mean(road_pixels));
annotatedImage_enhanced(-road_center(2), road_center(1)) = 255;
annotatedImage_enhanced_w_tracks = annotatedImage_enhanced;

%identify the centers of the crosswalks; use the knowledge of the
%scene; we know that there are two crosswalks and they are on the
%opposite ends of the intersection; use x = 500 as the mid-point for x to
%differentiate the two crosswalks       
[cw.y1,cw.x1] = find(double(annotatedImage==1));
[cw.y2,cw.x2] = find(double(annotatedImage==4));
[cw.y3,cw.x3] = find(double(annotatedImage==5));
[cw.y4,cw.x4] = find(double(annotatedImage==6));

cw.center_y = -int32([mean(cw.y1), mean(cw.y2), mean(cw.y3), mean(cw.y4)]);
cw.center_x = int32([mean(cw.x1), mean(cw.x2), mean(cw.x3), mean(cw.x4)]);
cw.center_lat_offset = int32([27, 24, 30, 27]);     %this was calculated manually from the images

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% loop starts
for scene_id = 1:length(images)
    image_no = images(scene_id);
 
    % Process tracks corresponding to the image no
    tracksData = readtable(strcat(num2str(image_no),'_tracks.csv'));
    tracksMetaData = readtable(strcat(num2str(image_no),'_tracksMeta.csv'));
    recordingMetaData = readtable(strcat(num2str(image_no),'_recordingMeta.csv'));
    tempTrackDiff = diff(tracksData.trackId); 
    % pixel to meter scaling factor
    orthopxToMeter = recordingMetaData.orthoPxToMeter;
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
    formattedTracksData{scene_id}{track_ind,1} = tracksData([track_start_frame : skip_ts : track_end_frame], :);
    
    % a) identify the hybrid state of the pedestrians and distance to closest CW (if the track is a
    % pedestrian track)
    % check if track is pedestrian
    flag.pred = false;
    if strcmp(formattedTracksData{scene_id}{track_ind,1}.class{1}, 'pedestrian')   
        [formattedTracksData{scene_id}{track_ind,1}] = ....
                hybridStateCopy(formattedTracksData{scene_id}{track_ind,1}, cw, flag, annotatedImage_enhanced, Params);    
        % plot background with all pedestrian tracks for this scene
        % imshow(annotatedImage_enhanced_w_tracks)  
    end   
    % b) identify the lane and the distance to the closest CW (if the track is a car track)  
    if strcmp(formattedTracksData{scene_id}{track_ind,1}.class{1}, 'car')  
        formattedTracksData{scene_id}{track_ind,1}  = carState(formattedTracksData{scene_id}{track_ind,1}, cw, road_center, Params);
    end
      
%     %% add a wait time variable
%     N_instances = size(formattedTracksData{scene_id}{track_id},1);
%     if strcmp(formattedTracksData{scene_id}{track_id}.class(1), 'pedestrian')
%        wait_time_steps  = cumsum(formattedTracksData{scene_id}{track_id}.ProbHybridState(:,2))-1;   %exclude the 1st time step when wait started
%         if (sum(wait_time_steps~=0)~=0)
%             formattedTracksData{scene_id}{track_id}.wait_time_steps = wait_time_steps*skip_ts;
%         else
%             formattedTracksData{scene_id}{track_id}.wait_time_steps = -1*ones(N_instances,1);
%         end
%     end

      
    % update loop id
    track_ind = track_ind+1;
end % inner loop ends for the image
% 
allTracksMetaData{scene_id} = tracksMetaData;
% 
end % loop ends for all images
% 
end % function end