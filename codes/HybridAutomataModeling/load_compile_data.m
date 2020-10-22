function [formattedTracksData, tracksMetaData, tracks] = load_compile_data(flag, dataset)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % compile/data tracks data if flag.dataCompile is set to true
% % a) Run the following if compiling data for the first time, else
% %     run1b) 
if flag.dataCompile
%     % for full data compilation
%     [formattedTracksData, allTracksMetaData, N_Scenes] = inD_compile(Params, resetStates);
%     [tracks, ~] = trackDescriptives(formattedTracksData, N_scenes);
%     %%%%%%%%%%%%%%%%
    % for recompiling the data from earlier resampled data
    load('tracksData_reSampled_v11.mat')     
    load('inD_trackDescriptives_v3.mat') 
    %%%%%%%%%%%%%%%%%
    % hybrid state
    inD_compile_resampled;
    %check for ego-pedestrian and pedestrian gaze for the entire dataset
    egoPed_Gaze_HPed_v2;
    %check pedestrian lane
    tmp_nearLaneCalc;
    
%     for scene_id = 1:N_scenes
%         tracksMetaData{scene_id}.ego_veh_gap_hist(1:size(tracksMetaData{scene_id},1))  = {zeros(20,1)};  % for inD dataset the maximum number of gaps
%         tracksMetaData{scene_id}.wait_start_hist(1:size(tracksMetaData{scene_id},1))  = {zeros(20,1)}; 
%     end

    % save the data file for later reuse
    save('tracksData_reSampled_v11.mat','formattedTracksData','tracksMetaData','-v7.3','-nocompression')
    x = 1;

else
    
    % b) load already compiled tracks data
    load('tracksData_reSampled_v11.mat')
    load('inD_trackDescriptives_v3.mat') 
%     tracks =  tracksUpdated;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%