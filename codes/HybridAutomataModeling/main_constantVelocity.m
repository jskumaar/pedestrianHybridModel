%% This scrits calculated predictions for a constant velocity model


% load formatted data
% load('tracksData_reSampled_compiled_all_ped_tracks.mat')
load('inD_trackDescriptives_removed_ped_tracks.mat')



N_scenes = 12;

for scene_id = 1:N_scenes
   N_pedTracks = length(tracks_updated{scene_id}.ped_tracks);
   
   for ii = 1:N_pedTracks
      track_id  = tracks_updated{scene_id}.ped_tracks(ii);
      
      N_instances = size(formattedTracksData{scene_id}{track_id},1);
       
       
   end
   
   
    
    
    
    
    
end




