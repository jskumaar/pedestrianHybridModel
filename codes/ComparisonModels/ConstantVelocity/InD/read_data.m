function dataset_x = read_data(dataset_x)

global pred_horizon obs_horizon

    if (dataset_x.name == 'inD')

        %parameters
        dataset_x.pred_horizon = ceil(pred_horizon/dataset_x.deltaT);
        dataset_x.obs_horizon  = ceil(obs_horizon/dataset_x.deltaT);

        % add dataset path
        addpath(genpath(dataset_x.path))
        %count the number of tracks in the dataset
        files = dir(strcat(dataset_x.path,'\tracks'));
        N_scenes = length(files)-3; %3 default hidden files
        track_ind = 1; %reset track index to 1 for this dataset

        % read and compile the tracks data
        for jj=1:N_scenes
            tempData = readtable(files(jj+2).name);
            tempTrackDiff = diff(tempData.trackId);

            % indices for track
            new_track_end_ind = find(tempTrackDiff~=0);
            new_track_start_ind = [1; new_track_end_ind+1];
            new_track_end_ind = [new_track_end_ind; size(tempData,1)];
            N_tracks = size(new_track_start_ind,1);

            for kk = 1:N_tracks
                dataset_x.trackData{track_ind,1} = tempData([new_track_start_ind(kk):new_track_end_ind(kk)],:);
                track_ind = track_ind+1;
            end

        end


        files = dir(strcat(dataset_x.path,'\tracksMeta'));
        track_ind = 1;
        % read and compile the tracks data
        for jj=1:N_scenes
            tempData = readtable(files(jj+2).name);
            N_tracks = size(tempData,1);
            for kk = 1:N_tracks
                dataset_x.class{track_ind,1} = tempData.class(kk);
                track_ind = track_ind+1;
            end
        end


    end


end