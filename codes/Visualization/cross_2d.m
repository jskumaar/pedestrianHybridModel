load('gaze.mat');

%crossing time gaze data
            gaze_cross_ind = [];
            for ii=1:3
                 gaze_cross_ind = [gaze_cross_ind Ain_ind(ii):1:Bout_ind(ii) Bin_ind(ii):1:Aout_ind(ii)];
            end

            gaze_cross =[];
            gaze_cross = gaze_obj(gaze_cross_ind,[1 2 3 4 5 9]);
            gaze_cross_size_inital = length(gaze_cross);
            %removing empty gaze data
            gaze_rem_ind_eye0 = find(strcmp(gaze_cross(:,5),'eye'));
            gaze_rem_ind_eye1 = find(strcmp(gaze_cross(:,6),')'));
            idx = ismember( gaze_rem_ind_eye0,  gaze_rem_ind_eye1);
            gaze_cross(gaze_rem_ind_eye0(idx),:) = [];
            gaze_cross_size_final = length(gaze_cross);

            %splitting gaze data into position and tags
            gaze_cross_position = gaze_cross(:,1:4);
            gaze_cross = gaze_cross(:,[1 5 6]); 

 
figure()
for ii=1:length(gaze_cross)
                %check for Traffic light
                
                if(~isempty(strfind(gaze_cross{ii,2},'Traffic'))||~isempty(strfind(gaze_cross{ii,3},'Traffic')))
                    scatter(str2double(gaze_cross_position(ii,2)),str2double(gaze_cross_position(ii,3)),'r');
                    hold on;
                
                     
                %check for Ped light
                else if(~isempty(strfind(gaze_cross{ii,2},'Ped'))||~isempty(strfind(gaze_cross{ii,3},'Ped')))
                     scatter(str2double(gaze_cross_position(ii,2)),str2double(gaze_cross_position(ii,3)),'b');
                      hold on;               
%                     
                %check for vehicles
                    else if(~isempty(strfind(gaze_cross{ii,2},'Vehicle'))||~isempty(strfind(gaze_cross{ii,3},'Vehicle')))
                     scatter(str2double(gaze_cross_position(ii,2)),str2double(gaze_cross_position(ii,3)),'k');
                 hold on;
%                     
                 %check for crosswalk buildings when user is on near side
                            else if(~isempty(strfind(gaze_cross{ii,2},'Crosswalk Building'))||~isempty(strfind(gaze_cross{ii,3},'Crosswalk Building')))
                                scatter(str2double(gaze_cross_position(ii,2)),str2double(gaze_cross_position(ii,3)),'g');
                                hold on;
                %check for crosswalk buildings when user is on far side
                                else if(~isempty(strfind(gaze_cross{ii,2},'Untagged Untagged eye'))||~isempty(strfind(gaze_cross{ii,3},'Untagged Untagged')))
                                    scatter(str2double(gaze_cross_position(ii,2)),str2double(gaze_cross_position(ii,3)),'y');
                                    hold on;
                %check for crosswalk road pieces
                                    else if(~isempty(strfind(gaze_cross{ii,2},'Crosswalk_piece'))||~isempty(strfind(gaze_cross{ii,2},'Crosswalk')))
                                        scatter(str2double(gaze_cross_position(ii,2)),str2double(gaze_cross_position(ii,3)),'*','r');
                                        hold on;
                %check for task;the remaining data are
                %untagged buildings
                            else if(~isempty(strfind(gaze_cross{ii,2},'Task'))||~isempty(strfind(gaze_cross{ii,3},'Task'))||~isempty(strfind(gaze_cross{ii,2},'Ball'))||~isempty(strfind(gaze_cross{ii,3},'Ball')))
                                     scatter(str2double(gaze_cross_position(ii,2)),str2double(gaze_cross_position(ii,3)),'m');
                              hold on;
                                else
                                scatter(str2double(gaze_cross_position(ii,2)),str2double(gaze_cross_position(ii,3)),'c');
                                hold on;
                                end
                            end
                        end
                    end
                end
                    end
                end
end