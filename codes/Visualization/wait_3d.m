
 
figure()

for ii=250:450
                %check for Traffic light
                
                if(~isempty(strfind(gaze_wait{ii,2},'Traffic'))||~isempty(strfind(gaze_wait{ii,3},'Traffic')))
                    scatter3((gaze_wait_position(ii,2)),(gaze_wait_position(ii,3)),(gaze_wait_position(ii,4)),'r');
                    hold on;
                
                     
                %check for Ped light
                else if(~isempty(strfind(gaze_wait{ii,2},'Ped'))||~isempty(strfind(gaze_wait{ii,3},'Ped')))
                     scatter3((gaze_wait_position(ii,2)),(gaze_wait_position(ii,3)),(gaze_wait_position(ii,4)),'b');
                      hold on;               
%                     
                %check for vehicles
                    else if(~isempty(strfind(gaze_wait{ii,2},'Vehicle'))||~isempty(strfind(gaze_wait{ii,3},'Vehicle')))
                     scatter3((gaze_wait_position(ii,2)),(gaze_wait_position(ii,3)),(gaze_wait_position(ii,4)),'k');
                 hold on;
%                     
                 %check for crosswalk buildings when user is on near side
                            else if(~isempty(strfind(gaze_wait{ii,2},'Crosswalk Building'))||~isempty(strfind(gaze_wait{ii,3},'Crosswalk Building')))
                                scatter3((gaze_wait_position(ii,2)),(gaze_wait_position(ii,3)),(gaze_wait_position(ii,4)),'g');
                                hold on;
                %check for crosswalk buildings when user is on far side
                                else if(~isempty(strfind(gaze_wait{ii,2},'Untagged Untagged eye'))||~isempty(strfind(gaze_wait{ii,3},'Untagged Untagged')))
                                    scatter3((gaze_wait_position(ii,2)),(gaze_wait_position(ii,3)),(gaze_wait_position(ii,4)),'y');
                                    hold on;
                %check for crosswalk road pieces
                                    else if(~isempty(strfind(gaze_wait{ii,2},'Crosswalk_piece'))||~isempty(strfind(gaze_wait{ii,2},'Crosswalk')))
                                        scatter3((gaze_wait_position(ii,2)),(gaze_wait_position(ii,3)),(gaze_wait_position(ii,4)),'*','r');
                                        hold on;
                %check for task;the remaining data are
                %untagged buildings
                            else if(~isempty(strfind(gaze_wait{ii,2},'Task'))||~isempty(strfind(gaze_wait{ii,3},'Task'))||~isempty(strfind(gaze_wait{ii,2},'Ball'))||~isempty(strfind(gaze_wait{ii,3},'Ball')))
                                     scatter3((gaze_wait_position(ii,2)),(gaze_wait_position(ii,3)),(gaze_wait_position(ii,4)),'m');
                              hold on;
                                else
                                scatter3((gaze_wait_position(ii,2)),(gaze_wait_position(ii,3)),(gaze_wait_position(ii,4)),'c');
                                hold on;
                                end
                            end
                        end
                    end
                end
                    end
                end
end