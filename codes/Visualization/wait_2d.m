 
 
figure()
x = [1:333 334:676 677:1090 1091:1205 1206:1373 1374:1488];
for ii=1374:1488
                %check for Traffic light
                
                if(~isempty(strfind(gaze_wait{ii,2},'Traffic'))||~isempty(strfind(gaze_wait{ii,3},'Traffic')))
                    scatter((gaze_wait_position(ii,2)),(gaze_wait_position(ii,3)),'r');
                    hold on;
                
                     
                %check for Ped light
                else if(~isempty(strfind(gaze_wait{ii,2},'Ped'))||~isempty(strfind(gaze_wait{ii,3},'Ped')))
                     scatter((gaze_wait_position(ii,2)),(gaze_wait_position(ii,3)),'b');
                      hold on;               
%                     
                %check for vehicles
                    else if(~isempty(strfind(gaze_wait{ii,2},'Vehicle'))||~isempty(strfind(gaze_wait{ii,3},'Vehicle')))
                     scatter((gaze_wait_position(ii,2)),(gaze_wait_position(ii,3)),'k');
                 hold on;
%                     
                 %check for crosswalk buildings when user is on near side
                            else if(~isempty(strfind(gaze_wait{ii,2},'Crosswalk Building'))||~isempty(strfind(gaze_wait{ii,3},'Crosswalk Building')))
                                scatter((gaze_wait_position(ii,2)),(gaze_wait_position(ii,3)),'g');
                                hold on;
                %check for crosswalk buildings when user is on far side
                                else if(~isempty(strfind(gaze_wait{ii,2},'Untagged Untagged eye'))||~isempty(strfind(gaze_wait{ii,3},'Untagged Untagged')))
                                    scatter((gaze_wait_position(ii,2)),(gaze_wait_position(ii,3)),'y');
                                    hold on;
                %check for crosswalk road pieces
                                    else if(~isempty(strfind(gaze_wait{ii,2},'Crosswalk_piece'))||~isempty(strfind(gaze_wait{ii,2},'Crosswalk')))
                                        scatter((gaze_wait_position(ii,2)),(gaze_wait_position(ii,3)),'*','r');
                                        hold on;
                %check for task;the remaining data are
                %untagged buildings
                            else if(~isempty(strfind(gaze_wait{ii,2},'Task'))||~isempty(strfind(gaze_wait{ii,3},'Task'))||~isempty(strfind(gaze_wait{ii,2},'Ball'))||~isempty(strfind(gaze_wait{ii,3},'Ball')))
                                     scatter((gaze_wait_position(ii,2)),(gaze_wait_position(ii,3)),'m');
                              hold on;
                                else
                                scatter((gaze_wait_position(ii,2)),(gaze_wait_position(ii,3)),'c');
                                hold on;
                                end
                            end
                        end
                    end
                end
                    end
                end
end