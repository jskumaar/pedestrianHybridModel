%%Visualization & Sorting


gaze_wait_viz = [gaze_wait_cwnear_build_time ones(length(gaze_wait_cwnear_build_time),1)];
gaze_wait_viz = [gaze_wait_viz; gaze_wait_signal_time 2*ones(length(gaze_wait_signal_time),1)];
gaze_wait_viz = [gaze_wait_viz; gaze_wait_task_time 3*ones(length(gaze_wait_task_time),1)];
gaze_wait_viz = [gaze_wait_viz; gaze_wait_traffic_time 4*ones(length(gaze_wait_traffic_time),1)];
gaze_wait_viz = [gaze_wait_viz; gaze_wait_vehicle_time 5*ones(length(gaze_wait_vehicle_time),1)];
gaze_wait_viz = [gaze_wait_viz; gaze_wait_untagged_time 6*ones(length(gaze_wait_untagged_time),1)];
gaze_wait_viz = [gaze_wait_viz; gaze_wait_cw_road_time 7*ones(length(gaze_wait_cw_road_time),1)];

[~,idx] = sort(gaze_wait_viz(:,1));
gaze_wait_viz_sort = gaze_wait_viz(idx,:);



% %% 
% dim = [.2 .5 .3 .3];
% for i=1:length(gaze_wait_viz_sort)
%     if gaze_wait_viz_sort(i,2)==1
%         figure(1)
%         
%         h = rectangle('Position',[3 0 2 4],'FaceColor',[0 .5 .5]);
%         axis([0 15 0 10])
%         k = annotation('textbox',dim,'String','Crosswalk Building','FitBoxToText','on');
%         pause(0.1);
%         delete(h);
%         delete(k);
%         
%         
%     else if gaze_wait_viz_sort(i,2)==2
%         figure(1)
%         
%         h=rectangle('Position',[1 4 1 1],'FaceColor',[.5 .5 .5]);
%         axis([0 15 0 10])
%         k=annotation('textbox',dim,'String','Signal','FitBoxToText','on');
%          pause(0.1);
%                  delete(h);
%         delete(k);
%         
%         
%     else if gaze_wait_viz_sort(i,2)==3
%             figure(1)
%         
%        h = rectangle('Position',[8 4 2 2],'FaceColor',[.5 0 .5]);
%        axis([0 15 0 10])
%         k=annotation('textbox',dim,'String','Task','FitBoxToText','on');
%          pause(0.1);
%                  delete(h);
%         delete(k);
%         
%         
%         else if gaze_wait_viz_sort(i,2)==4
%                 figure(1)
%         
%         h=rectangle('Position',[0 6 1 1],'FaceColor',[.5 .5 0]);
%         axis([0 15 0 10])
%         k=annotation('textbox',dim,'String','Traffic','FitBoxToText','on');
%          pause(0.1);
%                  delete(h);
%         delete(k);
%         
%         
%        else if gaze_wait_viz_sort(i,2)==5
%                figure(1)
%         
%         h=rectangle('Position',[6 0 2 2],'FaceColor',[1 0 0]);
%         axis([0 15 0 10])
%         k=annotation('textbox',dim,'String','Vehicle','FitBoxToText','on');
%          pause(0.1);
%                  delete(h);
%         delete(k);
%         
%         
%            else
%                figure(1)
%         
%         h=rectangle('Position',[5 5 3 3],'FaceColor',[.5 .5 .5]);
%         axis([0 15 0 10])
%         k =annotation('textbox',dim,'String','Untagged','FitBoxToText','on');
%          pause(0.1);
%                  delete(h);
%         delete(k);
%         
%            end
%             end
%         end
%         end
%     end
% end

%% plot
figure(1)
descr = {'1:Crosswalk Buildings';
    '2:Pedestrian Signal';
    '3:Task Objects';
    '4:Traffic Signal';
    '5:Vehicle';
    '6:Others'};

scatter(gaze_wait_viz_sort(:,1),gaze_wait_viz_sort(:,2))
title('unfiltered')

hold on;
for i =1:length(gaze_wait_viz_sort)-1
    if(gaze_wait_viz_sort(i,2)~=gaze_wait_viz_sort(i+1,2))
        plot(gaze_wait_viz_sort(i:i+1,1),gaze_wait_viz_sort(i:i+1,2))
        hold on;
    end
end
ax1 = axes('Position',[0.8 0.7 0.3 0.3],'Visible','off');
axes(ax1) % sets ax1 to current axes
text(.025,0.6,descr)


%filtering
gaze_filtered = gaze_filter(gaze_wait_viz_sort,0);



figure(2)
scatter(gaze_filtered(:,1),gaze_filtered(:,2))
title('filtered')
hold on;
for i =1:length(gaze_filtered)-1
    if(gaze_filtered(i,2)~=gaze_filtered(i+1,2))
        plot(gaze_filtered(i:i+1,1),gaze_filtered(i:i+1,2))
        hold on;
    end
end
ax1 = axes('Position',[0.8 0.7 0.3 0.3],'Visible','off');
axes(ax1) % sets ax1 to current axes
text(.025,0.6,descr)