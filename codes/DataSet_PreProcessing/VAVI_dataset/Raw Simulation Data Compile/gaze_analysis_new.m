%This script reads the raw gaze data and compiles into an usable excel
%sheet


%Updated: 10/21/2019
%Code cleaning done a bit.

clear all;
close all;

%addpaths to the various files






%%to correct
%1) the wait end time depends on when users step onto road; add when they
%start to move along with that. when they 


%updating start time corrections-manually done while checking waiting times after
%checking the simulation video; correction not needed now as the final wait
%indices in waiting_time_function.m have been corrected by adding this correction
wait_time_correction = zeros(30,6);
% wait_time_correction(12,6) = 16;
% wait_time_correction(12,5) = 27;
% wait_time_correction(12,4) = 20;
% wait_time_correction(11,6) = 95.5;
% wait_time_correction(9,4) = 137;
% wait_time_correction(29,:) = [-5 -7 -8 -9 -7 -9];

%excel write variables
excel.wait.position = [];
excel.wait.gaze = [];
excel.wait.tags = [];
excel.wait.dist = [];
excel.wait.user = [];
excel.wait.ratio = [];
excel.wait.useful_data = [];
excel.compile_scenario = [];
excel.wait.scenario = [];

excel.cross.position = [];
excel.cross.gaze = [];
excel.cross.tags = [];
excel.cross.dist = [];
excel.cross.user = [];
excel.cross.ratio = [];
excel.cross.useful_data = [];
excel.cross.scenario = [];

excel.task.position = [];
excel.task.gaze = [];
excel.task.tags = [];
excel.task.dist = [];
excel.task.user = [];
excel.task.ratio = [];
excel.task.useful_data = [];
excel.task.scenario = [];

excel.waitratioscenario = [];
excel.waitusefulscenario = [];
excel.crossratioscenario = [];
excel.crossusefulscenario = [];
excel.taskratioscenario = [];
excel.taskusefulscenario = [];


%Reading files
for kk = 1:30 %for every subject
    for jj = 1:6 %for every scenario

        % load compiled simulation data
        filename = char(strcat('Subject_',num2str(kk),'_','Cond_',num2str(jj)));
        load(filename)
        
        if jj<4
            % read parameters for unsignalized
            del_t = simParam.del_t;
            wait_start = unsigData.waitData.waitStart_ind;
            wait_end = unsigData.waitData.waitEnd_ind;
            Ain_ind = unsigData.crossData.crossStart_ind(1:3);
            Bin_ind = unsigData.crossData.crossStart_ind(4:6);
            Aout_ind = unsigData.crossData.crossEnd_ind(1:3);
            Bout_ind = unsigData.crossData.crossEnd_ind(4:6);
            sim_end = unsigData.taskData.sim_end;
            vehicle_pos = unsigData.vehicleData.veh_pos;
            user_pos = unsigData.userData.pos;
            
            wait_start(1,1) =  wait_start(1,1) + wait_time_correction(kk,jj);
        else            
            % read parameters for signalized
            del_t = simParam.del_t;
            wait_start = sigData.waitData.waitStart_ind;
            wait_end = sigData.waitData.waitEnd_ind;
            Ain_ind = sigData.crossData.crossStart_ind(1:3);
            Bin_ind = sigData.crossData.crossStart_ind(4:6);
            Aout_ind = sigData.crossData.crossEnd_ind(1:3);
            Bout_ind = sigData.crossData.crossEnd_ind(4:6);
            sim_end = sigData.taskData.sim_end;
            vehicle_pos = sigData.vehicleData.veh_pos;
            user_pos = sigData.userData.pos;
            
            wait_start(1,1) =  wait_start(1,1) + wait_time_correction(kk,jj);
        end
    
        % load and read gaze data
        filepath = char(strcat('E:\User Study I Data\Data Analysis\Simulation Data\Subject',{' '},num2str(kk),'\',num2str(jj)));       
        files_list = dir(filepath);
        
        if jj<4
            gaze_file = files_list(end-3).name;
        else
            gaze_file = files_list(end-4).name;  
        end    
             
        gaze_data = fileread(gaze_file);
        patterns.gaze = 'Gaze points at time ([\-\d\.]+) are eye0 (([\-\d\.]+),\s([\-\d\.]+),\s([\-\d\.]+)\S\s([^1]+)\S\s(([\-\d\.]+),\s([\-\d\.]+),\s([\-\d\.]+)([^\n]+)';      
        gaze_str = regexp(gaze_data, patterns.gaze,'tokens');
        
        % convert to string; easier to manipulate
        gaze_obj = [];
        for ii=1:length(gaze_str)
            temp = gaze_str{1,ii};
            gaze_obj = [gaze_obj; temp];              
        end
        N = size(gaze_obj,1);
        
        %vehicle data simplification
        [vehicle] = vehicle_data_compile(vehicle_pos,user_pos,N,del_t);
   
%% splitting gaze data into three zones -  waiting, crossing and doing the task           
            
      %for each crossing 
        for ll = 1:6
            gaze_wait = gaze_wait_analysis(gaze_obj,vehicle,del_t,wait_start,wait_end,ll,jj,kk);
            gaze_cross = gaze_cross_analysis(gaze_obj,vehicle,del_t,Ain_ind,Aout_ind,Bin_ind,Bout_ind,ll,jj,kk);
            gaze_task = gaze_task_analysis(gaze_obj,vehicle,del_t,Ain_ind,Aout_ind,Bin_ind,Bout_ind,sim_end,ll,jj,kk);         
 
         %%Wait zone data export to file
            excel.wait.position = [excel.wait.position;gaze_wait.position];
            excel.wait.gaze = [excel.wait.gaze;gaze_wait.gaze];
            excel.wait.tags = [excel.wait.tags;gaze_wait.tags];
            excel.wait.dist = [excel.wait.dist;gaze_wait.dist'];
            excel.wait.user = [excel.wait.user;gaze_wait.user];
            excel.wait.scenario = [excel.wait.scenario;[kk*ones(size(gaze_wait.gaze,1),1),jj*ones(size(gaze_wait.gaze,1),1),ll*ones(size(gaze_wait.gaze,1),1)]];

            excel.wait.ratio = [excel.wait.ratio;gaze_wait.ratio];
            excel.wait.useful_data = [excel.wait.useful_data;gaze_wait.useful_data];
            
            %Cross zone data export to excel
            excel.cross.position = [excel.cross.position;gaze_cross.position];
            excel.cross.gaze = [excel.cross.gaze;gaze_cross.gaze];
            excel.cross.tags = [excel.cross.tags;gaze_cross.tags];
            excel.cross.dist = [excel.cross.dist;gaze_cross.dist'];
            excel.cross.user = [excel.cross.user;gaze_cross.user];
            excel.cross.scenario = [excel.cross.scenario;[kk*ones(size(gaze_cross.gaze,1),1),jj*ones(size(gaze_cross.gaze,1),1),ll*ones(size(gaze_cross.gaze,1),1)]];

            excel.cross.ratio = [excel.cross.ratio;gaze_cross.ratio];
            excel.cross.useful_data = [excel.cross.useful_data;gaze_cross.useful_data];
            
            %task data export to excel
            excel.task.position = [excel.task.position;gaze_task.position];
            excel.task.gaze = [excel.task.gaze;gaze_task.gaze];
            excel.task.tags = [excel.task.tags;gaze_task.tags];
            excel.task.dist = [excel.task.dist;gaze_task.dist'];
            excel.task.user = [excel.task.user;gaze_task.user];
            excel.task.scenario = [excel.task.scenario;[kk*ones(size(gaze_task.gaze,1),1),jj*ones(size(gaze_task.gaze,1),1),ll*ones(size(gaze_task.gaze,1),1)]];

            excel.task.ratio = [excel.task.ratio;gaze_task.ratio];
            excel.task.useful_data = [excel.task.useful_data;gaze_task.useful_data];
            
            excel.compile_scenario = [excel.compile_scenario;[kk,jj,ll]];
         end
    end     
end

%convert gaze ratio data from struct to matrix double to write to excel
excel.waitratio = [excel.wait.ratio.traffic;excel.wait.ratio.approach_vehicle;excel.wait.ratio.crossed_vehicle;excel.wait.ratio.signal;...
                excel.wait.ratio.signalpress;excel.wait.ratio.cw_build;excel.wait.ratio.cw_road;excel.wait.ratio.task;excel.wait.ratio.untagged;excel.wait.ratio.check_vehicle]';

excel.crossratio = [excel.cross.ratio.traffic;excel.cross.ratio.approach_vehicle;excel.cross.ratio.crossed_vehicle;excel.cross.ratio.signal;...
                excel.cross.ratio.signalpress;excel.cross.ratio.cw_build;excel.cross.ratio.cw_road;excel.cross.ratio.task;excel.cross.ratio.untagged;excel.cross.ratio.check_vehicle]';

excel.taskratio = [excel.task.ratio.traffic;excel.task.ratio.approach_vehicle;excel.task.ratio.crossed_vehicle;excel.task.ratio.signal;...
                excel.task.ratio.signalpress;excel.task.ratio.cw_build;excel.task.ratio.cw_road;excel.task.ratio.task;excel.task.ratio.untagged;excel.task.ratio.check_vehicle]';            
            
%scenario wise gaze ratio           
for ii=1:180
    excel.waitratioscenario = [excel.waitratioscenario;mean(excel.waitratio(6*(ii-1)+1:6*ii,:))];
    excel.waitusefulscenario = [excel.waitusefulscenario;mean(excel.wait.useful_data(6*(ii-1)+1:6*ii,:))];
    excel.crossratioscenario = [excel.crossratioscenario;mean(excel.crossratio(6*(ii-1)+1:6*ii,:))];
    excel.crossusefulscenario = [excel.crossusefulscenario;mean(excel.cross.useful_data(6*(ii-1)+1:6*ii,:))];
    excel.taskratioscenario = [excel.taskratioscenario;mean(excel.taskratio(6*(ii-1)+1:6*ii,:))];
    excel.taskusefulscenario = [excel.taskusefulscenario;mean(excel.task.useful_data(6*(ii-1)+1:6*ii,:))];
end

%write to excel
xlswrite('Gaze_data_compiled.xlsx',excel.wait.scenario,1,'A2');
xlswrite('Gaze_data_compiled.xlsx',excel.wait.position,1,'D2');
xlswrite('Gaze_data_compiled.xlsx',excel.wait.gaze(:,2:3),1,'H2');
xlswrite('Gaze_data_compiled.xlsx',excel.wait.tags,1,'J2');
xlswrite('Gaze_data_compiled.xlsx',excel.wait.dist,1,'K2');
xlswrite('Gaze_data_compiled.xlsx',excel.wait.user,1,'L2');
xlswrite('Gaze_data_compiled.xlsx',["Subject ID","Scenario ID","Crossing ID","Time","gaze_x","gaze_y","gaze_z","eye0 tag","eye1 tag","Object Tag","car-gaze distance","user_x","user_y"],1,'A1');


xlswrite('Gaze_data_compiled.xlsx',excel.cross.scenario,2,'A2');
xlswrite('Gaze_data_compiled.xlsx',excel.cross.position,2,'D2');
xlswrite('Gaze_data_compiled.xlsx',excel.cross.gaze(:,2:3),2,'H2');
xlswrite('Gaze_data_compiled.xlsx',excel.cross.tags,2,'J2');
xlswrite('Gaze_data_compiled.xlsx',excel.cross.dist,2,'K2');
xlswrite('Gaze_data_compiled.xlsx',excel.cross.user,2,'L2');
xlswrite('Gaze_data_compiled.xlsx',["Subject ID","Scenario ID","Crossing ID","Time","gaze_x","gaze_y","gaze_z","eye0 tag","eye1 tag","Object Tag","car-gaze distance","user_x","user_y"],2,'A1');

xlswrite('Gaze_data_compiled.xlsx',excel.task.scenario,3,'A2');
xlswrite('Gaze_data_compiled.xlsx',excel.task.position,3,'D2');
xlswrite('Gaze_data_compiled.xlsx',excel.task.gaze(:,2:3),3,'H2');
xlswrite('Gaze_data_compiled.xlsx',excel.task.tags,3,'J2');
xlswrite('Gaze_data_compiled.xlsx',excel.task.dist,3,'K2');
xlswrite('Gaze_data_compiled.xlsx',excel.task.user,3,'L2');
xlswrite('Gaze_data_compiled.xlsx',["Subject ID","Scenario ID","tasking ID","Time","gaze_x","gaze_y","gaze_z","eye0 tag","eye1 tag","Object Tag","car-gaze distance","user_x","user_y"],3,'A1');




%compiled ratios export to excel
xlswrite('Gaze_data_compiled.xlsx',excel.compile_scenario,4,'A2')
xlswrite('Gaze_data_compiled.xlsx',excel.wait.useful_data,4,'D2');
xlswrite('Gaze_data_compiled.xlsx',excel.waitratio,4,'E2');

xlswrite('Gaze_data_compiled.xlsx',excel.cross.useful_data,4,'O2');
xlswrite('Gaze_data_compiled.xlsx',excel.crossratio,4,'P2');

xlswrite('Gaze_data_compiled.xlsx',excel.task.useful_data,4,'AA2');
xlswrite('Gaze_data_compiled.xlsx',excel.taskratio,4,'AB2');

xlswrite('Gaze_data_compiled.xlsx',["Subject ID","Scenario ID","Crossing ID","Wait Useful data","Wait Ratio_traffic","Wait Ratio_approach_vehicle","Wait Ratio_crossed_vehicle","Wait Ratio_signal","Wait Ratio_signalpress","Wait Ratio_cw_build",...
        "Wait Ratio_cw_road","Wait Ratio_task","Wait Ratio_untagged","Wait Ratio_check_vehicle","Cross Useful data","Cross Ratio_traffic","Cross Ratio_approach_vehicle","Cross Ratio_crossed_vehicle","Cross Ratio_signal","Cross Ratio_signalpress",...
        "Cross Ratio_cw_build","Cross Ratio_cw_road","Cross Ratio_task","Cross Ratio_untagged","Cross Ratio_check_vehicle","Task Useful data","Task Ratio_traffic","Task Ratio_approach_vehicle","Task Ratio_crossed_vehicle","Task Ratio_signal",...
        "Task Ratio_signalpress","Task Ratio_cw_build","Task Ratio_cw_road","Task Ratio_task","Task Ratio_untagged","Task Ratio_check_vehicle"],4,'A1');



xlswrite('Gaze_data_compiled.xlsx',excel.waitusefulscenario,5,'D2');
xlswrite('Gaze_data_compiled.xlsx',excel.waitratioscenario,5,'E2');
xlswrite('Gaze_data_compiled.xlsx',excel.crossusefulscenario,5,'O2');
xlswrite('Gaze_data_compiled.xlsx',excel.crossratioscenario,5,'P2');
xlswrite('Gaze_data_compiled.xlsx',excel.taskusefulscenario,5,'AA2');
xlswrite('Gaze_data_compiled.xlsx',excel.taskratioscenario,5,'AB2');
xlswrite('Gaze_data_compiled.xlsx',["Subject ID","Scenario ID","Crossing ID","Wait Useful data","Wait Ratio_traffic","Wait Ratio_approach_vehicle","Wait Ratio_crossed_vehicle","Wait Ratio_signal","Wait Ratio_signalpress","Wait Ratio_cw_build",...
        "Wait Ratio_cw_road","Wait Ratio_task","Wait Ratio_untagged","Wait Ratio_check_vehicle","Cross Useful data","Cross Ratio_traffic","Cross Ratio_approach_vehicle","Cross Ratio_crossed_vehicle","Cross Ratio_signal","Cross Ratio_signalpress",...
        "Cross Ratio_cw_build","Cross Ratio_cw_road","Cross Ratio_task","Cross Ratio_untagged","Cross Ratio_check_vehicle","Task Useful data","Task Ratio_traffic","Task Ratio_approach_vehicle","Task Ratio_crossed_vehicle","Task Ratio_signal",...
        "Task Ratio_signalpress","Task Ratio_cw_build","Task Ratio_cw_road","Task Ratio_task","Task Ratio_untagged","Task Ratio_check_vehicle"],5,'A1');



