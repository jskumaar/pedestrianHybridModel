clear all;
close all;

%Reading files
del_t = 0.01;

writeToExcel.rawData = [];
writeToExcel.crossing = [];
writeToExcel.scenario = [];

for kk = [1 3:25 27:29]  %eye tracker data not available for subjects 2, 26 and 30
% for kk = 1
    for jj = 1:6    %for every scenario
        gazeFixation.scenario = [];
        gazeFixation.time.scenario = zeros(1,10);
        for ll=1:6  %for every crossing
            gazeFixation.crossing = [];
%             gazeFixation.matstring ="";
            gazeFixation.mat = [];
            %load wait gaze data - manually check if there are no duplicate data files in
            %any of the directory
            filename = strcat('GazeData_Wait_Subject_',num2str(kk),'_',num2str(jj),'_',num2str(ll),'.mat');
            load(filename);  
            gaze_wait.position = double(round(gaze_wait.position,1)/del_t)*del_t;
            
            %% Working with gaze fixation data from eye tracker
            %%Read fixation file
            filepath = char(strcat('E:\User Study I Data\Data Analysis\Simulation Data\Pupil Data\Subject',...
                                {' '},num2str(kk),'\',num2str(kk),'_',num2str(jj),'\','fixations.csv'));
            gazeFixation.data = xlsread(filepath);
            writeToExcel.rawData  = [writeToExcel.rawData;gazeFixation.data];
           
            %cleaning and ordering the fixation data
            [~,idx] = sort(gazeFixation.data(:,3));     %sort by time
            gazeFixation.mat = gazeFixation.data(idx,:);        %fixation data gets re-written for every crossing
            
            %removing right eye fixation data - to maintain consistency
            %with gaze data which used only left eye data
            gazeFixation.mat = gazeFixation.mat(gazeFixation.mat(:,2) == 0,:);   
            gazeFixation.mat = [ones(size(gazeFixation.mat,1),3).*[kk,jj,ll],gazeFixation.mat];
 
            %finding starting and ending indices of fixations within the waiting zone of this crossing            
            gazeFixation.startInd = find(round(gazeFixation.mat(:,6),1)>(wait_start(ll)-1)*del_t,1,'first');          
            gazeFixation.endInd  = find(round(gazeFixation.mat(:,6),1)<(wait_end(ll)-1)*del_t,1,'last');        %the last fixation is also included even though the end of the fixation might be outside the wait zone
            
            writeToExcel.variable = zeros(1,5);
            writeToExcel.variable(1:3) = [kk,jj,ll];
            if (~isempty(gazeFixation.startInd)&&~isempty(gazeFixation.endInd))
                if (gazeFixation.startInd<=gazeFixation.endInd)
                    writeToExcel.variable(4:5) = [mean(gazeFixation.mat(gazeFixation.startInd:gazeFixation.endInd,13)),gazeFixation.endInd-gazeFixation.startInd+1];
                end
            end
            
            %crossing wise data to be written
            writeToExcel.crossing = [writeToExcel.crossing;writeToExcel.variable];
        end
        %scenario wise data to be written
        idx=1;      %to adjust for missing subject data
        if kk>1&&kk<26
            idx=kk-1;
        elseif kk>26
                idx=kk-2;
        end
        
        scenario_pupil_dia = sum(writeToExcel.crossing(36*(idx-1) + 6*(jj-1) + 1:36*(idx-1) + 6*(jj),4).*writeToExcel.crossing(36*(idx-1) + 6*(jj-1) + 1:36*(idx-1) + 6*(jj),5),1)/sum(writeToExcel.crossing(36*(idx-1) + 6*(jj-1) + 1:36*(idx-1) + 6*(jj),5),1);
        scenario_no_fixations = sum(writeToExcel.crossing(36*(idx-1) + 6*(jj-1) + 1:36*(idx-1) + 6*(jj),5));               
        writeToExcel.scenario = [writeToExcel.scenario;kk,jj,scenario_pupil_dia,scenario_no_fixations];
    end
end


%%Write to excel file
xlswrite('Fixation_pupil_dia.xlsx',writeToExcel.rawData,1,'A2'); 

xlswrite('Fixation_pupil_dia.xlsx',gazeFixation.mat,2,'A2');   
xlswrite('Fixation_pupil_dia.xlsx',["Index","Eye ID","Fixation Start Time","Fixation duration","Start Frame","End Frame","Norm_pos_x","Norm_pos_y","Dispersion","Average Pupil diameter","Confidence"],2,'A1');

xlswrite('Fixation_pupil_dia.xlsx',writeToExcel.crossing,3,'A2');
xlswrite('Fixation_pupil_dia.xlsx',["Subject ID","Scenario ID","Crossing ID","Average Pupil diameter","No.of fixations"],3,'A1');

xlswrite('Fixation_pupil_dia.xlsx',writeToExcel.scenario,4,'A2');
xlswrite('Fixation_pupil_dia.xlsx',["Subject ID","Scenario ID","Average Pupil diameter","No.of fixations"],4,'A1');