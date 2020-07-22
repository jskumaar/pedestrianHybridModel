% This file converts all the data in the excel sheets to mat files, which
% is more convenient for processsing (less read time)

%% Updated: 07-04-2019: Added the folder paths to the files

EventIndices = xlsread('G:\My Drive\Research\Pedestrian Modelling Project\Study I Data for Modeling\Compiled Data\DiscreteStateEventIndicesW5.xlsx');

ApproachStart =  EventIndices(:,4);
RetreatEnd = EventIndices(:,11);

for kk=1:30
    for jj=1:3
        ind = 6*(kk-1)+jj;      %sheet number in excel file        
        [VehiclePosRawData,~] = xlsread('G:\My Drive\Research\Pedestrian Modelling Project\Study I Data for Modeling\Raw Data\VehicleDataResampled.xlsx',ind);      
        [PedRawData,~] = xlsread('G:\My Drive\Research\Pedestrian Modelling Project\Study I Data for Modeling\Raw Data\PedestrianDataRaw.xlsx',ind);
        [GazeData,~] = xlsread('G:\My Drive\Research\Pedestrian Modelling Project\Study I Data for Modeling\Raw Data\GazeNoDuplicate.xlsx',ind);
        
        % Data lengths
        N = length(PedRawData);
        LaneChangeInd = find(diff(VehiclePosRawData(:,4))<0,1,'first');
        N_veh = length(VehiclePosRawData)-1;
        N_LaneA = LaneChangeInd-1;
        N_LaneB = N_veh-N_LaneA;
        
        % split vehicle data based on lane
        VehPosLaneA = zeros(N,size(VehiclePosRawData,2)-4);     %removing 4 columns for Subject, Scenario, Crossing IDs and time.
        VehPosLaneB = zeros(N,size(VehiclePosRawData,2)-4);
               
        % The no.of data points of pedestrian, vehicle in lane A and
        % vehicle in lane B don't always match
        
        if N==N_LaneA
            VehPosLaneA = VehiclePosRawData(2:N_LaneA+1,5:end);
        elseif N<N_LaneA
            VehPosLaneA = VehiclePosRawData(2:N+1,5:end);
        elseif N>N_LaneA
            VehPosLaneA(1:N_LaneA,:) = VehiclePosRawData(2:N_LaneA+1,5:end);
        end
        
        if N==N_LaneB
            VehPosLaneB = VehiclePosRawData(N_LaneA+2:end,5:end);
        elseif N<N_LaneB
            VehPosLaneB = VehiclePosRawData(N_LaneA+2:N_LaneA+2+N-1,5:end);
        elseif N>N_LaneB
            VehPosLaneB(1:N_LaneB,:) = VehiclePosRawData(N_LaneA+2:N_LaneA+2+N_LaneB-1,5:end);
        end
        
        RawData = table(PedRawData,GazeData,VehPosLaneA,VehPosLaneB);        
        VRStudy_I_ScenarioData{ind,1} = RawData;
        
        
        % crossing wise data         
        S = vartype('numeric');
        % compile data for every crossing        
        for ii=1:6           
           indCrossing = 18*(kk-1)+6*(jj-1)+ii;
           indStart = ApproachStart(indCrossing);
           indEnd = RetreatEnd(indCrossing);
           
           VRStudy_I_CrossingData{indCrossing,1} = RawData(indStart:indEnd,S);        
        end
         
    end 
end



save('VRStudy_I_ScenarioData.mat','VRStudy_I_ScenarioData');        %data split scenario wise
save('VRStudy_I_CrossingData.mat','VRStudy_I_CrossingData');        %same data split crossing wise (easier for trajectory modelling; each crossing is a trajectory)



