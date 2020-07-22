%% Gaps from DTC measures

%This file calculates gap based on DTC measures; The current DTC measures
%are calculated from raw position data of vehicle and pedestrian. The
%assumption is that filtering would not drastically influence the indices
%of the gaps.

% The earlier version of gap calculation used the events of the vehicle
% crossing the crosswalks to indicate gap start or gap end whereas in this
% version, the events of vehicle crossing the pedestrian is used (and thus
% DTC).


clear all
close all

VehicleGapData = [];
%GapTolerance = 10;

for kk=1:30     % for every subject
    for jj=1:3  % for every scenario
        
        %% Read excel data
        [VehicleData,~] = xlsread('VehicleDTCSpeedDataV2.xlsx',6*(kk-1)+jj);    %latest compiled closest vehicle data
        [PedData,~] = xlsread('PedestrianDiscreteDataW11_onlyforMLmodel.xlsx',6*(kk-1)+jj);     %filtered pedestrian data (Mwa, WS=5)
        [EventsData,~] = xlsread('DiscreteStateEventIndicesW5.xlsx');           %discrete event data from the filtered pedestrian data
       
        %% Vehicle gap identification from the sign change in DTC difference
        temp = diff(VehicleData(:,5));
        gapChange = find(temp>2);       % gaps greater than 2m difference
        
        GapStart = [1;gapChange(1:end)+1];
        GapEnd = [gapChange;length(VehicleData)];  
        GapDuration = GapEnd-GapStart;
        for ii=1:6
            indStartCross(ii) = find(GapStart<EventsData(18*(kk-1)+6*(jj-1)+ii,4),1,'last');
            indEndCross(ii) = find(GapEnd>EventsData(18*(kk-1)+6*(jj-1)+ii,11),1,'first');          
        end
        
        
        
        
        % for every crossing
        for ii=1:6
            GapIndexVariable = size(VehicleGapData,1);
%             indStart = find(GapStart<EventsData(18*(kk-1)+6*(jj-1)+ii,5),1,'last'); % last gap that starts before approach ends
            %indStart = find(GapStart>EventsData(18*(kk-1)+6*(jj-1)+ii,4),1,'first');  % first gap that starts after approach starts
            indStart = indStartCross(ii);
            
            %indEnd = find(GapEnd>EventsData(18*(kk-1)+6*(jj-1)+ii,8),1,'first');    % first gap that ends after crossing starts
            %indEnd = find(GapEnd>EventsData(18*(kk-1)+6*(jj-1)+ii,12),1,'first');    % first gap that ends after pedestrian is on the road
            %indEnd = find(GapEnd>EventsData(18*(kk-1)+6*(jj-1)+ii,11),1,'first');    % first gap that ends after pedestrian ends retreat
             indEnd = indEndCross(ii);
            
            %To calculate waiting decision classification, need the gap
            %when approaching
            if indStart~=1
                indStart = indStart-1;
            end
            
%             % if there is only one gap in the crossing, include the
%             % previous gap also - now not needed as including approach
%             gap for all crossings (check previous block of code)
%             if (indStart==indEnd)
%                 indStart = indStart-1;
%             end
            
            % compile the Gap data
            VehicleGapData = [VehicleGapData;[repmat([kk,jj,ii,0],[indEnd-indStart+1,1]),GapStart(indStart:indEnd),...
                              GapEnd(indStart:indEnd),GapStart(indStart:indEnd),GapEnd(indStart:indEnd),GapEnd(indStart:indEnd),zeros(indEnd-indStart+1,1)]];
            
                          
            % index when crossing starts
            CrossStartInd = EventsData(18*(kk-1)+6*(jj-1)+ii,8);               
                          
                          
            % find closest starting gap index before wait time starts
            ind = find(GapStart(indStart:indEnd)<=EventsData(18*(kk-1)+6*(jj-1)+ii,6),1,'last');
            if isempty(ind)  % when there is no waiting, the wait start will have '0'
                VehicleGapData(GapIndexVariable+1,7) = EventsData(18*(kk-1)+6*(jj-1)+ii,6); %replace first index with wait Start index
                VehicleGapData(GapIndexVariable+indEnd-indStart+1,8) = EventsData(18*(kk-1)+6*(jj-1)+ii,8);  %crossing gap end (i.e. start of cross)           
            else          
                gapIndex = [GapIndexVariable+ind:GapIndexVariable+indEnd-indStart+1];   % gap indices to change
                
                VehicleGapData(GapIndexVariable+ind,7) = EventsData(18*(kk-1)+6*(jj-1)+ii,6); %replace first index with wait Start index
                 
                %crossing gap end (on road) and crossing start by walking
                %are in same last vehicle gap (i.e. crossing gap)
                if (CrossStartInd>=GapStart(indEnd) && CrossStartInd<=GapEnd(indEnd))
                    VehicleGapData(GapIndexVariable+indEnd-indStart+1,8) = EventsData(18*(kk-1)+6*(jj-1)+ii,8);
                    VehicleGapData(GapIndexVariable+indEnd-indStart+1,9) = EventsData(18*(kk-1)+6*(jj-1)+ii,12);  %crossing gap end (i.e. start of cross)
                else
                    VehicleGapData(GapIndexVariable+indEnd-indStart,8) = EventsData(18*(kk-1)+6*(jj-1)+ii,8);
                    VehicleGapData(GapIndexVariable+indEnd-indStart+1,9) = EventsData(18*(kk-1)+6*(jj-1)+ii,12);  %crossing gap end (i.e. start of cross)
                end
                
                %cumulative waiting times
                VehicleGapData(gapIndex,10) = cumsum([VehicleGapData(gapIndex,8)-VehicleGapData(gapIndex,7)]/10);     
 
                %else
%                 gapIndex = [GapIndexVariable+1:GapIndexVariable+indEnd-indStart+1];
%                 
%                 VehicleGapData(GapIndexVariable+ind,7) = EventsData(18*(kk-1)+6*(jj-1)+ii,6); %replace first index with wait Start index
%                 %additionally make the Gap Start (wait start) indiices '0'
%                 VehicleGapData(GapIndexVariable+1:GapIndexVariable+ind-1,7)=0;
%                
%                 VehicleGapData(GapIndexVariable+indEnd-indStart+1,8) = EventsData(18*(kk-1)+6*(jj-1)+ii,8);  %crossing gap end (i.e. start of cross)
%                 
%                 VehicleGapData(gapIndex,9) = cumsum([VehicleGapData(gapIndex,8)-VehicleGapData(gapIndex,7)]/10);      % cumulative waiting times for
%  

              
            end
                          
            

           
            % if crossing is in the last gap
            if (CrossStartInd>=GapStart(indEnd) && CrossStartInd<=GapEnd(indEnd))
                % crossing decision
                VehicleGapData(GapIndexVariable+indEnd-indStart+1,4) = 2;   %Both on road and started walking
            else
                % crossing decision
                VehicleGapData(GapIndexVariable+indEnd-indStart+1,4) = 2;   %on Road
                VehicleGapData(GapIndexVariable+indEnd-indStart,4) = 1;     %started walking                
            end
            

           x=1;  
        end

    end
end

%the cumulative wait times are messed up when on road gap and cross start
%gaps are different; the below code corrects that
ind = find(VehicleGapData(:,4)==1);
VehicleGapData(ind+1,10) = VehicleGapData(ind,10);

%
% VehicleGapData(:,11:12) = VehicleGapData(:,[8,10]);
% VehicleGapData = VehicleGapData(:,[1:9,11,10,12]);


% ind = find(VehicleGapData(:,7)>VehicleGapData(:,8));
% VehicleGapData(:,12) = VehicleGapData(:,8);
% if ~isempty(ind)
%     VehicleGapData(ind,12) = VehicleGapData(ind,7)+0.1;
% end

% for the gaps when waiting starts (or no waiting)
ind = find(VehicleGapData(:,7)>VehicleGapData(:,5));
VehicleGapData(ind,11)=2;   % waiting index
VehicleGapData(ind-1,11)=1; % approaching index

ind = find(VehicleGapData(:,10)==0 & VehicleGapData(:,4)==2);
VehicleGapData(ind,11)=3;   % crossing without waiting index
VehicleGapData(ind-1,11)=1; % approaching index

% for the gaps when crossing starts with waiting, i.e cumulative waiting
% time not zero
ind = find(VehicleGapData(:,7)>VehicleGapData(:,5) | VehicleGapData(:,10)==0 & VehicleGapData(:,4)==2);
ind2 = find(VehicleGapData(:,4)==2 & VehicleGapData(:,10)~=0);

for ii=1:length(ind2)
    ind3 = find(ind<ind2(ii),1,'last');
    ind3 = ind(ind3);
    VehicleGapData(ind3:ind2(ii)-1,12)=1;
    VehicleGapData(ind2(ii),12)=2;
end


excelHeader = {'SubjectID','ScenarioID','Crossing ID','Crossing Decision','Gap Start Index',...
               'Gap End Index','Gap Start (Wait Start)','Gap End (Cross Start)','On Road','Cumulative Wait time',...
               'Cross from wait Gap Decision','Approach to Wait/Cross Gap Decision'};
      
xlswrite('VehicleGapTimesV6.xlsx',VehicleGapData,1,'A2');   
xlswrite('VehicleGapTimesV6.xlsx',excelHeader,1,'A1');

