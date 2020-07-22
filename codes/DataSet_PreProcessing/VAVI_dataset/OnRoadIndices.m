%% 

%% Updated: 07/04/2019

%This file calculates the indices when the pedestrian stepped onto the road
%for each crossing


clear all
close all

% read data
[EventsData,~] = xlsread('G:\My Drive\Research\Pedestrian Modelling Project\Study I Data for Modeling\Compiled Data\DiscreteStateEventIndicesW5.xlsx');  


% Inputs
CrossThreshold = 3.2;       %z-position to be considered on road
VelocityThreshold = 0.2;    %minimum velocity considered as walking

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% event times (discrete states)
indices.ApproachStart = EventsData(:,4);
indices.ApproachEnd = EventsData(:,5);
indices.WaitStart = EventsData(:,6);
indices.WaitEnd = EventsData(:,7);
indices.CrossStart = EventsData(:,8);
indices.CrossEnd = EventsData(:,9);
indices.RetreatStart = EventsData(:,10);
indices.RetreatEnd = EventsData(:,11);

% indices to search for discrete states
indices.start = indices.CrossStart;
indices.end = indices.CrossEnd;
indices.NoWait = find(indices.WaitStart==0);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

EventIndices=[];
for kk=1:30             % subject ID
    for jj=1:3          % scenario ID; 1 - defensive unsignalized, 2 - normal unsignalized, 3 - aggressive unsignalized
        [PedestrianData,~] = xlsread('G:\My Drive\Research\Pedestrian Modelling Project\Study I Data for Modeling\Compiled Data\PedestrianDiscreteDataW5.xlsx',6*(kk-1)+jj);
      
        for ii=1:6      % for each crossing; there are six crossings in every scenario
            %% calculate new approach, cross, wait and retreat indices
            ind = 18*(kk-1)+6*(jj-1)+ii;    %crossing index
                       
            % indices to search for wait end
            indStart = indices.start(ind);  %approach start 
            indEnd = indices.end(ind);      %end of crossing 

            indicesOfInterest = find((abs(PedestrianData(indStart:indEnd,5))> CrossThreshold & PedestrianData(indStart:indEnd,8)>VelocityThreshold ));    %when they just almost crossed the road and walking (not stopped)            
            tempInd = find(diff(indicesOfInterest)>1,1,'last');
            
            if isempty((intersect(ind,indices.NoWait)))    %i.e. when the users wait, there will be a break (when waiting) in between; else there wont be a break.                       
                OnRoad = indicesOfInterest(tempInd)+double(indStart-1)+1;       %added 1 as that is the time instance when they crossed the road.
            else
                OnRoad = indices.CrossStart;
            end

            EventIndices = [EventIndices;OnRoad];
        end
    end 
end

x=1;