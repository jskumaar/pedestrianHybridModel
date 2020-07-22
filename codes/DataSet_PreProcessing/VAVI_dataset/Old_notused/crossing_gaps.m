%% crossing gaps

%10/21/19: this script just calculates which gaps 

for kk=1:30
    for jj=1:3
        
        %Read events data
        [EventsData,~] = xlsread('EventsDatawithApproach.xlsx');
        times.ApproachStart = EventsData(:,3);
        times.WaitStart = EventsData(:,7);
        times.CrossStart = EventsData(:,9);
        
        %Read nearest vehicle DTC, position, next vehicle position,
        %adjacent vehicle position data
        [vehicleData,~] = xlsread('VehicleGapTimesV6.xlsx',6*(kk-1)+jj);
        
        CrossingGap = NaN*ones(size(vehicleData,1),1);

        for ii=1:6 %each crossing
            %%Find indices of start and end gaps
            %works only if sampling time is not changed (i.e. 0.1 s)
            indices.ApproachStart = times.ApproachStart(18*(kk-1)+6*(jj-1)+ii)*10 + 1;
            indices.WaitEnd = times.WaitEnd(18*(kk-1)+6*(jj-1)+ii)*10 + 1;
            
            %remember the indices obtained here are for a subset of the overall array
            temp.vehicleDTCdiff = diff(vehicleData(indices.ApproachStart:indices.WaitEnd,5));            
            indices.VehicleCrossPedestrian = find(temp.vehicleDTCdiff>0);

            indices.StartGap{ii} = indices.ApproachStart + indices.VehicleCrossPedestrian + 1;      %add the starting index to get the correct indices for the overall set
            indices.StartGap{ii} = [indices.ApproachStart;indices.StartGap{ii}];
            indices.EndGap{ii} = indices.StartGap{ii}(2:end)-1;
            indices.EndGap{ii} = [indices.EndGap{ii};indices.WaitEnd];
            
            %add crossing, non-crossing
            for mm=1:size(indices.EndGap{ii},1)
                startInd = vehicleData(indices.StartGap{ii}(mm));
                endInd = indices.EndGap{ii}(mm);
                
                if mm~=size(indices.EndGap{ii},1)
                    CrossingGap(startInd:endInd) = 0;
                else
                    CrossingGap(startInd:endInd) = 1;
                end 
                
            end
            

        end
  
    end
end
