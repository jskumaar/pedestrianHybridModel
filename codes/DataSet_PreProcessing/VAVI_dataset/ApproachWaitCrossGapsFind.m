%% This script calculates the gaps during which it was approach to wait/cross or wait to cross. The values have been added to
%% latest GapWiseCompiledData_V'x'.xlsx



clear all

FullData = xlsread('GapWiseCompiledDataV6.xlsx',1);

%waitThreshold = 0.2;

 %% approaching gaps
indDiff = diff(FullData(:,3));
indChange = find(indDiff~=0);
indStart = [1;indChange+1];
indEnd = [indChange;length(FullData)];


for ii=1:length(indStart)-1
    indCheck = indStart(ii):indEnd(ii);
    
    %approaching indices
    % when pedestrian is not crossing and when cumulative wait time is zero
    tempInd = find((FullData(indCheck,4)==0 | FullData(indCheck,4)==1) & FullData(indCheck,10)==0);
    FullData(indStart(ii)+tempInd-1,104)=1;  
    
    %waiting indices
    %when user is not crossing and when cumulative wait time is more than
    %zero
    tempInd = find((FullData(indCheck,4)==0 | FullData(indCheck,4)==1) & FullData(indCheck,10)>0);
    FullData(indStart(ii)+tempInd-1,104)=2;
    
    %crossing indices
    tempInd = find(FullData(indCheck,4)==2);
    FullData(indStart(ii)+tempInd-1,104)=3;
end
    
  

for ii=1:length(FullData)-1
    
    %approaching to wait or cross indices
    % when pedestrian is not crossing and when cumulative wait time is zero
    if (FullData(ii,11)==1 & FullData(ii+1,11)==2)
        FullData(ii,105)=1;
        FullData(ii+1,105)=2;
    elseif (FullData(ii,11)==1 & FullData(ii+1,11)==3)
        FullData(ii,105)=1;
        FullData(ii+1,105)=3;
    elseif (FullData(ii,11)==1)
        FullData(ii,105)=1;
    end
        
       
    
    %waiting to cross indices
    if (FullData(ii,11)==2 & FullData(ii+1,11)==3)
        FullData(ii,106)=1;
        FullData(ii+1,106)=2;
    elseif (FullData(ii,11)==2)
         FullData(ii,106)=1;
    end
end
    
    
    


