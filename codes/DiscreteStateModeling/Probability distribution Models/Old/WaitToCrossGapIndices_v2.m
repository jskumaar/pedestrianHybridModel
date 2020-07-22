function  indices = WaitToCrossGapIndices_v2(GapData,out)

for mm=1:length(GapData)
   crossingIndex(mm) = 18*(GapData(mm,1)-1) + 6*(GapData(mm,2)-1) + GapData(mm,3);    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Wait to Cross: find vehicle gaps indices of interest

% crossing and approach indices; do not count for gap rejection
indices.Gap_Approach = find(GapData(:,2)==1);
indices.Gap_Cross = find(GapData(:,2)==3);

indices.FirstWait = zeros(540,1);
for mm=1:540
   GapIndicesForEachCrossing{mm,1} = find(crossingIndex==mm);
   temp = find(GapData(GapIndicesForEachCrossing{mm},2)==2,1,'first');
   if ~isempty(temp)
        indices.FirstWait(mm) = GapIndicesForEachCrossing{mm}(1)+temp-1;
   end
end
indices.FirstWait(indices.FirstWait==0) = [];


%unique vehicle gap indices for rejected gaps
indices.Gap_uniqueRemoveRejectedGap = unique([indices.Gap_Approach;indices.Gap_Cross]);


%% Rejected Gap indices
indices.Gap_RejectedGapIndices = [1:length(GapData)]';
indices.Gap_RejectedGapIndices(indices.Gap_uniqueRemoveRejectedGap) = [];


%% Accepted gaps while waiting
indices.Gap_AcceptedGapWhileWaitIndices = find(GapData(:,2)==3 & (GapData(:,1)~=0));
indices.Gap_AcceptedGapsWhileApproachIndices = find(GapData(:,2)==3 & (GapData(:,1)==0));

 
% compile indices
indices.WCRejectedGaps = indices.Gap_RejectedGapIndices;
indices.WCAcceptedGaps = [indices.Gap_AcceptedGapWhileWaitIndices];
indices.WCAllGaps = [indices.WCAcceptedGaps;indices.WCRejectedGaps];
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% to include the approach to cross/wait gaps when wait starts

if out==1
    indices.WCRejectedGaps = indices.Gap_RejectedGapIndices;
    indices.WCAcceptedGaps = [indices.Gap_AcceptedGapWhileWaitIndices;indices.Gap_AcceptedGapsWhileApproachIndices];
else
    indices.WCRejectedGaps = indices.Gap_RejectedGapIndices;
    indices.WCAcceptedGaps = indices.Gap_AcceptedGapWhileWaitIndices;
end
    


indices.WCAllGaps = [indices.WCAcceptedGaps;indices.WCRejectedGaps];


end