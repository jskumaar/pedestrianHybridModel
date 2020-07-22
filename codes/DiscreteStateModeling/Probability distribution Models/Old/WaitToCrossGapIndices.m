function  indices = WaitToCrossGapIndices(GapData,EventIndices,waitThreshold,out)

GapData = 0;        % so that this function is not being used anywhere else

for mm=1:length(GapData)
   crossingIndex(mm) = 18*(GapData(mm,1)-1) + 6*(GapData(mm,2)-1) + GapData(mm,3);    
end
indices.FirstWait = zeros(540,1);
for mm=1:540
   GapIndicesForEachCrossing{mm,1} = find(crossingIndex==mm);
   temp = find(GapData(GapIndicesForEachCrossing{mm},11)==2,1,'first');
   if ~isempty(temp)
        indices.FirstWait(mm) = GapIndicesForEachCrossing{mm}(1)+temp-1;
   end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Wait to Cross: find vehicle gaps indices of interest

% crossing and approach indices; do not count for gap rejection
indices.Gap_Approach = find(GapData(:,11)==1);
indices.Gap_Cross = find(GapData(:,11)==3);
indices.FirstWait(indices.FirstWait==0) = [];

%use gap acceptance data for only crossings with nominal waiting times;
%remove the outliers data! These indices are crossing indices
indices.Cross_WaitActivityOutliers = find(GapData(indices.Gap_Cross,10)>waitThreshold);


% vehicle gaps indices corresponding to the waiting outliers
indices.Gap_ToRemoveWaitTimeOutliers =[];
for ii=1:length(indices.Cross_WaitActivityOutliers)
     indices.Gap_ToRemoveWaitTimeOutliers = [indices.Gap_ToRemoveWaitTimeOutliers;GapIndicesForEachCrossing{indices.Cross_WaitActivityOutliers(ii)}'];
end


% % vehicle gaps indices corresponding to the waiting outliers
% indices.Gap_ToRemoveWaitTimeOutliers =[];
% for ii=1:length(indices.Cross_WaitActivityOutliers)
%      indices.Gap_ToRemoveWaitTimeOutliers = [indices.Gap_ToRemoveWaitTimeOutliers;[indStart(indices.Cross_WaitActivityOutliers(ii)):indEnd(indices.Cross_WaitActivityOutliers(ii))]'];
% end

%unique vehicle gap indices for rejected gaps
indices.Gap_uniqueRemoveRejectedGap = unique([indices.Gap_Approach;indices.Gap_ToRemoveWaitTimeOutliers;indices.Gap_Cross]);
% indices.Gap_uniqueRemoveRejectedGap = unique([indices.Gap_Approach;indices.Gap_Cross]);

%% Rejected Gap indices
indices.Gap_RejectedGapIndices = [1:length(GapData)]';
indices.Gap_RejectedGapIndices(indices.Gap_uniqueRemoveRejectedGap) = [];

%crossing indices within wait time threshold
indices.Cross_WaitActivityNotOutliers = find(GapData(indices.Gap_Cross,10)<=waitThreshold);

%% Accepted Gaps without outliers
indices.Gap_AcceptedGapIndices = indices.Gap_Cross(indices.Cross_WaitActivityNotOutliers);
indices.Gap_AcceptedGapIndices(isempty(GapIndicesForEachCrossing))=[];

%% Accepted gaps while waiting
indices.Gap_AcceptedGapWhileWaitIndices = find(GapData(:,11)==3 & (GapData(:,10)~=0));
indices.Gap_AcceptedGapsWhileApproachIndices = find(GapData(:,11)==3 & (GapData(:,10)==0));

[~,ind,~] = intersect(indices.Gap_AcceptedGapWhileWaitIndices,indices.Gap_AcceptedGapIndices);
indices.Gap_AcceptedGapWhileWaitIndicesNoOutlier = indices.Gap_AcceptedGapWhileWaitIndices(ind);

[~,ind,~] = intersect(indices.Gap_AcceptedGapsWhileApproachIndices,indices.Gap_AcceptedGapIndices);
indices.Gap_AcceptedGapsWhileApproachIndicesNoOutlier = indices.Gap_AcceptedGapsWhileApproachIndices(ind);

 
% %% indices including waiting time outliers
% indices.WCRejectedGaps = indices.Gap_RejectedGapIndices;        %always without outliers for rejected gaps; 
% if out==0
%     indices.WCAllGaps = [indices.Gap_AcceptedGapWhileWaitIndicesNoOutlier;indices.Gap_RejectedGapIndices];
%     indices.WCAcceptedGaps = indices.Gap_AcceptedGapWhileWaitIndicesNoOutlier;
% else
%     indices.WCAllGaps = [indices.Gap_AcceptedGapWhileWaitIndices;indices.Gap_RejectedGapIndices];
%     indices.WCAcceptedGaps = indices.Gap_AcceptedGapWhileWaitIndices;
% end


% includes outliers for crossing gaps
    indices.WCRejectedGaps = indices.Gap_RejectedGapIndices;
    
%     indices.WCAcceptedGaps = [indices.Gap_AcceptedGapWhileWaitIndices];
    indices.WCAcceptedGaps = indices.Gap_AcceptedGapWhileWaitIndicesNoOutlier;
    
    indices.WCAllGaps = [indices.WCAcceptedGaps;indices.WCRejectedGaps];
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% to include the approch to cross/wait gaps when wait starts

if out==1
    indices.WCRejectedGaps = unique([indices.WCRejectedGaps]);
    indices.WCAcceptedGaps = unique([indices.WCAcceptedGaps;indices.Gap_AcceptedGapsWhileApproachIndices]);
    indices.WCAllGaps = [indices.WCAcceptedGaps;indices.WCRejectedGaps];
end
    
  

end