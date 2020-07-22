function [ObservationType,DataBinned,Num_levels] = CombineObservations(Data,binsize,startlimit,endlimit)

%%
N = size(Data,2);
len = size(Data,1);
del = 0.001;    % to avoid zero indices in bins



for ii=1:N
   DataBinned(:,ii) = min(ceil((Data(:,ii)-startlimit(ii)+del)/binsize(ii)),ceil((endlimit(ii)-startlimit(ii))/binsize(ii)));
   unique_bins(ii,1) = ceil((endlimit(ii)-startlimit(ii))/binsize(ii));
end



ObservationType = zeros(len,1);
unique_bins=[unique_bins;1];

for ii=1:N

ObservationType = [ObservationType + (DataBinned(:,ii)-1).*prod(unique_bins(ii+1:end))];

end

ObservationType = ObservationType + 1;
Num_levels = prod(unique_bins(1:end-1));

end