        %has the pedestrian started crossing?
        if ( strcmp(currentPedData.HybridState{time_step},'Crossing') && ~strcmp(currentPedData.HybridState{time_step-1},'Crossing') )

            % if the pedestrian crossed for the ego-vehicle
            % (closeCar_ind), whose gap started sometime
            % back
            GapFeatures_ind_temp1 = find(GapFeatures.recording == currentPedData.recordingId(1)); 
            GapFeatures_ind_temp2 = find(GapFeatures.pedTrack  == ped_track); 
            GapFeatures_ind_temp3 = find(GapFeatures.egoCarTrack == closeCar_ind); 

            GapFeatures_ind = intersect(intersect(GapFeatures_ind_temp1,GapFeatures_ind_temp2), GapFeatures_ind_temp3);

            if ~isempty(GapFeatures_ind)
                GapFeatures.CrossDecision(GapFeatures_ind,1) = true;
                GapFeatures.CrossStart(GapFeatures_ind,1) = pedFrame;
                GapFeatures.CrossCW(GapFeatures_ind,1) = cw_ped;
            end

        end
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        
start_ind = find(diff(GapFeatures.recording)~=0);
start_ind = [1; start_ind + 1];
end_ind = [start_ind(2:end) - 1; size(GapFeatures,1)];


for ii=1:12
    temp = GapFeatures(start_ind(ii):end_ind(ii),:);
    pedTrack = temp.pedTrack;
    a = unique(pedTrack);
    Ncount = histc(pedTrack, a);
    max(Ncount)
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% load the Gap features
% 

AcceptedGaps = GapFeatures(GapFeatures.CrossDecision==1, :);
CrossDelay = (AcceptedGaps.CrossStart - AcceptedGaps.frame)/25;
figure()
h = histogram(CrossDelay,'BinWidth',2,'Normalization','probability')

% curve fit
y = h.Values;
x = (h.BinEdges(1:end-1) + h.BinEdges(2:end))/2;
figure()
plot(x,y)
% fit curve using the curve fit toolbox GUI

x = x';
y = y';

g = fittype('a-b*exp(-c*x)');
f0 = fit(x,y,g,'StartPoint',[[ones(size(x)), -exp(-x)]\y; 1]);

figure()
a = [0:0.1:50];
plot(a, f0(a)); hold on;
plot(x, y, '*')
