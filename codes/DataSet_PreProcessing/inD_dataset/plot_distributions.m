% %% plot wait time distribution
% waitTime = [];
% for sceneId = 1:12
%    pedTracks = [tracks{sceneId}.pedCrossingTracks; tracks{sceneId}.pedNotCrossingTracks];
%  
%    for trackNo = 1:length(pedTracks)
%        pedTrackId = pedTracks(trackNo);
%        pedData = formattedTracksData{sceneId}{pedTrackId};
%        
%        crossStartTime = find(pedData.HybridState=="Crossing",1,'first');
%        if ~isempty(crossStartTime)
%            if isfield(pedData, 'waitTimeSteps')
%                 waitTime = [waitTime; pedData.waitTimeSteps(crossStartTime)/25];  
%            elseif isfield(pedData, 'wait_time_steps')
%                 waitTime = [waitTime; pedData.wait_time_steps(crossStartTime)/25];
%            else
%                x=1;
%            end
%        end
%    end   
% end
% 
% 
% %% plot histogram
% figure()
% h2 = histogram(waitTime,'Normalization','probability','BinWidth',0.5);
% xlabel('Wait time[s]')
% ylabel('Probability')
% title('Wait time distribution')
% yData = h2.Values;
% xData = [0.5: 0.5: 45];
% mean(waitTime)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

distCW = [];
tracks_no = 1;
for sceneId = 1:12
pedTracks = [tracks{sceneId}.pedCrossingTracks; tracks{sceneId}.pedNotCrossingTracks];
 
   for trackNo = 1:length(pedTracks)
       pedTrackId = pedTracks(trackNo);
       pedData = formattedTracksData{sceneId}{pedTrackId};
       
       if isfield(pedData, 'longDispPedCw')
            distCW = [distCW; pedData.longDispPedCw];
            tracks_no = tracks_no + 1;
       end
   end
end

%% plot histogram
figure()
h2 = histogram(distCW,'Normalization','probability','BinWidth',1);
xlabel('Dist CW[s]')
ylabel('Probability')
title('Dist CW distribution')


figure()
h3 = histogram(GapFeatures_SVM_legal.F_cumWait,'Normalization','probability','BinWidth',1);
xlabel('Wait[s]')
ylabel('Probability')
title('Waitdistribution')
