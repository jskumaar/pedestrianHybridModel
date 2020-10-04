% temp_check_rotation

imgCenter = [Params.imgSize(2), -Params.imgSize(1)]/2*(Params.orthopxToMeter*Params.scaleFactor);

for sceneId = 1:1
    
  pedTracks = [tracks{sceneId}.pedCrossingTracks; tracks{sceneId}.pedNotCrossingTracks];
 
%    for trackNo = 1:length(pedTracks)
   for trackNo = 1:1
       pedTrackId = pedTracks(trackNo);
       pedData = formattedTracksData{sceneId}{pedTrackId};
       closestCW = pedData.closestCW;
           
       for ii=1:length(closestCW)
          if closestCW(ii)~=0 && closestCW(ii)~=inf
              theta = cw.theta(closestCW(ii));
              cwPos = double([cw.center_x(closestCW(ii)), cw.center_y(closestCW(ii))]) * (Params.orthopxToMeter*Params.scaleFactor);
              rot = [cosd(theta), -sind(theta); sind(theta), cosd(theta)];
              pedPos = [pedData.xCenter(ii), pedData.yCenter(ii)];
              pedPosRot = ((pedPos  - double(imgCenter))*rot + double(imgCenter));
              cwPosRot =  ((cwPos - double(imgCenter) * rot) + double(imgCenter));
              dispPedCw(ii,:) = pedPosRot - cwPosRot;
          else
              dispPedCw(ii,:) = [inf,inf];
          end
  
       end

   end
end  
    


