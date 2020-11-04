%% debug

% check if the dataset hs calculated the cwInd, swInd, and Lane correctly

indices_imp = [1, 46, 167, 34, 76];

sceneId_interest = indices_imp(1);
car_index_interest = indices_imp(2);
pedTrackId_interest = indices_imp(3);
predictionTimeStep_interest = indices_imp(4);
actualTimeStep_interest = indices_imp(5);

sf = 0.0978;

% % load image and dataset
% load('tracksData_reSampled_v11.mat')
% load('inD_trackDescriptives_v3.mat') 

% load background images
% annotatedImage = imread(strcat('annotated_',num2str(18),'_background.png'));        %using the same background image for all scenes as it roughly remains the same; also segmenting all images takes time.
% imgSize = size(annotatedImage);

% % for better visualization, increase the grayscale value of the different regions
% annotatedImageEnhanced = annotatedImage;
% for ii = 1:imgSize(1)
%     for jj = 1:imgSize(2)
%         if (annotatedImage(ii,jj)==2) %Road
%             annotatedImageEnhanced(ii,jj) = 50;
%         end
%         if (annotatedImage(ii,jj)==4 || annotatedImage(ii,jj)==5 || annotatedImage(ii,jj)==6) %unmarked crosswalk
%             annotatedImageEnhanced(ii,jj) = 100;
%         end        
%         if (annotatedImage(ii,jj)==3) %Sidewalk
%             annotatedImageEnhanced(ii,jj) = 150;
%         end        
%         if (annotatedImage(ii,jj)==1) %marked crosswalk
%             annotatedImageEnhanced(ii,jj) = 200;
%         end
%     end
% end

% plot tracks, cwInd, swInd, and Lane
for sceneId = sceneId_interest
    
    
    for track_index = pedTrackId_interest
        pedData = formattedTracksData{sceneId}{track_index};
        N_ts = size(pedData.frame,1);
        
        for timeStep = 1:N_ts
            
            % pedestrian position
            pedPosPixels = int32([pedData.xCenter(timeStep), pedData.yCenter(timeStep)]/sf);
            annotatedImageEnhanced(-pedPosPixels(2), pedPosPixels(1)) = 255;
            
            % cwInd, swInd and Lane
            text = strcat('cwInd: ', num2str(pedData.closestCW(timeStep)));
            annotatedImageEnhanced_text = insertText(annotatedImageEnhanced,[10, 10], text);
                        
            text = strcat('swInd: ', num2str(pedData.swInd(timeStep)));
            annotatedImageEnhanced_text = insertText(annotatedImageEnhanced_text,[10, 30], text);
            
            text = strcat('Lane: ', pedData.Lane(timeStep));
            annotatedImageEnhanced_text = insertText(annotatedImageEnhanced_text,[10, 50], text);
            
            % display image
            imshow(annotatedImageEnhanced_text);
            pause(0.3)
        
        end
    end
end
        


