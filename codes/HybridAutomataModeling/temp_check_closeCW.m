%% load image

% % load background images
% annotatedImage = imread(strcat('annotated_',num2str(18),'_background.png'));        %using the same background image for all scenes as it roughly remains the same; also segmenting all images takes time.
% imgSize = size(annotatedImage);
% 
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


sceneId = 1;
trackId = 5;
startTimeStep = 1;

scaleFactor = Params.scaleFactor;
orthopxToMeter = Params.orthopxToMeter;


pedData = formattedTracksData{sceneId}{trackId};

N_timeSteps = size(pedData.trackLifetime,1);
for ii = 1:N_timeSteps
    
    xCenterPix =  int32(pedData.xCenter(ii) / (orthopxToMeter * scaleFactor));
    yCenterPix =  int32(pedData.yCenter(ii) / (orthopxToMeter * scaleFactor));
    
    annotatedImageEnhanced(-yCenterPix, xCenterPix) = 255;
end

imshow(annotatedImageEnhanced)

xCenterPix = int32(pedData.xCenter(116) / (orthopxToMeter * scaleFactor));
yCenterPix =  int32(pedData.yCenter(116) / (orthopxToMeter * scaleFactor));
annotatedImageEnhanced(-yCenterPix, xCenterPix) = 10;
