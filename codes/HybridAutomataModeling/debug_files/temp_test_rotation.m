%% test image rotation
imgCenter = [Params.imgSize(2), -Params.imgSize(1)]/2;

annotatedImageEnhanced = annotatedImage;
for ii = 1:imgSize(1)
    for jj = 1:imgSize(2)
        if (annotatedImage(ii,jj)==2) %Road
            annotatedImageEnhanced(ii,jj) = 50;
        end
        if (annotatedImage(ii,jj)==4 || annotatedImage(ii,jj)==5 || annotatedImage(ii,jj)==6) %unmarked crosswalk
            annotatedImageEnhanced(ii,jj) = 100;
        end        
        if (annotatedImage(ii,jj)==3) %Sidewalk
            annotatedImageEnhanced(ii,jj) = 150;
        end        
        if (annotatedImage(ii,jj)==1) %marked crosswalk
            annotatedImageEnhanced(ii,jj) = 200;
        end
    end
end

%
theta = reshape([cw.theta; cw.theta], [8,1]);

for ii=1:1
    
    cw_Ind = ceil(ii/2);
    cw_ped_rot = [cosd(theta(ii)), -sind(theta(ii)); sind(theta(ii)), cosd(theta(ii))];
    
    pedPosPixels = resetStates.walkaway.erCw1Final(1,:);
    % goal location in rotated frame
    a = (cw_ped_rot * (pedPosPixels  - double(imgCenter))' + double(imgCenter)');
    
    % goal bounding box in rotated frame
    if cw_Ind==1 || cw_Ind==2
        cw_Ind
        a
        a1 = a - [5;10]
        a2 = a - [-5;10]
        a3 = a - [5;0]
        a4 = a - [-5;0]
    end
    
    
    if cw_Ind==3 || cw_Ind==4        
        cw_Ind
        a
        a1 = a - [10;5]
        a2 = a - [10;-5]
        a3 = a - [0;5]
        a4 = a - [0;-5]
    end
    
    b1 = inv(cw_ped_rot)*(a1-double(imgCenter)') + double(imgCenter)'
    b2 = inv(cw_ped_rot)*(a2-double(imgCenter)') + double(imgCenter)'
    b3 = inv(cw_ped_rot)*(a3-double(imgCenter)') + double(imgCenter)'
    b4 = inv(cw_ped_rot)*(a4-double(imgCenter)') + double(imgCenter)'
    
    
    bb_walkaway_erCw1Final(ii,:) = [b1', b2', b3', b4'];
    x=1;

end








