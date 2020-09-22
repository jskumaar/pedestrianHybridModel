pedVisionPoints = [0, 0;
                   300, 0;
                   260, 200];
               
carBoxPoints = [278, 100;
                330, 100;
                330, 150;
                278, 150];
            
            
ms = 12;
            
% figure()
% 
% for ii=1:length(pedVisionPoints)
%    plot(pedVisionPoints(ii,1), pedVisionPoints(ii,2), 'b*', 'MarkerSize', ms);hold on;
% end
% 
% for ii=1:length(carBoxPoints)
%    plot(carBoxPoints(ii,1), carBoxPoints(ii,2), 'r*', 'MarkerSize', ms);hold on;
% end

%% a)
tic
carBox = polyshape(carBoxPoints(:,1), carBoxPoints(:,2));
pedBox = polyshape(pedVisionPoints(:,1), pedVisionPoints(:,2));
intersectRegion = intersect(carBox, pedBox);
if ~isempty(intersectRegion.Vertices)
    isLooking = true;
else
    isLooking = false;   
end
disp('polyshape')
isLooking
toc

%%b) 
tic
[in,on] = inpolygon(carBoxPoints(:,1),carBoxPoints(:,2),pedVisionPoints(:,1),pedVisionPoints(:,2));
if ~isempty(in) || ~isempty(on)
    isLooking = true;
else
    isLooking = false;   
end
disp('inpoly')
isLooking
toc
    

% c)
tic
pedBox = polyshape(pedVisionPoints(:,1), pedVisionPoints(:,2));
[TFin,TFon] = isinterior(pedBox, carBoxPoints);
if ~isempty(TFin) || ~isempty(TFon)
    isLooking = true;
else
    isLooking = false;   
end
disp('isinterior')
isLooking
toc