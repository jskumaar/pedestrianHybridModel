%%
%%%% This file is a compilation of different kinds of plots that can be used %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% 1) Histogram
figure()
h=histogram(Data,NumberOfBins);
%h=histogram(Data,NumberOfBins,'Normalization','probability');   %to plot probability density
h.FaceColor = 'none';       %adjust colour of the bin
xlabel('Waiting time [s]')
ylabel('Counts [m]')
title('Waiting time distribution')
set(gca,'fontsize', 18)
%To plot the distribution curve
hold on;
plot(conv(h.BinEdges, [0.5 0.5], 'valid'), h.BinCounts/sum(h.BinCounts))
%To save figure
saveas(gcf,'Waiting time distribution','fig')
saveas(gcf,'Waiting time distribution','png')
saveas(gcf,'Waiting time distribution','eps')



%% 2) Bar Graph
figure()
y = [1.40,1.52;1.58, 1.68];
b=bar(y);
axis([0.5,2.5,0,2.5])
width = b.BarWidth;

ylabel('Walking Speed [m/s]')
title('Pedestrian walking speed comparison')
set(gca,'fontsize', 18)
set(gca,'xticklabel',{'On Sidewalk','On Crosswalk'})
legend('Real world and human-driven vehicle','VR & Automated vehicle');
for i=1:length(y(:, 1))
    row = y(i, :);
    % 0.5 is approximate net width of white spacings per group
    offset = (((width)-0.15) / length(row)) / 2;
    x = linspace(i-offset, i+offset, length(row));
    text(x,row,num2str(row'),'vert','bottom','horiz','center','FontSize',18);
end
%To save figure
FigH = figure('Position', get(0, 'Screensize'));
saveas(FigH,'Pedestrian walking speed comparison','fig')
saveas(FigH,'Pedestrian walking speed comparison','png')
saveas(FigH,'Pedestrian walking speed comparison','eps')

%% 3) Line Chart
figure()
plot(x,'-r*','MarkerSize',15);hold on;
plot(y,'-b*','MarkerSize',15);
ylabel('Pedestrian speed [m/s]')
xlabel('Encounter number based on driving condition')
title('Walking Speed and Crossing Speed Evolution Condition order')
legend('Walking Speed ','Crossing Speed')
set(gca,'fontsize', 18)
%To save figure
filename = 'Walking Speed and Crossing Speed Evolution Condition order';
% FigH = figure('Position', get(0, 'Screensize'));
saveas(gca,filename,'fig')
saveas(gca,filename,'png')
saveas(gca,filename,'eps')
saveas(gca,filename,'emf')

% FigH = figure('Position', get(0, 'Screensize'));
figure()
plot(x2,'-r*','MarkerSize',15);hold on;
plot(y2,'-b*','MarkerSize',15);
ylabel('Pedestrian speed [m/s]')
xlabel('Encounter number based on experiment order')
title('Walking Speed and Crossing Speed Evolution Experiment Order')
legend('Walking Speed','Crossing Speed')
set(gca,'fontsize', 18)
%To save figure
filename = 'Walking Speed and Crossing Speed Evolution Experiment Order';
saveas(gca,filename,'fig')
saveas(gca,filename,'png')
saveas(gca,filename,'eps')
saveas(gca,filename,'emf')


%% Error bar

er = errorbar(x,data,errlow,errhigh);  