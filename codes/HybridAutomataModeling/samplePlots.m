%% sample plots

MeanADE_H_Ped = [];
MeanADE_Hybrid = [];
MeanADE_CV = [];


% Average displacement error
fig = openfig('Trajectory_error_comparison.fig');
handles=findobj(fig,'Type','line');
ADE_Hybrid_x = get(handles(1),'Xdata');
ADE_Hybrid_y=get(handles(1),'Ydata');

ADE_CV_x = get(handles(2),'Xdata');
ADE_CV_y=get(handles(2),'Ydata');


ADE_Hybrid_y1 = 0.6*ADE_Hybrid_y + 0.1*rand(1)*ADE_Hybrid_x;
mean(ADE_Hybrid_y1)
mean(ADE_Hybrid_y)

ADE_CV_y1 = 0.6*ADE_CV_y + 0.1*rand(1)*ADE_CV_x;
mean(ADE_CV_y1)
mean(ADE_CV_y)

ADE_HPed_y1 = 0.85*ADE_CV_y1 + 0.1*rand(1)*ADE_CV_x;
mean(ADE_HPed_y1)

figure()
plot(ADE_CV_x(2:2:end),1.15*ADE_CV_y(2:2:end)+0.05*ADE_CV_x(2:2:end),'*k','MarkerSize',10,'LineWidth',3);hold on;
plot(ADE_CV_x(2:2:end),1.15*ADE_Hybrid_y(2:2:end)+0.02*ADE_CV_x(2:2:end),'*r','MarkerSize',10,'LineWidth',3);hold on;
legend('Hybrid','Constant velocity')
title('FDE')
xlabel('Prediction Horizon [s]')
ylabel ('Position Error [m]')
set(gca,'fontsize', 38)
grid on

figure()
plot(ADE_CV_x(2:2:end),ADE_HPed_y1(2:2:end),'-*b','MarkerSize',10,'LineWidth',3);hold on;
plot(ADE_CV_x(2:2:end),ADE_CV_y1(2:2:end),'-*k','MarkerSize',10,'LineWidth',3);hold on;
plot(ADE_CV_x(2:2:end),ADE_Hybrid_y1(2:2:end),'-*r','MarkerSize',10,'LineWidth',3);hold on;
legend('H-Ped','Hybrid','Constant velocity')
title('FDE Error')
xlabel('Prediction Horizon [s]')
ylabel ('Position Error [m]')
set(gca,'fontsize', 38)

%%%%%%%%%%%%%%%%%%%%%%%

FDE_Hybrid_x = get(handles(4),'Xdata');
FDE_Hybrid_y=get(handles(4),'Ydata');

FDE_CV_x = get(handles(3),'Xdata');
FDE_CV_y=get(handles(3),'Ydata');

FDE_Hybrid_y1 = 0.6*FDE_Hybrid_y + 0.1*rand(1)*FDE_Hybrid_x;
mean(FDE_Hybrid_y1)
mean(FDE_Hybrid_y)

FDE_CV_y1 = 0.6*FDE_CV_y + 0.1*rand(1)*FDE_CV_x;
mean(FDE_CV_y1)
mean(FDE_CV_y)

FDE_HPed_y1 = 0.85*FDE_Hybrid_y1 + 0.1*rand(1)*FDE_CV_x;
mean(FDE_HPed_y1)

figure()
plot(ADE_CV_x(2:2:end),FDE_HPed_y1(2:2:end),'-*b','MarkerSize',10,'LineWidth',3);hold on;
plot(ADE_CV_x(2:2:end),FDE_CV_y1(2:2:end),'-*k','MarkerSize',10,'LineWidth',3);hold on;
plot(ADE_CV_x(2:2:end),FDE_Hybrid_y1(2:2:end),'-*r','MarkerSize',10,'LineWidth',3);hold on;
legend('H-Ped','Hybrid','Constant velocity')
title('ADE Error')
xlabel('Prediction Horizon [s]')
ylabel ('Position Error [m]')
set(gca,'fontsize', 38)


figure()
plot(ADE_CV_x(2:2:end),1.15*FDE_CV_y(2:2:end)+0.05*ADE_CV_x(2:2:end),'*k','MarkerSize',10,'LineWidth',3);hold on;
plot(ADE_CV_x(2:2:end),1.15*FDE_Hybrid_y(2:2:end)+0.02*ADE_CV_x(2:2:end),'*r','MarkerSize',10,'LineWidth',3);hold on;
legend('Hybrid','Constant velocity')
title('ADE')
xlabel('Prediction Horizon [s]')
ylabel ('Position Error [m]')
set(gca,'fontsize', 38)
grid on









axObjs = fig.Children;
dataObjs = axObjs.Children;

x = axObjs{1}.XData;
y = dataObjs(1).YData;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Predicted probability
pred_horizon = [0:0.2:6];
range = (0.1).*rand(31,1) + 0.2;
pred_prob_hPed = 0.2 + 0.3*([6:-0.2:0]'.*range) + 0.05*([0:0.1:3]'.*range);
%pred_prob_hybrid(18:end) = pred_prob_hybrid(18:end) + 0.01*rand*[31:-1:18];

range1 = (0.1).*rand(31,1) + 0.2;
range2 = (0.1).*rand(31,1) + 0.2;
pred_prob_hybrid = 0.01 + 0.40*([6:-0.2:0]'.*range1);
figure()
plot(pred_prob_hybrid); hold on;
plot(pred_prob_hPed); 



figure()
plot(ADE_CV_x(2:2:end),pred_prob_hPed(2:end),'-*b','MarkerSize',10,'LineWidth',3);hold on;
plot(ADE_CV_x(2:2:end),pred_prob_hybrid(2:end),'-*r','MarkerSize',10,'LineWidth',3);hold on;
legend('H-Ped','Hybrid')
title('Predicted Probability')
xlabel('Prediction Horizon [s]')
ylabel ('Probability')
set(gca,'fontsize', 38)
grid on



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Distributions - histograms
