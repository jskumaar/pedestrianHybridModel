function animateGame(X0,time,NA,NB,flagA_arr,flagB_arr,Amode,Bmode)

global XG wSA wSB XFA XFB rho_FA rho_FB deltaA deltaB
N=NA+NB;
tlen=length(time);
for ii=1:N
    X(2*ii-1:2*ii,:)=X0(4*ii-3:4*ii-2,:);  %get the positions only
end
figure('units','normalized','outerposition',[.2 0.3 .6 .5])
%Start Recording
vid=VideoWriter('Results/test.avi');
vid.FrameRate=60;
open(vid);
makeVideo=1;
%%
fontSize=15;
%Plot the game domain, safe zone, and flag zones
tic
rectangle('Position',[XG(1,1),XG(1,2),XG(2,1)-XG(1,1),XG(4,2)-XG(1,2)]);
hold on
%Home zone for team A
rectangle('Position',[XG(1,1),XG(1,2),wSA,XG(4,2)-XG(1,2)],'FaceColor',[0,0,1,0.5]);
hold on
text(XG(1,1)+10,XG(1,2)+10,'$S_A$','fontsize',fontSize)
%Home zone for team B
rectangle('Position',[XG(2,1)-wSB,XG(2,2),wSB,XG(4,2)-XG(1,2)],'FaceColor',[1,0,0,0.5]);
hold on
text(XG(2,1)-25,XG(2,2)+10,'$S_B$','fontsize',fontSize)
%Mid-line of the game domain
plot(0.5*(XG(1,1)+XG(2,1))*ones(1,2),[XG(1,2),XG(4,2)],'k--')
%Flag zone of team A
ang=0:2*pi/100:2*pi;
fill(XFA(1)+rho_FA*cos(ang),XFA(2)+rho_FA*sin(ang),[0,0,1]);
alpha(0.5); %transparency
hand_flagA=fill([XFA(1)-0.2*rho_FA,XFA(1)-0.2*rho_FA,XFA(1)+0.65*rho_FA,XFA(1)-0.2*rho_FA],[XFA(2)-0.8*rho_FA,XFA(2)+0.8*rho_FA,XFA(2)+0.4*rho_FA,XFA(2)],'b');
plot(XFA(1)+deltaA*cos(ang),XFA(2)+deltaA*sin(ang),'b--')

%Flag zone of team B
fill(XFB(1)+rho_FB*cos(ang),XFB(2)+rho_FB*sin(ang),[1,0,0])
alpha(0.5); %transparency
hand_flagB=fill([XFB(1)-0.2*rho_FB,XFB(1)-0.2*rho_FB,XFB(1)+0.65*rho_FB,XFB(1)-0.2*rho_FB],[XFB(2)-0.8*rho_FB,XFB(2)+0.8*rho_FB,XFB(2)+0.4*rho_FB,XFB(2)],'r');
plot(XFB(1)+deltaB*cos(ang),XFB(2)+deltaB*sin(ang),'r--')

dax=20;
axis([XG(1,1)-dax,XG(2,1)+dax,XG(1,2)-dax,XG(4,2)+dax])
%%
%Intial positions\
markSize=8;
for j=1:N
    if j<=NA
        plot(X(2*(j-1)+1,1),X(2*(j-1)+2,1),'bsquare','markersize',markSize);
        hold on;
        hand(j)=plot(X(2*(j-1)+1,1),X(2*(j-1)+2,1),'bo','markersize',markSize);        
       textModeA =text(X(2*(j-1)+1,1)-7,X(2*(j-1)+2,1)+7,num2str(Amode(1)));
    else
        plot(X(2*(j-1)+1,1),X(2*(j-1)+2,1),'rsquare','markersize',markSize);
        hold on;
        hand(j)=plot(X(2*(j-1)+1,1),X(2*(j-1)+2,1),'ro','markersize',markSize);
        textModeB= text(X(2*(j-1)+1,1)-7,X(2*(j-1)+2,1)+7,num2str(Bmode(1)));
    end
end
hold on;


xlabel('x [m]');
ylabel('y [m]');
title('Hybrid Capture-the-Flag Game');


FlagHistory=1;
if FlagHistory==1
    ns=30;  %add ns values everytime the plot is updated
    for i=1:ns:tlen
        if i+ns-1<=tlen
            for j=1:N
                
                %Delete the captured flag from the corresponding flag zone
                if flagA_arr(i+ns-1)==1
                    if exist('hand_flagA','var')
                        delete(hand_flagA);
                    end
                elseif flagB_arr(i+ns-1)==1
                    if exist('hand_flagB','var')
                        delete(hand_flagB);
                    end
                end
                
                if j<=NA
                    %plot(X(2*(j-1)+1,i:i+ns-1),X(2*(j-1)+2,i:i+ns-1),'bo','markersize',markSize)
                    set(hand(j),'XData',X(2*(j-1)+1,i+ns-1),'YData',X(2*(j-1)+2,i+ns-1),'markersize',markSize);
                    %modeText = text(X(2*(j-1)+1,i+ns-1)-7,X(2*(j-1)+2,i+ns-1)+7,num2str(Amode(i+ns-1)));
                    set(textModeA,'Position',[X(2*(j-1)+1,i+ns-1)-7,X(2*(j-1)+2,i+ns-1)+7,0],'String',num2str(Amode(i+ns-1)));
                    if ~isvalid(hand_flagB) && ~exist('handAwF','var')
                        handAwF=plot(X(2*(j-1)+1,i+ns-1),X(2*(j-1)+2,i+ns-1),'rd','markersize',1.5*markSize);
                    end
                    if exist('handAwF','var')
                        set(handAwF,'XData',X(2*(j-1)+1,i+ns-1),'YData',X(2*(j-1)+2,i+ns-1));
                    end   
                    drawnow;
                else
                    
                    %plot(X(2*(j-1)+1,i:i+ns-1),X(2*(j-1)+2,i:i+ns-1),'ro','markersize',markSize);
                    set(hand(j),'XData',X(2*(j-1)+1,i+ns-1),'YData',X(2*(j-1)+2,i+ns-1),'markersize',markSize);
                    %modeText = text(X(2*(j-1)+1,i+ns-1)-7,X(2*(j-1)+2,i+ns-1)+7,num2str(Bmode(i+ns-1)));
                    set(textModeB,'Position',[X(2*(j-1)+1,i+ns-1)-7,X(2*(j-1)+2,i+ns-1)+7,0],'String',num2str(Bmode(i+ns-1)));
                    if ~isvalid(hand_flagA) && ~exist('handBwF','var')
                        handBwF=plot(X(2*(j-1)+1,i+ns-1),X(2*(j-1)+2,i+ns-1),'bd','markersize',1.5*markSize);
                    end
                    if exist('handBwF','var')
                        set(handBwF,'XData',X(2*(j-1)+1,i+ns-1),'YData',X(2*(j-1)+2,i+ns-1));
                    end                   
                    drawnow;
                    
                end
                hold on;
                
                
                if makeVideo==1
                    fr=getframe(gcf);
                    writeVideo(vid,fr);
                    %delete(modeText);
                end
            end
        end
    end
elseif FlagHistory==0
    for i=2:tlen
        for j=1:N
            if j<=NA
                set(hand(j),'XData',X(2*(j-1)+1,1:i),'YData',X(2*(j-1)+2,1:i));
                drawnow;
                text(X(2*(j-1)+1,1:i)-5,X(2*(j-1)+2,1:i)+5,num2str(Amode(1:i)));
                hold on;
            else
                set(hand(j),'XData',X(2*(j-1)+1,1:i),'YData',X(2*(j-1)+2,1:i));
                drawnow;
                text(X(2*(j-1)+1,1:i)-5,X(2*(j-1)+2,1:i)+5,num2str(Bmode(1:i)));
                hold on;
            end
            hold on;
            if makeVideo==1
                fr=getframe(gcf);
                writeVideo(vid,fr);
            end
        end
        %drawnow;
    end
end

%Final positions
for j=1:N
    if j<=NA
        set(hand(j),'XData',X(2*(j-1)+1,tlen),'YData',X(2*(j-1)+2,tlen));
        hold on;
        plot(X(2*(j-1)+1,tlen),X(2*(j-1)+2,tlen),'bo','markersize',1.5*markSize)
        set(textModeA,'Position',[X(2*(j-1)+1,tlen)-7,X(2*(j-1)+2,tlen)+7,0],'String',num2str(Amode(tlen)));
    else
        set(hand(j),'XData',X(2*(j-1)+1,tlen),'YData',X(2*(j-1)+2,tlen));
        hold on;
        plot(X(2*(j-1)+1,tlen),X(2*(j-1)+2,tlen),'ro','markersize',1.5*markSize)
        set(textModeB,'Position',[X(2*(j-1)+1,tlen)-7,X(2*(j-1)+2,tlen)+7,0],'String',num2str(Bmode(tlen)));
    end
    hold on;
    if makeVideo==1
        fr=getframe(gcf);
        writeVideo(vid,fr);
    end
end
close(vid)
PlotTime=toc;
end
