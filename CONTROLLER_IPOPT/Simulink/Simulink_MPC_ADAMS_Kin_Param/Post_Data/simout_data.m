% %%%%%% DA ESEGUIRE UNA VOLTA SOLA ALL'INIZIO

robot=mobile_manipulator(0.185,0,0.7,pi/2);

% COMPILE MEX CODE
cd Post_Data/PLOTMM/ur_kinematics;
run compile_ur5kinematics.m
cd ..
cd ..
cd ..
%% 
close all
clearvars -EXCEPT robot simout xd T_elapsed_vect X_steps N m_b m_m Ak_base Ak_ee nometraj man_index Ak_mm T J_comp
clc

% save data = 1 (YES)
% save data = 0 (NO)
savedata = 0; 

%% Richiamo e Ricostruisco vettore X

simoutin=simout.';

for i=1:size(simoutin,2)
    
    i=real(i);
    
    X(1,i)       = simoutin(1,i);
    X(2,i)       = simoutin(2,i);
    X(3,i)       = simoutin(3,i);
    X(4:9,i)     = simoutin(4:9,i);
    X(10:15,i)   = X_steps.signals.values(10:15,1,i);
    
end

if size(X,2)>=size(xd,2)
    X=X(:,1:size(xd,2));
else
    xd=xd(:,1:size(X,2));
end

%% Calcolo errori sulla traiettoria

err_vec     = X-xd;
err_vec_abs = abs(err_vec);
abs_xy_err  = sqrt( err_vec(1,:).^2 + err_vec(2,:).^2 );
abs_pee_err = sqrt( err_vec(10,:).^2 + err_vec(11,:).^2 + err_vec(12,:).^2 );

maxerrors   = max(err_vec_abs);
minerrors   = min(err_vec_abs);

%% STATIC PLOTS

figure(1) 
plot(abs_xy_err)
grid on
title('ABS ERROR XY BASE')

figure(2)
plot(X(1,:),X(2,:),'-o')
hold on 
grid on
plot(xd(1,:),xd(2,:),'-*')
legend('computed','desired')
title('MOVEMENT OF THE BASE')

figure(3)
plot(abs_pee_err);
grid on
title('ABS ERROR XYZ EE')

fig4=figure(4);
fig4.Position = [0 1200-900 1440 900];
plot3(X(10,:),X(11,:),X(12,:),'-o')
hold on
grid on
plot3(xd(10,:),xd(11,:),xd(12,:),'-*')
axis([[-1 4 -1 2 0 2]])
legend('computed','desired')
title('MOVEMENT OF THE EE XYZ')

figure(5)
plot(man_index)
grid on
title('Manipulability Index')

%% PLOT DRAW

fig6=figure(6);

fig6.Position = [0 1200-900 1440 900];
[Xc,Yc,Zc]=cylinder(0.13,10000);

Zc=Zc+0.5;
[Xcc,Ycc,Zcc]=cylinder(0.7384,10000);
Xc=Xc-0.76/2;
Yc=Yc-0.2;
Zc2=Ycc;
[Xs,Ys,Zs] = sphere(10);
rs=0.2;
Xs=Xs*rs;
Ys=Ys*rs;
Zs=Zs*rs;

Xc2=Zcc-0.4;
Yc2=Xcc;

    
for j=1:size(X,2)
    
    [P_ee,Psi_ee,T]=FK(X(:,j));
    
    Xss=Xs+T(33,4);
    Yss=Ys+T(34,4);
    Zss=Zs+T(35,4);
    
    
    camlight;
    material metal;
    robot.visualize_mm(X(1:9,j).',fig6);
    axis([-1 1 -1 1 0 2])

    plot3(xd(10,:),xd(11,:),xd(12,:),'r')
    xlabel('x')
    ylabel('y')
    zlabel('z')
    grid on
    hold on
    plot3(xd(1,:),xd(2,:),xd(12,:).*0,'k')
    surf(Xc,Yc,Zc,'facealpha',0.4,'edgealpha',0.3,'LineWidth',5)
%     surf(Xc2,Yc2,Zc2,'facealpha',0.02,'edgealpha',0.03)
    surf(Xss,Yss,Zss,'facealpha',0.5,'edgealpha',0.06)
    view(0,0);%-10, 20);
    drawnow();
    

%     [T,P] = TeP(X(1:9,j));
%     plot3(P(1,3:4),P(2,3:4),P(3,3:4),'b','LineWidth',10)
%     hold on
%     plot3(P(1,4:5),P(2,4:5),P(3,4:5),'color',[0.5 0.5 0.5],'LineWidth',3)
%     plot3(P(1,5:6),P(2,5:6),P(3,5:6),'k','LineWidth',3)
%     plot3(P(1,6:7),P(2,6:7),P(3,6:7),'color',[0.5 0.5 0.5],'LineWidth',3)
%     plot3(P(1,7:8),P(2,7:8),P(3,7:8),'k','LineWidth',3)
%     plot3(P(1,8:9),P(2,8:9),P(3,8:9),'color',[0.5 0.5 0.5],'LineWidth',3)
%     plot3(P(1,9:10),P(2,9:10),P(3,9:10),'k','LineWidth',3)
%     plot3(xd(10,:),xd(11,:),xd(12,:),'r')
%     plot3(xd(1,:),xd(2,:),xd(12,:).*0,'k')
%     grid on
%     axis([[-1 2 -1 2 0 2]])
%     hold off
%     view(-40, 20);
%     xlabel('x')
%     ylabel('y')
%     zlabel('z')

      delete(gca)
%     F_video(j)=getframe(gcf); % get frame for the video
end

% video= VideoWriter('Sim_MM.avi');
% myVideo.FrameRate = 40;
% open(video)
% writeVideo(video,F_video)
% close(video)

%% PLOT t_elapsed

figure(7)
set(gcf,'color','white')
plot(T_elapsed_vect)
ylabel('T elapsed')
xlabel('Step')
grid on

clc

%% SAVE DATA
F_video=0;

if savedata == 1

savename = ['SIM_' nometraj '_' datestr(now, 'HH-MM dd-mmm-yyyy')];
matfile = fullfile('Data_saved/', savename);
save(matfile,'X','xd','N','T','T_elapsed_vect','Ak_base','Ak_ee','m_b','Ak_mm', ...
            'm_m','err_vec','err_vec_abs','abs_xy_err','abs_pee_err', ...
            'simout','X_steps','F_video','man_index')

end
