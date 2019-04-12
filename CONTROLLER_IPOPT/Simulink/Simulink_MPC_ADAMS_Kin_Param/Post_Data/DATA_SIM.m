
savedata           = 0;
visualize_plot     = 0;
visualize_3d_plot  = 1;
plot_position_3d   = [20,20]; % AZ,EL
visualize_spheres  = 0;
record_video       = 1;
visualize_horizons = 0;
static_3d_plot     = 0;

prompt             = 'Is this a Real test? (y/n)  ';
real_sim           =  input(prompt,'s');


%% DIMENSION SET

state_x=state_x_sim.';
if exist('state_qp')==1
    if size(state_qp,1)==6
    else
    state_qp=state_qp(:,1:6).';
    end
end
if size(u_given,1)~=8
u_given=u_given.';
end

    if size(state_x,2)>=size(xd,2)
        state_x=state_x(:,1:size(xd,2));
    else
        xd=xd(:,1:size(state_x,2));
    end
close all
% warning off

%% CALCOLO P_EE DAI JOINTS

if exist('P_ee_out')==0
T=cell(size(state_x,2),1);
Psi_ee_out=cell(size(state_x,2),1);

for i=1:size(state_x,2)
[P_ee_out(:,i),Psi_ee_out{i},T{i}]=FK(state_x(1:9,i));
end
end

%% CALCOLO ERROR EE XYZ

    if exist('error_abs_xyz')==0
        error_abs_xyz=sqrt((P_ee_out(1,:)-xd(10,:)).^2+(P_ee_out(2,:)-xd(11,:)).^2+(P_ee_out(3,:)-xd(12,:)).^2);
    end

%% >>>> STATICS PLOTS <<<<
if visualize_plot == 1
close all

%% EE TRAJECTORY 3D (1)

figure(1)

plot3(xd(10,:),xd(11,:),xd(12,:),'r-.','linewidth',1)
hold on
grid on
plot3(P_ee_out(1,:),P_ee_out(2,:),P_ee_out(3,:),'b','linewidth',1)
plot3(P_ee_out_s(1,:),P_ee_out_s(2,:),P_ee_out_s(3,:),'c','linewidth',1)
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
legend('Desired','Real','Simulated')
axis equal

%% ERROR ABS XYZ PLOT (2)

figure(2)
%     plot(error_abs_xyz)
error_ee=sqrt((P_ee_out(1,1:end-2)-xd(10,3:end)).^2+(P_ee_out(2,1:end-2)-xd(11,3:end)).^2+(P_ee_out(3,1:end-2)-xd(12,3:end)).^2);
plot(0:dt:(size(P_ee_out,2)-3)*dt,error_ee,'linewidth',1)
% ACTUALLY MORE CORRECT:
% error_ee=sqrt((P_ee_out(1,1:end-1)-xd(10,2:end)).^2+(P_ee_out(2,1:end-1)-xd(11,2:end)).^2+(P_ee_out(3,1:end-1)-xd(12,2:end)).^2);
% plot(0:dt:(size(P_ee_out,2)-2)*dt,error_ee,'linewidth',1)
grid on
axis tight
ylabel('Cartesian error EE[m]')

%% ARM JOINTS VELOCITIES PLOTS (3) 
if exist('state_qp')==1
    
figure(3)

fatt=2.4;
xp_base = [0 diff(state_x(1,:))]./dt/fatt; 
yp_base = [0 diff(state_x(2,:))]./dt/fatt;
v_base = xp_base.*cos(state_x(3,:)) + yp_base.*sin(state_x(3,:));
om_base = [0 diff(state_x(3,:))]./dt/fatt;
enne= size(state_x,2)-1;
a_base = [diff(v_base) 0]./dt;
alph_base = [diff(om_base) 0]./dt;
a_given = [0 diff(u_given(1,1:enne+1))]./dt;
alph_given = [0 diff(u_given(2,1:enne+1))]./dt;

subplot(241)
plot(0:dt:(enne)*dt,a_base,'-','linewidth',1)
hold on 
grid on
plot(0:dt:(enne)*dt,a_given,'-','linewidth',1)
plot(ones(1,enne+1)*0.1,'k--','linewidth',1)
plot(ones(1,enne+1)*-0.1,'k--','linewidth',1)
axis([0 enne*dt -0.15 0.15])
title('a_{base} ')


subplot(242)
plot(0:dt:(enne)*dt,alph_base,'-','linewidth',1)
hold on 
grid on
plot(0:dt:(enne)*dt,alph_given,'-','linewidth',1)
plot(ones(1,enne+1)*0.1,'k--','linewidth',1)
plot(ones(1,enne+1)*-0.1,'k--','linewidth',1)
axis([0 enne*dt -0.15 0.15])
title('\omega')

subplot(243)
plot(0:dt:(enne)*dt,state_qp(1,2:enne+2),'-','linewidth',1)
hold on
grid on
plot(0:dt:(enne)*dt,u_given(3,1:enne+1),'-','linewidth',1)
plot(ones(1,enne+1)*0.2,'k--','linewidth',1)
plot(ones(1,enne+1)*-0.2,'k--','linewidth',1)
axis([0 enne*dt -0.25 0.25])
title('d\theta_1')
 

subplot(244)
plot(0:dt:(enne)*dt,state_qp(2,2:enne+2),'-','linewidth',1)
hold on
grid on
plot(0:dt:(enne)*dt,u_given(4,1:enne+1),'-','linewidth',1)
plot(ones(1,enne+1)*0.2,'k--','linewidth',1)
plot(ones(1,enne+1)*-0.2,'k--','linewidth',1)
axis([0 enne*dt -0.25 0.25])
title('d\theta_2')
 

subplot(245)
plot(0:dt:(enne)*dt,state_qp(3,2:enne+2),'-','linewidth',1)
hold on
grid on
plot(0:dt:(enne)*dt,u_given(5,1:enne+1),'-','linewidth',1)
plot(ones(1,enne+1)*0.2,'k--','linewidth',1)
plot(ones(1,enne+1)*-0.2,'k--','linewidth',1)
axis([0 enne*dt -0.25 0.25])
title('d\theta_3')
 

subplot(246)
plot(0:dt:(enne)*dt,state_qp(4,2:enne+2),'-','linewidth',1)
hold on
grid on
plot(0:dt:(enne)*dt,u_given(6,1:enne+1),'-','linewidth',1)
plot(ones(1,enne+1)*0.2,'k--','linewidth',1)
plot(ones(1,enne+1)*-0.2,'k--','linewidth',1)
axis([0 enne*dt -0.25 0.25])
title('d\theta_4')
 

subplot(247)
plot(0:dt:(enne)*dt,state_qp(5,2:enne+2),'-','linewidth',1)
hold on
grid on
plot(0:dt:(enne)*dt,u_given(7,1:enne+1),'-','linewidth',1)
plot(ones(1,enne+1)*0.2,'k--','linewidth',1)
plot(ones(1,enne+1)*-0.2,'k--','linewidth',1)
axis([0 enne*dt -0.25 0.25])
title('d\theta_5')


subplot(248)
plot(0:dt:(enne)*dt,state_qp(6,2:enne+2),'-','linewidth',1)
hold on
grid on
plot(0:dt:(enne)*dt,u_given(8,1:enne+1),'-','linewidth',1)
plot(ones(1,enne+1)*0.2,'k--','linewidth',1)
plot(ones(1,enne+1)*-0.2,'k--','linewidth',1)
axis([0 enne*dt -0.25 0.25])
title('d\theta_6')
legend('Measured','Given')
 
end

warning off

%% ARM JOINTS VELOCITIES ERROR (4)
% if exist('state_qp')==1
%     
% aj_error=state_qp-u_given(3:end,:);
% 
% figure(4)
% subplot(321)
% plot(aj_error(1,:))
% grid on
% title('error thetap1')
% 
% subplot(322)
% plot(aj_error(2,:))
% grid on
% title('error thetap2')
% 
% subplot(323)
% plot(aj_error(3,:))
% grid on
% title('error thetap3')
% 
% subplot(324)
% plot(aj_error(4,:))
% grid on
% title('error thetap4')
% 
% subplot(325)
% plot(aj_error(4,:))
% grid on
% title('error thetap5')
% 
% subplot(326)
% plot(aj_error(6,:))
% grid on
% title('error thetap6')
% 
% end

%% T ELAPSED PLOT (5)

figure(5)

plot(T_elapsed_vect)
grid on
title('T elapsed')
xlabel('Iteration')
ylabel('Time [s]')

%% BASE INPUT SPEED (6)

figure(6)

plot(u_given(1:2,:).')
grid on
legend('v','omega')
title('Base control action')

%% BASE MOTION (7)

figure(7)
plot(state_x(1,:),state_x(2,:),'-*','linewidth',1.5,'MarkerSize',4)
grid on
hold on
plot(xd(1,:),xd(2,:),'-o','linewidth',1.5,'MarkerSize',4)
xlabel('x [m]')
ylabel('y [m]')
axis([min(xd(1,:))-0.2 max(xd(1,:))+0.2 min(xd(2,:))-0.2 max(xd(2,:))+0.2])
% title('Base motion XY')

%% J FUNCTION COMPONENTS (8)

for aa=1:size(X_steps.signals.values,3)    
    J_parts(aa,:)=J_comp(:,:,aa);
end
figure(8)
title('cost function terms')
subplot(211)
plot(J_parts)
grid on
legend('J','h1','h2','h3','h4','h5')
subplot(212)
stem(0,sum(J_parts(:,1))/sum(J_parts(:,1))*100,'linewidth',2,'MarkerSize',3)
hold on
stem(1,sum(J_parts(:,2))/sum(J_parts(:,1))*100,'linewidth',2,'MarkerSize',3)
stem(2,sum(J_parts(:,3))/sum(J_parts(:,1))*100,'linewidth',2,'MarkerSize',3)
stem(3,sum(J_parts(:,4))/sum(J_parts(:,1))*100,'linewidth',2,'MarkerSize',3)
stem(4,sum(J_parts(:,5))/sum(J_parts(:,1))*100,'linewidth',2,'MarkerSize',3)
stem(5,sum(J_parts(:,6))/sum(J_parts(:,1))*100,'linewidth',2,'MarkerSize',3)
xlabel('cost function components')
ylabel('% of the total cost')
grid on; hold off

%% MANIPULABILIY INDEX (9)

figure(9)
plot(man_index,'LineWidth',0.8); ylim([0 1])
grid on
title('Manipulability Index')

%% X-Y-Z EE COMPARISON WITH XD (13)


figure(13)

subplot(311)
plot([0:dt:size(P_ee_out(1,1:end-2),2)*dt],P_ee_out(1,1:end-1),'linewidth',1.7)
grid on
hold on
plot([0:dt:size(xd(1,1:end-2),2)*dt],xd(10,2:end),'--','linewidth',1.7)
xlabel('Time [s]')
ylabel('Distance [m]')
legend('Real','Desired')
axis tight
title('X End-Effector')

subplot(312)
plot([0:dt:size(P_ee_out(1,1:end-2),2)*dt],P_ee_out(2,1:end-1),'linewidth',1.7)
grid on
hold on
plot([0:dt:size(xd(1,1:end-2),2)*dt],xd(11,2:end),'--','linewidth',1.7)
xlabel('Time [s]')
axis tight
ylabel('Distance [m]')

title('Y End-Effector')

subplot(313)
plot([0:dt:size(P_ee_out(1,1:end-2),2)*dt],P_ee_out(3,1:end-1),'linewidth',1.7)
grid on
hold on
plot([0:dt:size(xd(1,1:end-2),2)*dt],xd(12,2:end),'--','linewidth',1.7)
xlabel('Time [s]')
ylabel('Distance [m]')
axis tight
title('Z End-Effector')

%% ORIENTATION ERROR (14)

for i =1:size(P_ee_out,2)

    e_orient_x(i) = 1-dot(Psi_ee_out{i}(:,1),xd(13:15,i));
    e_orient_z(i) = 1-dot(Psi_ee_out{i}(:,2),xd(16:18,i));
    
end

figure(14)

subplot(211)
plot([0:0.5:size(P_ee_out(1,1:end-1),2)*0.5],e_orient_x)
grid on
title('orientation error on X')

subplot(212)
plot([0:0.5:size(P_ee_out(1,1:end-1),2)*0.5],e_orient_z)
grid on
title('orientation error on Z')

%% BASE X-Y ERROR PLOT (15)

figure(15)
plot(0:dt:(length(xd(1,:))-1)*dt,sqrt((state_x(1,:)-xd(1,:)).^2+(state_x(2,:)-xd(2,:)).^2),'linewidth',1);
grid on
xlabel('Time [s]')
ylabel('Cartesian error base[m]')

%% BASE PLOT X and Y (16)

figure(16)

subplot(211)
plot(0:dt:(length(xd(1,:))-1)*dt,xd(1,:),'--','linewidth',2)
grid on
hold on
plot(0:dt:(length(xd(1,:))-1)*dt,state_x(1,:),'linewidth',2)
xlabel('Time [s]')
ylabel('x [m]')
legend('Desired','Real')

subplot(212)
plot(0:dt:(length(xd(2,:))-1)*dt,xd(2,:),'--','linewidth',2)
grid on
hold on
plot(0:dt:(length(xd(2,:))-1)*dt,state_x(2,:),'linewidth',2)
xlabel('Time [s]')
ylabel('y [m]')
% legend('Desired','Real')

end

%% >>>> 3D PLOTS <<<<

if visualize_3d_plot == 1

%% 3D PLOT OF MM (10)  

fig11=figure(10);

%figure proprietà
fig11.Position = [0 1200-900 1440 900];

% Genero il robot
robot=mobile_manipulator(0.185,0,0.7,pi/2);
cd Post_Data/PLOTMM/ur_kinematics;
run compile_ur5kinematics.m
cd ..
cd ..
cd ..

% inizializzo funzione frame per il video
F_video = struct('cdata', cell(1,size(X,2)), 'colormap', cell(1,size(X,2)));
    
    for j=1:size(state_x,2)

        camlight;
        material metal;
        hold on
        robot.visualize_mm(state_x(1:9,j).',fig11);
        plot3(xd(10,:),xd(11,:),xd(12,:),'r')
        plot3(xd(1,:),xd(2,:),xd(12,:).*0,'k')
        if visualize_spheres == 1
        spheres_gen(state_x(:,j),T{j})
        end
        xlabel('x')
        ylabel('y')
        zlabel('z')
        grid on

        axis([-1+min(P_ee_out(1,:)) 0.5+max(P_ee_out(1,:)) -1+min(P_ee_out(2,:)) 0.5+max(P_ee_out(2,:)) 0 2])
        view(plot_position_3d(1),plot_position_3d(2));%-10, 20);
        drawnow();

         % Salvo frame 
         F_video(j) = getframe;

         % tengo ultima schermata
         if j~=size(state_x,2)
            delete(gca)          
         end
           % get frame for the video
    end
F_video(1)=F_video(2);
end

%% STATIC 3D PLOT (11)
if static_3d_plot
    if exist('robot')~=1
        robot=mobile_manipulator(0.185,0,0.7,pi/2);
        cd Post_Data/PLOTMM/ur_kinematics;
        run compile_ur5kinematics.m
        cd ..
        cd ..
        cd ..
    end
    fig11=figure(11);

    %figure proprietà
    fig11.Position = [0 1200-900 1440 900];

    camlight;
    material metal;
    robot.visualize_mm(state_x(1:9,1).',fig11)
%      plot3(xd(10,:),xd(11,:),xd(12,:),'r')
%         plot3(xd(1,:),xd(2,:),xd(12,:).*0,'k')
%         XX = [const_vec(9,1),const_vec(9,2),const_vec(9,2),const_vec(9,1)];
%         YY = [const_vec(10,1),const_vec(10,1),const_vec(10,2),const_vec(10,2)];
%         ZZ = [0,0,0,0];
%         h2=fill3(XX,YY,ZZ,[0 0 1]);
%         h2.FaceAlpha=0.3;
        
        if visualize_spheres == 1
        spheres_gen(state_x(:,1),T{1})
        end
        xlabel('x')
        ylabel('y')
        zlabel('z')
        grid on
%         axis off
        axis([-1.5+const_vec(9,1) 1.5+const_vec(9,2) -1.5+const_vec(10,1) 1.5+const_vec(10,2) 0 2])
        view(plot_position_3d(1),plot_position_3d(2));%-10, 20);
end
    
%%  VIDEO RECORDING

if record_video == 1 
    if visualize_3d_plot==0
        error('3D PLOT NOT VISUALIZED')
    end
    video_savename = ['Videos/REAL_TEST_'  datestr(now, 'HH-MM dd-mmm-yyyy') '.avi'];
    video= VideoWriter(video_savename);
    myVideo.FrameRate = 10;
    open(video)
    writeVideo(video,F_video)
    close(video)

end

%%  HORIZONS PLOTS (12)
if visualize_horizons == 1
    for aa=1:size(xd,2)

            J_parts(aa,:)=J_comp(:,:,aa);

            for j=1:N+1
                xhee(j)=X_steps.signals.values(10,j,aa);
                yhee(j)=X_steps.signals.values(11,j,aa);
                zhee(j)=X_steps.signals.values(12,j,aa);
                
                xhb(j)=X_steps.signals.values(1,j,aa);
                yhb(j)=X_steps.signals.values(2,j,aa);

                cc(:,j)  = SCA(X_steps.signals.values(1:9,j,aa));
            end

        figure(12)
        
        %%%% EE
        subplot(211)
        plot3(xd(10,:),xd(11,:),xd(12,:))
        hold on
        plot3(xhee,yhee,zhee,'linewidth',2)
        grid on
%         view(0,0)
        % axis([-1 1 -1 1 -1 1])
        xlabel('x')
        ylabel('y')
        zlabel('z')
        drawnow()
        pause(0.05)
        hold off
        
        %%%% BASE
        subplot(212)
        plot3(xd(1,:),xd(2,:),xd(1,:).*0)
        hold on
        plot3(xhb,yhb,xhb.*0,'linewidth',2)
          grid on
%         view(0,0)
        % axis([-1 1 -1 1 -1 1])
        xlabel('x')
        ylabel('y')
        zlabel('z')
        drawnow()
        pause(0.05)
        hold off
        
        
    end
end

%% DATA SAVE

if savedata ==1
    
    if real_sim == 'y'
        Type_sym='REAL_';
    else
        Type_sym='SIM_';
    end
savename = [Type_sym, nometraj '_'  datestr(now, 'HH-MM dd-mmm-yyyy')];
% savename = [Type_sym, nometraj '_N=',num2str(N),'_', datestr(now, 'HH-MM dd-mmm-yyyy')];
matfile = fullfile('Data_saved/', savename);
save(matfile)

end

clc

%% PLOT AGGIUNTIVI (13 e 14)


