
savedata           = 0;
visualize_plot     = 1;
visualize_3d_plot  = 0;
plot_position_3d   = [40,10]; % AZ,EL
visualize_spheres  = 0;
record_video       = 0;
visualize_horizons = 0;
static_3d_plot     = 0;

prompt             = 'Is this a Real test? (y/n)  ';
real_sim           = 'n';%input(prompt,'s');


%% DIMENSION SET

state_x=state_x_sim.';
if exist('state_qp')==1
state_qp=state_qp(:,1:6).';
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

plot3(P_ee_out(1,:),P_ee_out(2,:),P_ee_out(3,:))
hold on
grid on
plot3(xd(10,:),xd(11,:),xd(12,:),'*')
xlabel('x')
ylabel('y')
zlabel('z')
legend('robot','desired')
axis equal

%% ERROR ABS XYZ PLOT (2)


figure(2)
plot(error_abs_xyz)
grid on
title('error xyz')

%% ARM JOINTS VELOCITIES PLOTS (3) 
if exist('state_qp')==1
    
figure(3)
subplot(321)
plot(state_qp(1,:))
hold on
grid on
plot(u_given(3,:))
title('thetap1')
legend('meas','giv')

subplot(322)
plot(state_qp(2,:))
hold on
grid on
plot(u_given(4,:))
title('thetap2')
legend('meas','giv')

subplot(323)
plot(state_qp(3,:))
hold on
grid on
plot(u_given(5,:))
title('thetap3')
legend('meas','giv')

subplot(324)
plot(state_qp(4,:))
hold on
grid on
plot(u_given(6,:))
title('thetap4')
legend('meas','giv')

subplot(325)
plot(state_qp(5,:))
hold on
grid on
plot(u_given(7,:))
title('thetap5')
legend('meas','giv')

subplot(326)
plot(state_qp(6,:))
hold on
grid on
plot(u_given(8,:))
title('thetap6')
legend('meas','giv')
end

warning off

%% ARM JOINTS VELOCITIES ERROR (4)
if exist('state_qp')==1
    
aj_error=state_qp-u_given(3:end,:);

figure(4)
subplot(321)
plot(aj_error(1,:))
grid on
title('error thetap1')

subplot(322)
plot(aj_error(2,:))
grid on
title('error thetap2')

subplot(323)
plot(aj_error(3,:))
grid on
title('error thetap3')

subplot(324)
plot(aj_error(4,:))
grid on
title('error thetap4')

subplot(325)
plot(aj_error(4,:))
grid on
title('error thetap5')

subplot(326)
plot(aj_error(6,:))
grid on
title('error thetap6')

end

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
plot(state_x(1,:),state_x(2,:))
grid on
title('Base motion XY')

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
stem(1,sum(J_parts(:,2))/sum(J_parts(:,1))*100,'linewidth',2,'MarkerSize',3)
hold on
stem(2,sum(J_parts(:,3))/sum(J_parts(:,1))*100,'linewidth',2,'MarkerSize',3)
stem(3,sum(J_parts(:,4))/sum(J_parts(:,1))*100,'linewidth',2,'MarkerSize',3)
stem(4,sum(J_parts(:,5))/sum(J_parts(:,1))*100,'linewidth',2,'MarkerSize',3)
stem(5,sum(J_parts(:,6))/sum(J_parts(:,1))*100,'linewidth',2,'MarkerSize',3)
xlabel('cost function components')
ylabel('% of the total cost')

%% MANIPULABILIY INDEX (9)

figure(9)
plot(man_index)
grid on
title('Manipulability Index')

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

        axis([-1.5+min(state_x(10,:)) 1.5+max(state_x(10,:)) -1.5+min(state_x(11,:)) 1.5+max(state_x(11,:)) 0 2])
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
     plot3(xd(10,:),xd(11,:),xd(12,:),'r')
        plot3(xd(1,:),xd(2,:),xd(12,:).*0,'k')
        if visualize_spheres == 1
        spheres_gen(state_x(:,1),T{1})
        end
        xlabel('x')
        ylabel('y')
        zlabel('z')
        grid on
        axis([-1.5+min(state_x(10,:)) 1.5+max(state_x(10,:)) -1.5+min(state_x(11,:)) 1.5+max(state_x(11,:)) 0 2])
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

                cc(:,j)  = SCA(X_steps.signals.values(1:9,j,aa));
            end

        figure(12)
        plot3(xd(10,:),xd(11,:),xd(12,:))
        hold on
        plot3(xhee,yhee,zhee,'linewidth',2)
        grid on
        view(0,0)
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
% savename = [Type_sym, nometraj '_N=',num2str(N),'_'  datestr(now, 'dd-mmm-yyyy'),'MANIP_FINTA'];
matfile = fullfile('Data_saved/', savename);
save(matfile)

end

clc
