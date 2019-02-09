savedata           = 0;
visualize_plot     = 0;
visualize_3d_plot  = 1;
plot_position_3d   = [-40,10]; % AZ,EL
visualize_spheres  = 0;
record_video       = 0;

%% DIMENSION SET

state_x=state_x_sim.';
if exist('state_qp')==1
state_qp=state_qp(:,1:6).';
end
u_given=u_given.';


if size(state_x,2)>=size(xd,2)
    state_x=state_x(:,1:size(xd,2));
else
    xd=xd(:,1:size(state_x,2));
end

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

plot(u_given(1:2,:))
grid on
legend('v','omega')
title('Base control action')

%% BASE MOTION (7)

figure(7)
plot(state_x(1,:),state_x(2,:))
grid on
title('Base motion XY')

end

%% >>>> 3D PLOTS <<<<
if visualize_3d_plot == 1

%% 3D PLOT OF MM (8)  

fig8=figure(8);

%figure proprietà
fig8.Position = [0 1200-900 1440 900];
camlight;
material metal;

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

        hold on
        robot.visualize_mm(state_x(1:9,j).',fig8);
        plot3(xd(10,:),xd(11,:),xd(12,:),'r')
        plot3(xd(1,:),xd(2,:),xd(12,:).*0,'k')
        spheres_gen(state_x(:,j),T{j})

        xlabel('x')
        ylabel('y')
        zlabel('z')
        grid on

        axis([-1.5+min(state_x(10,:)) 1.5+max(state_x(10,:)) -1.5+min(state_x(11,:)) 1.5+max(state_x(11,:)) 0 2])
        view(plot_position_3d(1),plot_position_3d(2));%-10, 20);
        drawnow();

         % Salvo frame 
         F_video(j) = getframe;

         % tengo iltima schermata
         if j~=size(state_x,2)
            delete(gca)          
         end
           % get frame for the video
    end

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
    
%% DATA SAVE

if savedata ==1

savename = [nometraj '_'  datestr(now, 'HH-MM dd-mmm-yyyy')];
matfile = fullfile('Data_saved/', savename);
save(matfile)

end
