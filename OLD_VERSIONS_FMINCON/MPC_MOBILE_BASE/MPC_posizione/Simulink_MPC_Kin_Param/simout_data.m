 
close 

clc

for i=1:size(simout.Data,3)
    i=real(i);
    
    x(i)=simout.Data(:,1,i);
    y(i)=simout.Data(:,2,i);
    th(i)=simout.Data(:,3,i);
    end

figure
plot(x,y)
set(gcf,'color','white')
hold on
% plot(xd(1,1:N),xd(2,1:N),'o')
% hold on;
grid on;
plot(xd(1,:),xd(2,:))
legend('Simulated Trajectory','Real Trajectory')
title('X-Y trajectory')

finpos=[ x(length(xd)) y(length(xd)) th(length(xd))];

err_fin_pos=xd(:,end).'-finpos;

errore=[x(1:length(xd)).' y(1:length(xd)).' th(1:length(xd)).']-xd.';

figure
set(gcf,'color','white')
plot(tt,errore)
legend('x','y','theta')
title('Errors (m)')
grid on

figure
set(gcf,'color','white')
plot(t_elapsed)
ylabel('T elapsed')
xlabel('Step')
grid on



savefile=['MPC_kin_Param_N=',num2str(N),' traj_ultimo.mat'];

save(savefile);


