clear all
close all
clc

col=['b','g','r','c'];
Nvect=[10 15 20 25];

load(['SIM_MM_sineOrient_N=10_12-Feb-2019MANIP_FINTA.mat'])
figure(1)
set(gcf,'color','white')
subplot(2,2,[1,2])
hold on
grid on
plot3(xd(10,:),xd(11,:),xd(12,:),'k-','LineWidth',2)

for ii=1:length(Nvect)
    clearvars -EXCEPT Nvect ii col
    load(['SIM_MM_sineOrient_N=',num2str(Nvect(ii)),'_12-Feb-2019NO_MANIP.mat'])
    figure(1)
    subplot(2,2,[1,2])
    hold on
    x_ee=P_ee_out(1,:);y_ee=P_ee_out(2,:);z_ee=P_ee_out(3,:);
    plot3(x_ee,y_ee,z_ee,'.-','Color',col(ii),'LineWidth',1)    
    legend('REFERENCE','N=10','N=15','N=20','N=25','Location','northwest')
    
    subplot(2,2,3)
    hold on; grid on
    plot(error_abs_xyz,'Color',col(ii),'LineWidth',1)
    title('Cartesian error')
    
    subplot(2,2,4)
    hold on; grid on
    if ii == 1 || ii==2
        fatt=0.5;
    else 
        fatt=0.5*0.8;
    end
    plot(T_elapsed_vect(1:length(x_ee))*fatt,'Color',col(ii),'LineWidth',1)
    title('Time Elapsed')
     
    

figure(2)
enne= size(state_x,2)-1;
a_given = [0 diff(u_given(1,1:enne+1))]./dt;
alph_given = [0 diff(u_given(2,1:enne+1))]./dt;

subplot(421)
hold on 
grid on
plot(0:dt:(enne)*dt,a_given,'-','Color',col(ii),'linewidth',1)
plot(ones(1,enne+1)*0.4,'k--','linewidth',1)
plot(ones(1,enne+1)*-0.4,'k--','linewidth',1)
axis([0 enne*dt -0.45 0.45])
title('a_{base} ')


subplot(422)
hold on 
grid on
plot(0:dt:(enne)*dt,alph_given,'-','Color',col(ii),'linewidth',1)
plot(ones(1,enne+1)*0.2,'k--','linewidth',1)
plot(ones(1,enne+1)*-0.2,'k--','linewidth',1)
axis([0 enne*dt -0.25 0.25])
title('\omega')

subplot(423)
hold on
grid on
plot(0:dt:(enne)*dt,u_given(3,1:enne+1),'-','Color',col(ii),'linewidth',1)
plot(ones(1,enne+1)*0.5,'k--','linewidth',1)
plot(ones(1,enne+1)*-0.5,'k--','linewidth',1)
axis([0 enne*dt -0.55 0.55])
title('d\theta_1')
 

subplot(424)
hold on
grid on
plot(0:dt:(enne)*dt,u_given(4,1:enne+1),'-','Color',col(ii),'linewidth',1)
plot(ones(1,enne+1)*0.5,'k--','linewidth',1)
plot(ones(1,enne+1)*-0.5,'k--','linewidth',1)
axis([0 enne*dt -0.55 0.55])
title('d\theta_2')
 

subplot(425)
hold on
grid on
plot(0:dt:(enne)*dt,u_given(5,1:enne+1),'-','Color',col(ii),'linewidth',1)
plot(ones(1,enne+1)*0.5,'k--','linewidth',1)
plot(ones(1,enne+1)*-0.5,'k--','linewidth',1)
axis([0 enne*dt -0.55 0.55])
title('d\theta_3')
 

subplot(426)
hold on
grid on
plot(0:dt:(enne)*dt,u_given(6,1:enne+1),'-','Color',col(ii),'linewidth',1)
plot(ones(1,enne+1)*0.5,'k--','linewidth',1)
plot(ones(1,enne+1)*-0.5,'k--','linewidth',1)
axis([0 enne*dt -0.55 0.55])
title('d\theta_4')
 

subplot(427)
hold on
grid on
plot(0:dt:(enne)*dt,u_given(7,1:enne+1),'-','Color',col(ii),'linewidth',1)
plot(ones(1,enne+1)*0.5,'k--','linewidth',1)
plot(ones(1,enne+1)*-0.5,'k--','linewidth',1)
axis([0 enne*dt -0.55 0.55])
title('d\theta_5')


subplot(428)
hold on
grid on
plot(0:dt:(enne)*dt,u_given(8,1:enne+1),'-','Color',col(ii),'linewidth',1)
plot(ones(1,enne+1)*0.5,'k--','linewidth',1)
plot(ones(1,enne+1)*-0.5,'k--','linewidth',1)
axis([0 enne*dt -0.55 0.55])
title('d\theta_6')

end
 