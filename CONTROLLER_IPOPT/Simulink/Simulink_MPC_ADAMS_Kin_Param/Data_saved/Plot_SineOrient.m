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
    plot(T_elapsed_vect(1:length(x_ee))*0.5,'Color',col(ii),'LineWidth',1)
    title('Time Elapsed')
     
    clearvars -EXCEPT xd Nvect Distvect ii j col
end
hold off
clear all

