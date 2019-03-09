clear all
close all
clc

%col=['b','g','r','c'];
col = [0 1 0;  0 0.67 0.33; 0.3010, 0.7450, 0.9330; 0 0 1];

Nvect=[10 15 20 25];

load('SIM_moveFast_N=10.mat')
figure(1)
set(gcf,'color','white')
subplot(2,1,1)
hold on
grid on
plot(xd(1,:),xd(2,:),'r','LineWidth',4)

figure(3)
set(gcf,'color','white')
hold on
grid on
plot((sqrt(diff(xd(1,:).').^2+diff(xd(2,:).').^2)/0.4).','r','LineWidth',2)

figure(2)
set(gcf,'color','white')
subplot(3,2,1)
plot(0:0.4:29.6, xd(4,:),'r','LineWidth',2)
xlabel('Time[s]')
hold on
grid on
subplot(3,2,2)
plot(0:0.4:29.6, xd(5,:),'r','LineWidth',2)
xlabel('Time[s]')
hold on
grid on
subplot(3,2,3)
plot(0:0.4:29.6, xd(6,:),'r','LineWidth',2)
xlabel('Time[s]')
hold on
grid on
subplot(3,2,4)
plot(0:0.4:29.6, xd(7,:),'r','LineWidth',2)
xlabel('Time[s]')
hold on
grid on
subplot(3,2,5)
plot(0:0.4:29.6, xd(8,:),'r','LineWidth',2)
xlabel('Time[s]')
hold on
grid on
subplot(3,2,6)
plot(0:0.4:29.6, xd(9,:),'r','LineWidth',2)
xlabel('Time[s]')
hold on
grid on
%%
for ii=1:length(Nvect)
    clearvars -EXCEPT Nvect ii col
    load(['SIM_moveFast_N=',num2str(Nvect(ii)),'.mat'])
    figure(1)
    subplot(2,1,1)
    hold on
    x_b=state_x(1,:);y_b=state_x(2,:);
    plot(x_b,y_b,'-.','Color',col(ii,:),'LineWidth',2)    
    xlabel('x[m]'); ylabel('y[m]');
    
    subplot(2,1,2)
    error = sqrt((state_x(1,:)-xd(1,:)).^2+(state_x(2,:)-xd(2,:)).^2);
    hold on; grid on;
    plot(0:0.4:29.6,error,'Color',col(ii,:),'LineWidth',1)
    xlabel('Time[s]')
    ylabel('Cartesian error: x_k-x_d_k')
  
    figure(3)
    plot((sqrt(diff(state_x(1,:).').^2+diff(state_x(2,:).').^2)/0.4).','-.','Color',col(ii,:),'LineWidth',2)
    
    figure(2)
    subplot(3,2,1)
    plot(0:0.4:29.6, state_x(4,:),'-.','Color',col(ii,:),'LineWidth',2)
    hold on
    grid on
    title('\theta_1')
    subplot(3,2,2)
    plot(0:0.4:29.6, state_x(5,:),'-.','Color',col(ii,:),'LineWidth',2)
    hold on
    grid on
    title('\theta_2')
    subplot(3,2,3)
    plot(0:0.4:29.6, state_x(6,:),'-.','Color',col(ii,:),'LineWidth',2)
    hold on
    grid on
    title('\theta_3')
    subplot(3,2,4)
    plot(0:0.4:29.6, state_x(7,:),'-.','Color',col(ii,:),'LineWidth',2)
    hold on
    grid on
    title('\theta_4')
    subplot(3,2,5)
    plot(0:0.4:29.6, state_x(8,:),'-.','Color',col(ii,:),'LineWidth',2)
    hold on
    grid on
    title('\theta_5')
    subplot(3,2,6)
    plot(0:0.4:29.6, state_x(9,:),'-.','Color',col(ii,:),'LineWidth',2)
    hold on
    title('\theta_6')
    grid on

   
    clearvars -EXCEPT xd Nvect ii col
end

figure(1)
subplot(211)
legend('REFERENCE','N=10','N=15','N=20','N=25','Location','best')
figure(2)
legend('REFERENCE','N=10','N=15','N=20','N=25','Location','best')
hold off



