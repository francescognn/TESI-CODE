clear all
close all
clc

load('SIM_PICK_UP_11-49 24-Jan-2019.mat')
clearvars -EXCEPT T_elapsed_vect
T_elapsed_vect1=T_elapsed_vect;
clear T_elapsed_vect

load('SIM_PICK_UP_12-13 24-Jan-2019.mat')
clearvars -EXCEPT T_elapsed_vect T_elapsed_vect1
T_elapsed_vect2=T_elapsed_vect;
clear T_elapsed_vect

figure(1)
plot(T_elapsed_vect1,'color','b')
grid on
hold on
plot(T_elapsed_vect2,'color','r')
legend()

plot(ones(1,size(T_elapsed_vect1,1)).*mean(T_elapsed_vect1),'color','b','linewidth',2)
grid on
hold on
plot(ones(1,size(T_elapsed_vect2,1)).*mean(T_elapsed_vect2),'color','r','linewidth',2)
legend('tol=1e-8','tol=1e-5','MEAN tol=1e-8','MEAN tol=1e-3')
