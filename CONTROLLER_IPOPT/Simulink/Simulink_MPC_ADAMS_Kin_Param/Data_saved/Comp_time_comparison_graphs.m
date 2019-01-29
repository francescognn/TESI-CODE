aclear all
close all
clc

k=1;
for i=5:5:25
    
    name1=['SIM_traj_raffronto_NO_param_', num2str(i), '.mat'];
    load(name1)
    clearvars -EXCEPT T_elapsed_vect k i Tmean_param  Tmean_no_param Tmean_param_vinc Tmean_no_param_vinc
    Tmean_no_param(k)=mean(T_elapsed_vect);
    clear T_elapsed_vect
    
    name2=['SIM_traj_raffronto_param_', num2str(i), '.mat'];
    load(name2)
    clearvars -EXCEPT T_elapsed_vect  k i Tmean_param  Tmean_no_param Tmean_param_vinc Tmean_no_param_vinc
    Tmean_param(k)=mean(T_elapsed_vect);
    clear T_elapsed_vect
    
    name1v=['SIM_traj_raffronto_NO_param_vinc_', num2str(i), '.mat'];
    load(name1v)
    clearvars -EXCEPT T_elapsed_vect k i Tmean_param  Tmean_no_param Tmean_param_vinc Tmean_no_param_vinc
    Tmean_no_param_vinc(k)=mean(T_elapsed_vect);
    clear T_elapsed_vect
    
    name2v=['SIM_traj_raffronto_param_vinc_', num2str(i), '.mat'];
    load(name2v)
    clearvars -EXCEPT T_elapsed_vect  k i Tmean_param  Tmean_no_param Tmean_param_vinc Tmean_no_param_vinc
    Tmean_param_vinc(k)=mean(T_elapsed_vect);
    clear T_elapsed_vect
    
    
    
    k=k+1;
    
end
Nvect=[5 10 15 20 25];

figure(1)

plot(Nvect,Tmean_no_param,'*','MarkerSize',8,'MarkerEdgeColor','b','MarkerFaceColor','b')
hold on
grid on
plot(Nvect,Tmean_param,'s','MarkerSize',8,'MarkerEdgeColor','r','MarkerFaceColor','r')
lsline
title('Mean Optimization times')
legend('PIECEWISE','PARAMETRIZED')
xlabel('Horizon length (N)')
ylabel('Time [s]')

figure(2)

plot(Nvect,Tmean_param_vinc,'*','MarkerSize',8,'MarkerEdgeColor','b','MarkerFaceColor','b')
hold on
grid on
plot(Nvect,Tmean_param,'s','MarkerSize',8,'MarkerEdgeColor','r','MarkerFaceColor','r')
lsline
title('Mean Optimization times with parametrization')
legend('CONSTRAINED','NOT CONSTRAINED')
xlabel('Horizon length (N)')
ylabel('Time [s]')

figure(3)

plot(Nvect,Tmean_no_param_vinc,'*','MarkerSize',8,'MarkerEdgeColor','b','MarkerFaceColor','b')
hold on
grid on
plot(Nvect,Tmean_no_param,'s','MarkerSize',8,'MarkerEdgeColor','r','MarkerFaceColor','r')
lsline
title('Mean Optimization times with piecewise')
legend('CONSTRAINED','NOT CONSTRAINED')
xlabel('Horizon length (N)')
ylabel('Time [s]')

figure(4)

plot(Nvect,Tmean_no_param_vinc,'*','MarkerSize',8,'MarkerEdgeColor','b','MarkerFaceColor','b')
hold on
grid on
plot(Nvect,Tmean_param,'s','MarkerSize',8,'MarkerEdgeColor','r','MarkerFaceColor','r')
lsline
title('Mean Optimization times comparison')
legend('CLASSICAL MPC','PARAMETRIZED CONSTRAINTS-FREE')
xlabel('Horizon length (N)')
ylabel('Time [s]')




