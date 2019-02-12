clear all
close all
clc 

addpath('Data_saved','Input_Traj','Post_Data');
Ni =10;
T=0.25;
Tsampling=0.01;
what = 'MM';
type = 'sine_orient';

[N,T,Tsample,t_total,xd,nometraj,initialize_starting_point,x0_val]=input_traj(Ni,T,Tsampling,what,type,0);

StopTime = t_total+2*T;
sim('MPC_mobile_base_KIN_IPOPT_fast','SimulationMode','rapid','StopTime', num2str(StopTime));

DATA_SIM