clear all
close all
clc

nometraj='STEP';

T  = 0.5;
N  = 15;
Tf = 15; 

Tsample=0.01;

tt=0:T:Tf;

xd=[linspace(0,5,length(tt));0.5*ones(size(tt));zeros(6,length(tt));zeros(size(tt));linspace(0,5,length(tt));0.5*ones(size(tt));1.6*ones(size(tt));zeros(3,length(tt))];

% CALCULATING X0

q0=[0 0 0 pi/4 pi pi/4 pi/4 pi/4 pi/4]';
[Pee0,A]=jacobian_MM(q0);

x0 = [q0;Pee0];
