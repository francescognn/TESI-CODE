
close all
clc

T  = 0.3;
N  = 20;
Tf = 15; 

Tsample=0.01;

tt=0:T:Tf;

xd=[linspace(0,5,length(tt));0.05*ones(size(tt));zeros(size(tt))];
