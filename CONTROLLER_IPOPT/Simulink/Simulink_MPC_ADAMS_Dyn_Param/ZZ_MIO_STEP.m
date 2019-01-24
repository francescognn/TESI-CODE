
close all
clc

T  = 0.2;
N  = 8;
Tf = 15; 

Tsample=0.01;

tt=0:T:Tf;

xd=[linspace(0,5,length(tt));0.25*ones(size(tt));zeros(size(tt))];
