clear all 
close all
clc

syms x dx ddx y dy ddy theta dtheta ddtheta t M J xc yc

L=0.5*M*(dx-yc*dtheta*cos(theta)-xc*dtheta*sin(theta))^2 ... 
    +0.5*M*(dy+xc*dtheta*cos(theta)-yc*dtheta*sin(theta))^2+0.5*J*dtheta^2;

Equation=Lagrange(L,[x,dx,ddx,y,dy,ddy,theta,dtheta,ddtheta]);


