clear 
close
clc


T=0.4;
N=10;
Ttot=30;
Tsample=0.01;
p_0=zeros(1,7);


tt=0:0.4:Ttot;

x=0.2*tt;
y=2+0.2*tt;

xpd=0.5;
ypd=0.5;

th=atan2(ypd,xpd)*ones(1,length(x));

xd=[x; y; th];

%plot(xd(1,:),xd(2,:),)
