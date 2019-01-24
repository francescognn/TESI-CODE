clear 
close
clc

T = 0.1; % tempo azionamento
N = 60; % n.punti della legge
u0=rand(2,1); % valore iniziale


% [cost] = costeval(p,u0,x0);
% 
% p=[a0 vc af dt a b c d];
% 
% x=system (x0,u0,p);

% ovvero:
% y = f(p,dt,N)

p=[0.4885    1.6185    0.4617   -0.0789   -0.0275    0.4570   -0.0345 ];
% p=zeros(1,9);
% p(1,5)=3;
%p=[l1 l3 A D a b c];
x0=zeros(1,3);
x=x0;

for i=1:N
    
    tt(i)=T*(i-1);
   
    u(1,i) = tretratti(tt(i),N*T,p,T);
    u(2,i) = p(4)*tt(i)^3+p(5)*tt(i)^2+p(6)*tt(i)+p(7);

    x(i+1,1)=x(i,1)+T*u(1,i)*cos(x(i,3));
    x(i+1,2)=x(i,2)+T*u(1,i)*sin(x(i,3));
    x(i+1,3)=x(i,3)+T*u(2,i);
end



  



% y(k+1)=y(k)+


% cost = (xd-x)*Q*(xd-x)


