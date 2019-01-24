function [u]=tretratti(t,Ttot,p,dt)
%

l1=0.1;
l2=0.2;
% A=p(3);
% D=p(4);
% u0=p(5);

% legge di moto TreTratti (acc.costante)
%
% t tempo per cui calcolare la legge
% T tempo di azionamento
% S0 posizione iniziale
% dS ampiezza movimento
% l1 lambda1 (durata 1^ intervallo/T  0<l1<1)
% l3 lambda3 (durata 3^ intervallo/T  0<l3<1)
% 
% si assume Vini=Vfin=0
%

T1=Ttot*l1;
T2=Ttot*(1-l1-l2);
Ta=T1+T2;

if t<T1
   u=p(3)+p(1)*t;
else
    if t<Ta
      u=p(3)+p(1)*(Ttot-dt)*l1;
else
      u=p(3)+p(1)*(Ttot-dt)*l1-p(2)*(t-Ta);
   end
end

           