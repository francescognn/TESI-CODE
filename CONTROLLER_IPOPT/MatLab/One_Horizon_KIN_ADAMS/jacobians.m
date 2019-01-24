function [Jp,Jo,T] = jacobians(q)
% 
% q=[x y th th1 th2 th3 th4 th5 th6]'

import casadi.*

q = q(1:9);
N=length(q);

[DHtable,A0,CoM_pos,m,In] = parameters_MM(q);

A = rototrasl(DHtable);

T=cell(N+1,1);
T{1}=A0;
%MX.sym('zeta',3,N+1); 
zeta = [A0(1:3,3) zeros(3,N)];
% P=[];%MX.sym('P',3,N+1); 
P = [A0(1:3,4) zeros(3,N)];

for kk = 1:N
    T{kk+1} = T{kk}*A{kk};
    zeta(:,kk+1) = T{kk+1}(1:3,3);
    P(:,kk+1) = T{kk+1}(1:3,4);
end

Jp = cell(N,1);
Jo=Jp;

for i = 2:N+1

    Pl_i = T{i-1}*[CoM_pos(:,i-1);1];
    
     Jpl = zeros(3,N).*q(1,1);
    Jol = Jpl;
    for j=1:i-1
        if j<=2
           Jpl(:,j) = zeta(:,j);
           Jol(:,j) = zeros(3,1);
        else
           Jpl(:,j) = cross(zeta(:,j),Pl_i(1:3) - P(:,j));   
           Jol(:,j) = zeta(:,j);
        end
    end   
    
    Jp{i-1} = Jpl;
    Jo{i-1} = Jol;
    
end




