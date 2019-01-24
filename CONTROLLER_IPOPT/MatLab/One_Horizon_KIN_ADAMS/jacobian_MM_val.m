function [J,T]= jacobian_MM_val(q)

 import casadi.*

N=length(q);

[DHtable,A0,CoM_pos,m,In] = parameters_MM(q);

A = rototrasl(DHtable);

T=cell(N+1,1);
T{1}=A0;
zeta=zeros(3,N+1); 
zeta(:,1) = A0(1:3,3);
P=zeros(3,N+1); 
P(:,1) = A0(1:3,4);

for kk = 1:N
    T{kk+1} = T{kk}*A{kk};
    zeta(:,kk+1) = T{kk+1}(1:3,3);
    P(:,kk+1) = T{kk+1}(1:3,4);
end

Jp = zeros(3,N);
Jo=Jp;

for j=1:N
 if j<=2
     Jp(:,j) = zeta(:,j);
     Jo(:,j) = zeros(3,1);
 else
     Jp(:,j) = cross(zeta(:,j),P(:,end) - P(:,j));   
     Jo(:,j) = zeta(:,j);
 end
end

th=q(3);
G = [cos(th),0;  sin(th),0;  0,1];

Jp_nh = [Jp(:,1:3)*G , Jp(:,4:end)];
Jo_nh = [Jo(:,1:3)*G , Jo(:,4:end)];

J_ee = [Jp_nh;Jo_nh];

G = [cos(q(3)) 0; sin(q(3)) 0; 0 1];
A = [G zeros(3,6); zeros(6,2) eye(6)];

 J=[A ;J_ee];

