function [T,P] = TeP(q)

q = q(1:9);
N=length(q);

[DHtable,A0,CoM_pos,m,In] = parameters_MM(q);

A = rototrasl(DHtable);

T=cell(N+1,1);
T{1}=A0;
zeta=(zeros(3,N+1)); 
zeta(:,1) = A0(1:3,3);
P=(zeros(3,N+1)); 
P(:,1) = A0(1:3,4);
for kk = 1:N
    T{kk+1} = T{kk}*A{kk};
    zeta(:,kk+1) = T{kk+1}(1:3,3);
    P(:,kk+1) = T{kk+1}(1:3,4);
end





