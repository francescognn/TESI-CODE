function [T]= jacobian_MM_simple_val(q)

N=length(q);

[DHtable,A0,CoM_pos,m,In] = parameters_MM(q);

A = rototrasl(DHtable);

T=zeros(4,4);
T=A0;
zeta=zeros(3,N+1); 
zeta(:,1) = A0(1:3,3);
P=zeros(3,N+1); 
P(:,1) = A0(1:3,4);

for kk = 1:N
    T = T*A{kk};
   
end

