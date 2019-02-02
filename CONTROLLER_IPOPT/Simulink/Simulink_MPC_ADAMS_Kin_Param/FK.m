function [T,R,P]= FK(q)
%   FORWARD KINEMATICS FUNCTION
%T = cell of DH rototranslation matrices
%R = cell of DH rotation matrices
%P = matrix of the position of the centres of the DH frames
N=length(q);

[DHtable,A0,CoM_pos,m,In] = parameters_MM(q);

A = rototrasl(DHtable);

T=cell(N+1,1);R = T;
T{1}=A0;
P(:,1) = A0(1:3,4);
P=sym(P); %just for symbolic things
R{1}=T{1}(1:3,1:3);
for kk = 1:N
    T{kk+1} = T{kk}*A{kk};
    P(:,kk+1) = T{kk+1}(1:3,4);
    R{kk+1}=T{kk+1}(1:3,1:3);
end


