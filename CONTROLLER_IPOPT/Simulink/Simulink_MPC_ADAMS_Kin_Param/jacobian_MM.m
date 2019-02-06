function [P_ee,A]= jacobian_MM(q)

 import casadi.*

N=length(q);

[DHtable,A0,CoM_pos,m,In] = parameters_MM(q);

A = rototrasl(DHtable);

T=cell(N+1,1);
T{1}=A0;
zeta=MX(3,N+1); 
zeta(:,1) = A0(1:3,3);
P=MX(3,N+1); 
P(:,1) = A0(1:3,4);

for kk = 1:N
    T{kk+1} = T{kk}*A{kk};
    zeta(:,kk+1) = T{kk+1}(1:3,3);
    P(:,kk+1) = T{kk+1}(1:3,4);
end

% Jp = MX(3,N);
% Jo=Jp;
% 
% for j=1:N
%  if j<=2
%      Jp(:,j) = zeta(:,j);
%      Jo(:,j) = zeros(3,1);
%  else
%      Jp(:,j) = cross(zeta(:,j),P(:,end) - P(:,j));   
%      Jo(:,j) = zeta(:,j);
%  end
% end
% 
% th=q(3);
% G = [cos(th),0;  sin(th),0;  0,1];
% 
% Jp_nh = [Jp(:,1:3)*G , Jp(:,4:end)];
% Jo_nh = [Jo(:,1:3)*G , Jo(:,4:end)];
% 
% J_ee = [Jp_nh;Jo_nh];
% J_ee = J_ee(4:end,4:end);

%  G  = [cos(q(3)) 0; sin(q(3)) 0; 0 1];
G  = zeros(3,2);
A    = [G zeros(3,6); zeros(6,2) eye(6);zeros(6,8)];

% T{end}(1,1) = T11;
% T{end}(1,2) = T12;
% T{end}(1,3) = T13;
% T{end}(2,1) = T21;
% T{end}(2,2) = T22;
% T{end}(2,3) = T23;
% T{end}(3,1) = T31;
% T{end}(3,2) = T32;
% T{end}(3,3) = T33;


th1 = atan2(T{end}(2,3),T{end}(3,3));
c2  = sqrt(T{end}(1,1)^2+T{end}(1,2)^2);
th2 = atan2(-T{end}(1,3),c2);
s1  = sin(th1);
c1  = cos(th1);
th3 = atan2(s1*T{end}(3,1)-c1*T{end}(2,1),c1*T{end}(1,1)-s1*T{end}(3,2));



P_ee = [T{end}(1:3,4);[th1 th2 th3].'];

%  J = [A ;J_ee];

