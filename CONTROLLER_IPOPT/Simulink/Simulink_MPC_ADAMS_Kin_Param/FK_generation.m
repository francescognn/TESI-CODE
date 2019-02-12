% x1=q(4);x2=q(5);x3=q(6);x4=q(7);%x5=q(8);x6=q(9);
import casadi.*

q = MX.sym('q',18,1);

N=length(q);

z_man_base = 0.7;    % height of the manipulator base from the ground
x_man_base = 0.185;  % distance of the manipulator base from the center of the base 
                   % along its longitudinal axis 
y_man_base = 0; % distance of the manipulator base from the center of the base 
                   % along its trasversal axis 

a_man=sqrt(x_man_base^2 + y_man_base^2);
d_man=z_man_base;
th_man=atan2(y_man_base,x_man_base);

                   
X=q(1); Y=q(2); TH=q(3); th1=q(4); th2=q(5); th3=q(6); th4=q(7); th5=q(8); th6=q(9); 

DHTABLE = [ 0         , pi/2  , X       , pi/2;
            0         , pi/2 , Y       , pi/2;
            a_man     , 0     , d_man   , TH+th_man;
            0         , pi/2  , 0.08916 , th1-th_man+pi/2;
           -0.425     , 0     , 0       , th2;
           -0.39225   , 0     , 0       , th3;
            0         , pi/2  , 0.10915 , th4;
            0         , -pi/2 , 0.09465 , th5;
            0         , 0     , 0.216  , th6];

A0    = [0 , 0 , 1 , 0;
         1 , 0 , 0 , 0;
         0 , 1 , 0 , 0;
         0 , 0 , 0 , 1];

N = size(DHTABLE,1);

A = MX.zeros((N+1)*4,4);
rotT=MX.zeros((N+1)*4,4);
rotT(1:4,:)=A0;
zeta=MX.sym('zeta',3,N+1); 
zeta(:,1) = A0(1:3,3);
P=MX.sym('P',3,N+1); 
P(:,1) = A0(1:3,4);

j=5;
k=1;
for i = 1:N
    
    a = DHTABLE(i,1);
    alp = DHTABLE(i,2);    
    d = DHTABLE(i,3);
    th = DHTABLE(i,4);
    A(j:j+3,:) =  [ cos(th) -sin(th)*cos(alp)  sin(th)*sin(alp) a*cos(th);
                    sin(th)  cos(th)*cos(alp) -cos(th)*sin(alp) a*sin(th);
                           0          sin(alp)          cos(alp)        d;
                           0            0                 0             1];   
    rotT(j:j+3,:) = rotT(k:k+3,:)*A(j:j+3,:);
    zeta(:,i+1) = rotT(j:j+2,3);
    P(:,i+1) = rotT(j:j+2,4);    
    
    j=j+4;
    k=k+4;
end


P_ee    = [rotT(end-3:end-1,4)];
Psi_ee  = [rotT(end-3:end-1,1),rotT(end-3:end-1,3)];
 
FK_casadi=Function('FK_casadi',{q},{P_ee,Psi_ee,rotT});

% FK_mex_casadi.generate('FK_mex_casadi',struct('mex', true, 'main', true));
