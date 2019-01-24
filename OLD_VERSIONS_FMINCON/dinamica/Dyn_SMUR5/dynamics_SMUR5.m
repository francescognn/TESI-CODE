function [B,C,gq]=dynamics_SMUR5(q,qp,Mbase,x_cog_base,y_cog_base,Ibase)
% all the parameters of UR5 are already known from previous experiments

%TO BE DEFINED:
z_man_base = 1;    % height of the manipulator base from the ground
x_man_base = 0.2;  % distance of the manipulator base from the center of the base 
                   % along its longitudinal axis 
y_man_base = 0.05; % distance of the manipulator base from the center of the base 
                   % along its trasversal axis 
             

a_man=sqrt(x_man_base^2 + y_man_base^2);
d_man=sqrt(z_man_base^2 + y_man_base^2);
                   
I1=Ibase(1);I2=Ibase(2);I3=Ibase(3);

X=q(1); Y=q(2); TH=q(3); th1=q(4); th2=q(5); th3=q(6); th4=q(7); th5=q(8); th6=q(9); 

N = length(q);
g = 9.81;

m = [0, Mbase, 0, 3.7 , 8.393 , 2.33 , 1.219 , 1.219 , 0.1879];

In=cell(N,1);
In{1}=zeros(3);
In{2}=zeros(3);
In{3}=diag([I1,I2,I3]);
In{4}=diag([0.0084 0.0064 0.0084]);
In{5}=diag([0.0078 0.21 0.21]);
In{6}=diag([0.0016 0.0462 0.0462]);
In{7}=diag([0.0016 0.0016 0.0009]);
In{8}=diag([0.0016 0.0016 0.0009]);
In{9}=diag([0.0001 0.0001 0.0001]);


%%%%%%%%%%%%%%%%%%%%%%%%
%%%    KINEMATICS    %%%
%%%%%%%%%%%%%%%%%%%%%%%%

DHtable = [ 0     , pi/2  , X       , pi/2;
            0     , -pi/2 , Y       , -pi/2;
            a_man , 0     , d_man   , TH;
            0     , pi/2  , 0.08916 , th1;
            0.425 , 0     , 0       , th2;
            0.39225, 0    , 0       , th3;
            0     , pi/2  , 0.10915 , th4;
            0     , -pi/2 , 0.09456 , th5;
            0     , 0     , 0.0823  , th6];

A = rototrasl(DHtable);
        
Abase = [0 , 0 , 1 , 0;
         1 , 0 , 0 , 0;
         0 , 1 , 0 , 0;
         0 , 0 , 0 , 1];
     
T=cell(N,1);
T{1}=Abase*A{1};
for kk = 2:N
    T{kk} = T{kk-1}*A{kk};
end
Tx = T{1};Ty = T{2};Tth = T{3};T1 = T{4};T2 = T{5};T3 = T{6};T4 = T{7};T5 = T{8};T6 = T{9};

z0 = Abase(1:3,3); p0 = Abase(1:3,4);
zx = Tx(1:3,3); px = Tx(1:3,4);
zy = Ty(1:3,3); py = Ty(1:3,4);
zth = Tth(1:3,3); pth = Tth(1:3,4);
z1 = T1(1:3,3); p1 = T1(1:3,4);
z2 = T2(1:3,3); p2 = T2(1:3,4);
z3 = T3(1:3,3); p3 = T3(1:3,4);
z4 = T4(1:3,3); p4 = T4(1:3,4);
z5 = T5(1:3,3); p5 = T5(1:3,4);
z6 = T6(1:3,3); p6 = T6(1:3,4);
zeta = [z0,zx,zy,zth,z1,z2,z3,z4,z5,z6]; 
P = [p0,px,py,pth,p1,p2,p3,p4,p5,p6];

%%%%%%%%%%%%%%%%%%%%%%
%%%    DYNAMICS    %%%
%%%%%%%%%%%%%%%%%%%%%%

CoM_pos= [0,0,0;
          0,0,0; 
          (x_cog_base-x_man_base),(y_cog_base-y_man_base),-z_man_base/2;
          0, -0.02561, 0.00193;
          0.2125, 0, 0.11336;
          0.15, 0.0, 0.0265;
          0, -0.0018, 0.01634;
          0, 0.0018,0.01634;
          0, 0, -0.001159]; 
      
CoM_pos = CoM_pos.';

Jpl=cell(N,1);
Jol=cell(N,1);

B=zeros(N);
C=zeros(N);
gq=zeros(N,1);

% Building B Matrix and Jacobians

for i = 2:N+1
  
  T_i = T{i-1};
  R_i = T_i(1:3,1:3);
  Pl_i = T{i-1}*[CoM_pos(:,i-1);1];
  
  Jp = zeros(3,N);
  Jo=Jp;
  
  for j=1:i-1
     if j<=2
         Jp(:,j) = zeta(:,j);
         Jo(:,j) = zeros(3,1);
     else
         Jp(:,j) = cross(zeta(:,j),Pl_i(1:3) - P(:,j));   
         Jo(:,j) = zeta(:,j);
     end
  end
  
  Jpl{i-1}=Jp;       
  Jol{i-1}=Jo;       

  I=In{i-1};
  B=B+m(i-1)*(Jp.'*Jp)+Jo.'*R_i*I*R_i.'*Jo;
  
end
B =double(B);
%%

% Building C Matrix

dB_sum=zeros(N);
dB = cell(9,1);
for ii = 1:N
    dB{ii} = diff(B,q(ii));
    dB_sum = dB_sum+dB{ii}.*qp(ii);
end

for i=1:N
    for j=1:N
       dBj = dB{i};
       dBi = dB{j};
       C(i,j) = 0.5*(dB_sum(i,j)+sum((dBj(i,:).*qp).')-sum(dBi(:,j).*qp.'));
    end
end
C=double(C);

%%

% Building gravity vector

for i=1:9

    hg=T{i}*[CoM_pos(:,i); 1];
    Peso=m(i,1)*g*hg(3);
    gq(i,1)=diff(Peso(i,1),q(i));
    
end
gq = double(gq);

end



     
         