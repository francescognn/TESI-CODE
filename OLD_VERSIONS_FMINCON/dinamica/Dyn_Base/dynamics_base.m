clear
close all

syms Mbase I1 I2 I3 h
syms x_cog_base y_cog_base
syms X Y TH Xp Yp THp
% x_cog_base =0; y_cog_base = 0;

q = [X Y TH];
qp = [Xp Yp THp];

% %TO BE DEFINED:
% z_man_base = 1;    % height of the manipulator base from the ground
% x_man_base = 0.2;  % distance of the manipulator base from the center of the base 
%                    % along its longitudinal axis 
% y_man_base = 0.05; % distance of the manipulator base from the center of the base 
                   % along its trasversal axis 
                   
syms x_man_base y_man_base
a_man =sqrt(x_man_base^2 + y_man_base^2);
th_const=atan2(y_man_base,x_man_base);

N = length(q);
g = 9.81;

m = [0, 0, Mbase];

In=cell(N,1);
In{1}=zeros(3);
In{2}=zeros(3);
In{3}=diag([I1,I2,I3]);

%%%%%%%%%%%%%%%%%%%%%%%%
%%%    KINEMATICS    %%%
%%%%%%%%%%%%%%%%%%%%%%%%

DHtable = [ 0  ,  pi/2  , X  , pi/2;
            0  ,  pi/2  , Y  , pi/2;
            0  ,  0     , h  , TH];

A = rototrasl(DHtable);
        
A0 = [0 , 0 , 1 , 0;
      1 , 0 , 0 , 0;
      0 , 1 , 0 , 0;
      0 , 0 , 0 , 1];
     
T=cell(N,1);
T{1}=A0*A{1};
for kk = 2:N
    T{kk} = T{kk-1}*A{kk};
end
Tx = T{1};Ty = T{2};Tth = T{3};

z0 = A0(1:3,3); p0 = A0(1:3,4);
zx = Tx(1:3,3); px = Tx(1:3,4);
zy = Ty(1:3,3); py = Ty(1:3,4);
zth = Tth(1:3,3); pth = Tth(1:3,4);
zeta = [z0,zx,zy,zth]; 
P = [p0,px,py,pth];

%%%%%%%%%%%%%%%%%%%%%%
%%%    DYNAMICS    %%%
%%%%%%%%%%%%%%%%%%%%%%

CoM_pos= [0,0,0;
          0,0,0; 
          x_cog_base,y_cog_base,-h/2]; 
      
CoM_pos = CoM_pos.';

Jpl=cell(N,1);
Jol=cell(N,1);

B=zeros(N);
C=sym(zeros(N));
gq=sym(zeros(N,1));

% Building B Matrix and Jacobians

for i = 2:N+1
  
  T_i = T{i-1};
  R_i = T_i(1:3,1:3);
  Pl_i = T{i-1}*[CoM_pos(:,i-1);1];
  
  Jp = zeros(3,N);
  Jp = sym(Jp); Jo=Jp;
  
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
B =(vpa(B));


% Building C Matrix

dB = cell(N,1);
dB_sum=zeros(N);
for ii = 1:N
    dB{ii} = diff(B,q(ii));
    dB_sum = dB_sum+dB{ii};
end
hijk=sym(zeros(1,3));
for j=1:N
    for i=1:N
        for k=1:N
            dBk=dB{k};
            dBi=dB{i};
            hijk(k)= dBk(i,j)-0.5*dBi(j,k);
        end
        C(i,j)=hijk*qp.';
    end
end

% Building gravitationalx vector

for i=1:N

    hg=T{i}*[CoM_pos(:,i); 1];
    Peso=m(i)*g*hg(3);
    gq(i,1)=diff(Peso,q(i));
    
end



     
         