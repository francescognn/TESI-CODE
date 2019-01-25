function [DHtable,Abase,CoM_pos,m,In] = parameters_MM(q)
N=9;
if length(q) ~= N
    error('q has the wrong number of elements');
end

%TO BE DEFINED:
z_man_base = 1;    % height of the manipulator base from the ground
x_man_base = 0;  % distance of the manipulator base from the center of the base 
                   % along its longitudinal axis 
y_man_base = 0; % distance of the manipulator base from the center of the base 
                   % along its trasversal axis 
Mbase = 100;       % mass of the base
I1 = 10;           % inertia values 
I2 = 10;
I3 = 10;
             

a_man=sqrt(x_man_base^2 + y_man_base^2);
d_man=z_man_base;
th_man=atan2(y_man_base,x_man_base);

                   
X=q(1); Y=q(2); TH=q(3); th1=q(4); th2=q(5); th3=q(6); th4=q(7); th5=q(8); th6=q(9); 

DHtable = [ 0         , pi/2  , X       , pi/2;
            0         , pi/2 , Y       , pi/2;
            a_man     , 0     , d_man   , TH+th_man;
            0         , pi/2  , 0.08916 , th1-th_man;
           -0.425     , 0     , 0       , th2;
           -0.39225   , 0     , 0       , th3;
            0         , pi/2  , 0.10915 , th4;
            0         , -pi/2 , 0.09465 , th5;
            0         , 0     , 0.0823  , th6];

Abase = [0 , 0 , 1 , 0;
         1 , 0 , 0 , 0;
         0 , 1 , 0 , 0;
         0 , 0 , 0 , 1];
     

m = [0, 0, Mbase, 3.7 , 8.393 , 2.33 , 1.219 , 1.219 , 0.1879];

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


CoM_pos= [0,0,0;
          0,0,0; 
          -x_man_base,-y_man_base,-z_man_base/2;
          0, -0.02561, 0.00193;
          0.2125, 0, 0.11336;
          0.15, 0.0, 0.0265;
          0, -0.0018, 0.01634;
          0, 0.0018,0.01634;
          0, 0, -0.001159]; 
      
CoM_pos = CoM_pos.';
