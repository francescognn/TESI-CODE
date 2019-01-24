function [DHtable,Abase] = parameters_MM_simple(q)
N=9;
if length(q) ~= N
    error('q has the wrong number of elements');
end

%TO BE DEFINED:
z_man_base = 1;    % height of the manipulator base from the ground
x_man_base = 0;  % distance of the manipulator base from the center of the base 
                   % along its longitudinal axis 
y_man_base = 0.; % distance of the manipulator base from the center of the base 
                   % along its trasversal axis 
   

a_man=sqrt(x_man_base^2 + y_man_base^2);
d_man=sqrt(z_man_base^2 + y_man_base^2);
                   
X=q(1); Y=q(2); TH=q(3); th1=q(4); th2=q(5); th3=q(6); th4=q(7); th5=q(8); th6=q(9); 

DHtable = [ 0     , pi/2  , X       , pi/2;
            0     , -pi/2 , Y       , -pi/2;
            a_man , 0     , d_man   , TH;
            0     , pi/2  , 0.08916 , th1;
            0.425 , 0     , 0       , th2;
            0.39225, 0    , 0       , th3;
            0     , pi/2  , 0.10915 , th4;
            0     , -pi/2 , 0.09456 , th5;
            0     , 0     , 0.0823  , th6];

Abase = [0 , 0 , 1 , 0;
         1 , 0 , 0 , 0;
         0 , 1 , 0 , 0;
         0 , 0 , 0 , 1] ;
     

