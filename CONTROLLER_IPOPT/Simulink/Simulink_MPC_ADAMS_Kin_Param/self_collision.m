function fun = self_collision(q)
x1=q(4);x2=q(5);x3=q(6);x4=q(7);%x5=q(8);x6=q(9);
xb = 0; 
yb = 0;
zb = 0.7;  
DHtable = [ 0         , pi/2  , 0.08916 , x1+pi/2;
           -0.425     , 0     , 0       , x2;
           -0.39225   , 0     , 0       , x3;
            0         , pi/2  , 0.10915 , x4];
%             0         , -pi/2 , 0.09465 , x5;
%             0         , 0     , 0.0823  , x6];
A = rototrasl(DHtable);
T=[eye(3),[xb;yb;zb];0 0 0 1];
for kk = 1:4
    T = T*A{kk};
end
sphere_center=T(1:3,4);
sphere_radius=0.15;
cylinder_radius=sqrt(zb^2+0.47^2);
dist=sqrt(sphere_center(2)^2+sphere_center(3)^2);

fun = dist - sphere_radius - cylinder_radius;
