function  [fun,fun1] = self_collision(q)
x1=q(4);x2=q(5);x3=q(6);x4=q(7);x5=q(8);x6=q(9);
 
base_length =0.76;
base_width=0.46;
base_height=0.7;
xb = 0.185; 
yb = 0;
zb = base_height; 

DHtable = [ 0         , pi/2  , 0.08916 , x1+pi/2;
           -0.425     , 0     , 0       , x2;
           -0.39225   , 0     , 0       , x3;
            0         , pi/2  , 0.10915 , x4;
            0         , -pi/2 , 0.09465 , x5;
            0         , 0     , 0.216  , x6];
A = rototrasl(DHtable);
T=[eye(3),[xb;yb;zb];0 0 0 1];
for kk = 1:5
    T = T*A{kk};
end
sphere_center=T(1:3,4);
sphere_radius=0.2;
% cylinder_radius=sqrt(zb^2+0.235^2);
% dist=sqrt(sphere_center(2)^2+sphere_center(3)^2);
% fun = dist - sphere_radius - cylinder_radius;

cent_sph_base1 = [base_length/4;0;zb/2];
cent_sph_base2 = [-base_length/4;0;zb/2];
rad_sph_base = sqrt((base_length/2-cent_sph_base1(1))^2+(base_width/2 -cent_sph_base1(2))^2+(base_height -cent_sph_base1(3))^2);
dist_sph1 =  sqrt(sum((sphere_center-cent_sph_base1).^2));
dist_sph2 =  sqrt(sum((sphere_center-cent_sph_base2).^2));
fun(1,1)=dist_sph1-sphere_radius-rad_sph_base;
fun(2,1)=dist_sph2-sphere_radius-rad_sph_base;

x_kinect = -0.76/2; y_kinect=-0.4; radius_kinect = 0.13;
fun1=(sqrt((x_kinect-sphere_center(1))^2 + (y_kinect-sphere_center(2))^2)-sphere_radius-radius_kinect);













