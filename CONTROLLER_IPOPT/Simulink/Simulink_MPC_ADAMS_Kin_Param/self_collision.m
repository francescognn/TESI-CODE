syms X Y TH x1 x2 x3 x4 x5 x6
q = [0 0 0 0 0 0 0 0 0 ];
[T,R,P]= FK(q);
T_cylinder = [R{4},[X;Y;0];0 0 0 1];
sphere_center=T{9}(:,4);
coord_cyl_sphere=T_cylinder*sphere_center;coord_cyl_sphere=coord_cyl_sphere(1:3);
sphere_radius=0.15;
