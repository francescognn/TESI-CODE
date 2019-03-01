function [] = spheres_gen(X,T)
%% Cilindro kinect

[X_cyl_kin,Y_cyl_kin,Z_cyl_kin]=cylinder(0.13,50);
dist_c=sqrt((0.76/2)^2+0.2^2);
phi_c=0.4845;
X_cyl_kin = X(1,:)-cos(X(3,:)+phi_c)*dist_c + X_cyl_kin;
Y_cyl_kin = X(2,:)-sin(X(3,:)+phi_c)*dist_c + Y_cyl_kin;
Z_cyl_kin=Z_cyl_kin+0.5;

%% Sfera EE

[Xsfera_ee,Ysfera_ee,Zsfera_ee] = sphere(10);

raggio_sfera_ee=0.2;
Xsfera_ee=Xsfera_ee*raggio_sfera_ee;
Ysfera_ee=Ysfera_ee*raggio_sfera_ee;
Zsfera_ee=Zsfera_ee*raggio_sfera_ee;
 
Xsfera_ee=Xsfera_ee+T(33,4);
Ysfera_ee=Ysfera_ee+T(34,4);
Zsfera_ee=Zsfera_ee+T(35,4);


%% Sfere base

[Xsfera_base,Ysfera_base,Zsfera_base] = sphere(10);
raggio_sfera_base=0.4599;
Xsfera_base=Xsfera_base*raggio_sfera_base;
Ysfera_base=Ysfera_base*raggio_sfera_base;
Zsfera_base=Zsfera_base*raggio_sfera_base+0.35;

Xsfera_base_fr=Xsfera_base+X(1)+0.19*cos(X(3));
Ysfera_base_fr=Ysfera_base+X(2)+0.19*sin(X(3));

Xsfera_base_re=Xsfera_base+X(1)-0.19*cos(X(3));
Ysfera_base_re=Ysfera_base+X(2)-0.19*sin(X(3));

%% SURFING

% surf(X_cyl_kin,Y_cyl_kin,Z_cyl_kin,'facealpha',0.001,'edgealpha',0.3,'LineWidth',5)
hold on
surf(Xsfera_ee,Ysfera_ee,Zsfera_ee,'facealpha',0.4,'edgealpha',0.04)
surf(Xsfera_base_fr,Ysfera_base_fr,Zsfera_base,'facealpha',0.4,'edgealpha',0.04)
surf(Xsfera_base_re,Ysfera_base_re,Zsfera_base,'facealpha',0.4,'edgealpha',0.04)

end

