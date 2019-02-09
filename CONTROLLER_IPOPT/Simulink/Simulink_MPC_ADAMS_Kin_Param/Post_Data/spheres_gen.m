function [] = spheres_gen(X,T)

[Xsfera,Yc0,Zc0]=cylinder(0.13,50);
[Xcc,Ycc,Zcc]=cylinder(0.7384,50);
[Xs,Ys,Zs] = sphere(10);

Zc=Zc0+0.5;
Zc2=Ycc;

rs=0.2;
Xs=Xs*rs;
Ys=Ys*rs;
Zs=Zs*rs;

Xc2=Zcc-0.4;
Yc2=Xcc;

 Xc=Xsfera-0.76/2+X(1,:);
    Yc=Yc0-0.2+X(2,:);
    
    Xss=Xs+T(33,4);
    Yss=Ys+T(34,4);
    Zss=Zs+T(35,4);
    
    surf(Xc,Yc,Zc,'facealpha',0.001,'edgealpha',0.3,'LineWidth',5)
    hold on

    surf(Xss,Yss,Zss,'facealpha',0.4,'edgealpha',0.04)

end

