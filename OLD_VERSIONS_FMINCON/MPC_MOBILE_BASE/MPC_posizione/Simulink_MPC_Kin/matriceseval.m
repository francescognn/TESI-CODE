function [A,F,D] = matriceseval(x)

X   = x(1);
Y   = x(2);
TH  = x(3);

       G= [cos(TH) 0; ...
           sin(TH) 0; ...
             0     1];

qp=G*[x(4);x(5)];

Xp=qp(1);
Yp=qp(2);
THp=qp(3);

[Mbase,x_cog_base,y_cog_base,I1,I2,I3,z_man_base,y_man_base,x_man_base,r,d]=parameters();

B = [                 Mbase                             ,                      0                           ,     -Mbase*y_cog_base*cos(TH)-Mbase*x_cog_base*sin(TH); ...
                        0                               ,                    Mbase                         ,     Mbase*x_cog_base*cos(TH)-Mbase*y_cog_base*sin(TH); ...  
      -Mbase*y_cog_base*cos(TH)-Mbase*x_cog_base*sin(TH)     ,   Mbase*x_cog_base*cos(TH)-Mbase*y_cog_base*sin(TH)  ,        I3+Mbase*y_cog_base.^2+Mbase*x_cog_base.^2     ];


C = zeros(3);
C(1,3)=Mbase*(-THp*x_cog_base*cos(TH)+THp*y_cog_base*sin(TH));
C(2,3)=Mbase*(-THp*y_cog_base*cos(TH)-THp*x_cog_base*sin(TH));

% B=[                                                                                                                               Mbase,                                                                                                                                        0,                                                                                                              Mbase*(cos(TH)*(y_cog_base - 1.0*y_man_base) + sin(TH)*(x_cog_base - 1.0*x_man_base) + sin(TH)*(x_man_base^2 + y_man_base^2)^(1/2)); ...
%                                                                                                                                       0,                                                                                                                                    Mbase,                                                                                                         -1.0*Mbase*(cos(TH)*(x_cog_base - 1.0*x_man_base) - sin(TH)*(y_cog_base - 1.0*y_man_base) + cos(TH)*(x_man_base^2 + y_man_base^2)^(1/2)); ...
%     Mbase*(cos(TH)*(y_cog_base - 1.0*y_man_base) + sin(TH)*(x_cog_base - 1.0*x_man_base) + sin(TH)*(x_man_base^2 + y_man_base^2)^(1/2)), -1.0*Mbase*(cos(TH)*(x_cog_base - 1.0*x_man_base) - sin(TH)*(y_cog_base - 1.0*y_man_base) + cos(TH)*(x_man_base^2 + y_man_base^2)^(1/2)), I3 + Mbase*(2.0*x_cog_base*(x_man_base^2 + y_man_base^2)^(1/2) - 2.0*x_man_base*(x_man_base^2 + y_man_base^2)^(1/2) - 2.0*x_cog_base*x_man_base - 2.0*y_cog_base*y_man_base + x_cog_base^2 + 2.0*x_man_base^2 + y_cog_base^2 + 2.0*y_man_base^2)];
%             
%    
%  C = [ 
%                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         0, -0.5*Mbase*(Xp*cos(TH)*(x_man_base^2 + y_man_base^2)^(1/2) - THp*cos(TH)*(x_man_base^2 + y_man_base^2)^(1/2) - THp*x_cog_base*cos(TH) + THp*x_man_base*cos(TH) + Xp*x_cog_base*cos(TH) - Xp*x_man_base*cos(TH) + Yp*y_cog_base*cos(TH) - Yp*y_man_base*cos(TH) + Yp*sin(TH)*(x_man_base^2 + y_man_base^2)^(1/2) + THp*y_cog_base*sin(TH) - THp*y_man_base*sin(TH) - Xp*y_cog_base*sin(TH) + Yp*x_cog_base*sin(TH) + Xp*y_man_base*sin(TH) - Yp*x_man_base*sin(TH)); ...
%                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         0, -0.5*Mbase*(Xp*cos(TH)*(x_man_base^2 + y_man_base^2)^(1/2) - THp*y_cog_base*cos(TH) + THp*y_man_base*cos(TH) + Xp*x_cog_base*cos(TH) - Xp*x_man_base*cos(TH) + Yp*y_cog_base*cos(TH) - Yp*y_man_base*cos(TH) - THp*sin(TH)*(x_man_base^2 + y_man_base^2)^(1/2) + Yp*sin(TH)*(x_man_base^2 + y_man_base^2)^(1/2) - THp*x_cog_base*sin(TH) + THp*x_man_base*sin(TH) - Xp*y_cog_base*sin(TH) + Yp*x_cog_base*sin(TH) + Xp*y_man_base*sin(TH) - Yp*x_man_base*sin(TH)); ...
%  (Mbase*(THp*cos(TH)*(x_man_base^2 + y_man_base^2)^(1/2) + Xp*cos(TH)*(x_man_base^2 + y_man_base^2)^(1/2) + THp*x_cog_base*cos(TH) - 1.0*THp*x_man_base*cos(TH) + Xp*x_cog_base*cos(TH) - 1.0*Xp*x_man_base*cos(TH) + Yp*y_cog_base*cos(TH) - 1.0*Yp*y_man_base*cos(TH) + Yp*sin(TH)*(x_man_base^2 + y_man_base^2)^(1/2) - 1.0*THp*y_cog_base*sin(TH) + THp*y_man_base*sin(TH) - 1.0*Xp*y_cog_base*sin(TH) + Yp*x_cog_base*sin(TH) + Xp*y_man_base*sin(TH) - 1.0*Yp*x_man_base*sin(TH)))/2, (Mbase*(Xp*cos(TH)*(x_man_base^2 + y_man_base^2)^(1/2) + THp*y_cog_base*cos(TH) - 1.0*THp*y_man_base*cos(TH) + Xp*x_cog_base*cos(TH) - 1.0*Xp*x_man_base*cos(TH) + Yp*y_cog_base*cos(TH) - 1.0*Yp*y_man_base*cos(TH) + THp*sin(TH)*(x_man_base^2 + y_man_base^2)^(1/2) + Yp*sin(TH)*(x_man_base^2 + y_man_base^2)^(1/2) + THp*x_cog_base*sin(TH) - 1.0*THp*x_man_base*sin(TH) - 1.0*Xp*y_cog_base*sin(TH) + Yp*x_cog_base*sin(TH) + Xp*y_man_base*sin(TH) - 1.0*Yp*x_man_base*sin(TH)))/2,                                                                                                                                                                                                                                                                                                                                                                                                                                                                  0];
%  
    
 g=zeros(3,1);
         
        Gp=[-THp*sin(TH) 0; ...
             THp*cos(TH) 0; ...
                  0      0];

        S=1/r.*[cos(TH) cos(TH); ...
                sin(TH) sin(TH); ...
                  -d       d    ];
              
Bt=G.'*B*G;
Ct=G.'*B*Gp+G.'*C*G;
gt=G.'*g;
St=G.'*S;

A = [zeros(3,3)    G; ...
     zeros(2,3) -Bt\Ct];

F = [zeros(3,2); Bt\St];

D = [zeros(3,1); -Bt\gt];

              
end

