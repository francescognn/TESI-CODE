function [A,F,D] = matriceseval_no_cog(x)

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

B = [ Mbase ,   0    ,  0; ...
      0     ,  Mbase ,  0; ...  
      0     ,   0    , I3];


C = zeros(3);
 
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

