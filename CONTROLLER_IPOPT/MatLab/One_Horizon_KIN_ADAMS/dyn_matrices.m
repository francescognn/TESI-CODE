function [A, B, C] = dyn_matrices(TH1P, TH2P, TH3P, TH4P, TH5P, TH6P, TH1, TH2, TH3, TH4, TH5, TH6, x, y, th, u, udot)

%% variables definition

import casadi.*

 q  = [x, y, th,TH1, TH2, TH3, TH4, TH5, TH6].';
 qp = [u(7)*cos(th), u(7)*sin(th), u(8), TH1P, TH2P, TH3P, TH4P, TH5P, TH6P].';
 
 N=length(q);
 
 B1=MX(N,N);
 C1=MX(N,N);
 gq1=MX(N,1);
 Peso=MX(N,1);
 
 
[DHtable,Abase,CoM_pos,m,In] = parameters_MM([x, y, th,TH1, TH2, TH3, TH4, TH5, TH6].');

[Jp,Jo,T] = jacobians([x, y, th,TH1, TH2, TH3, TH4, TH5, TH6].');

%% Building B1 marix

for i = 1:N
    B1=B1+m(i)*(Jp{i}.'*Jp{i}) + Jo{i}.'*T{i}(1:3,1:3)*In{i}*T{i}(1:3,1:3).'*Jo{i}; 
end

%B1_sym=B1f(psi_sym);

%% Building C1 Matrix

dB = {};
% dB_sum=zeros(N);
   
    for ii=1:N
        
    dBt=jacobian(B1,[x y th TH1 TH2 TH3 TH4 TH5 TH6]');
    dB{ii}=reshape(dBt(:,ii),[9,9]).';
    
    end

%C1={};
hijk={};

for j=1:N
    for i=1:N
        for k=1:N
            
            hijk{k} = dB{k}(i,j)-0.5*dB{i}(j,k);
            %C1{1}(i,j)=hijk*qp(k).';
            
            C1(i,j)=hijk{k}*qp(k);
        end
    end
end

%% Building gravity vector
g=9.81;

    for i=1:N

        hg=T{i}*[CoM_pos(:,i); 1];
        Peso(i)=m(i)*g*hg(3);
    end
    
    gq1=sum(jacobian(Peso,[x, y, th, TH1, TH2, TH3, TH4, TH5, TH6])).';
    
%% Recalling therms

Bq  = B1(4:end,4:end);
Bqx = B1(4:end,1:3);
Cq  = C1(4:end,4:end);
Cqx = C1(4:end,1:3);

gq=gq1(4:end,1);

G = [ cos(th) 0; ...
      sin(th) 0; ...
         0    1];

Gdot = [-sin(th) 0; ...
         cos(th) 0; ...
           0     0].*u(8);
       
vdot = udot(7:8);
v    = u(7:8);
 
Fin=Bqx*G*vdot+(Bqx*Gdot+Cqx*G)*v;

O2  = zeros(2);
O3  = zeros(3);
O6  = zeros(6);
O36 = zeros(3,6);
O63 = zeros(6,3);
O61 = zeros(6,1);
O31 = zeros(3,1);
O62 = zeros(6,2);



I6=eye(6);

%% Building A , B , C matrices & T_ee



A  = [ -inv(Bq)*Cq    O6      O63   ; ...
        I6            O6      O63   ;
        O36           O36     O3    ];


B  = [-inv(Bq) O62 ; ...
        O6     O62 ; ...
        O36    G  ];

    
C  = [-inv(Bq)*(Fin+gq); O61; O31];

end





     
         