function [A, B, C] = dyn_matrices(psi,u)

import casadi.*

% psi = [TH1P TH2P TH3P TH4P TH5P TH6P TH1 TH2 TH3 TH4 TH5 TH6 x y th ]'



%% Building B marix

[DHtable,Abase,CoM_pos,m,In] = parameters_MM(psi(7:15));

N=length(psi(7:15));

[Jp,Jo,T] = jacobians(psi(7:15));

B=sym(zeros(N));
C=sym(zeros(N));
gq=sym(zeros(N,1));


for i = 1:N
    B=B+m(i)*(Jp{i}.'*Jp{i}) + Jo{i}.'*T{i}(1:3,1:3)*In{i}*T{i}(1:3,1:3).'*Jo{i}; 
end

%% Building C Matrix

dB = cell(N,1);
% dB_sum=zeros(N);
for ii = 1:N
    dB{ii} = diff(B,q(ii));
%     dB_sum = dB_sum+dB{ii};
end
hijk=sym(zeros(1,3));
for j=1:N
    for i=1:N
        for k=1:N
            hijk(k)= dB{k}(i,j)-0.5*dB{i}(j,k);
        end
        C(i,j)=hijk*qp.';
    end
end

%%  Building gravity vector
g=9.81;
for i=1:N

    hg=T{i}*[CoM_pos(:,i); 1];
    Peso=m(i)*g*hg(3);
    gq(i,1)=diff(Peso,q(i));
    
end

end





     
         