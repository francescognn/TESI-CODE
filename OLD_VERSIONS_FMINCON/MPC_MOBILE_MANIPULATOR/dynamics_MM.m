clear all
close all
clc

syms x1 x2 x3 x4 x5 x6 x7 x8 x9
syms x1p x2p x3p x4p x5p x6p x7p x8p x9p
q = [x1 x2 x3 x4 x5 x6 x7 x8 x9];
qp = [x1p x2p x3p x4p x5p x6p x7p x8p x9p];

%%
[DHtable,Abase,CoM_pos,m,In] = parameters_MM(q);
N=length(q);

[Jp,Jo,T] = jacobians(q);

B=sym(zeros(N));
C=sym(zeros(N));
gq=sym(zeros(N,1));


for i = 1:N
    B=B+m(i)*(Jp{i}.'*Jp{i}) + Jo{i}.'*T{i}(1:3,1:3)*In{i}*T{i}(1:3,1:3).'*Jo{i}; 
end

%%

% Building C Matrix

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

%%

% Building gravity vector
g=9.81;
for i=1:N

    hg=T{i}*[CoM_pos(:,i); 1];
    Peso=m(i)*g*hg(3);
    gq(i,1)=diff(Peso,q(i));
    
end





     
         