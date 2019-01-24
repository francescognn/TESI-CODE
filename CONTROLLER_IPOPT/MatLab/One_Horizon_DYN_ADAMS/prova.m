clear
close all
clc

import casadi.*

x   = MX.sym('x'); 
y   = MX.sym('y');
th  = MX.sym('th');


psi=[x,y,th]';

B = [x*y+th, x, th].';

% dB=MX.sym('dB',3,3);
%  
dB=jacobian(B,[x y th]);
%dB1=reshape(dB(:,2),[]).';
aa=Function('aa',{psi},{dB});



% 
% for i=1:3
%     
%    for j=1:3
%    
%       dB(i,j) = gradient(B(i,j),x); 
%        
%    end 
%    
% end

