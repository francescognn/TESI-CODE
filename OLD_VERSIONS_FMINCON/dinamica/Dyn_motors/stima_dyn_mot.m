function [J,F]= stima_dyn_mot(v,w,vp,wp,M,I3,d,r)

% v  = column vector of longitudinal speed
%
% w  = column vector of rotational speed
%
% vp = column vector of longitudinal aceleration
%
% wp = column vector of rotational aceleration
%
% M = mass of the base
% 
% I3 = moment of inertia of the base
% 
% d = half distance between weels
%
% r = radius of the wheels
% 

p=[M;I3];

SS=[1/r 1/r; -d/r +d/r];

for i=1:length(v)

    [omega_ruote(:,i)]=SS\[v(i);w(i)];
    [omegap_ruote(:,i)]=SS\[vp(i);wp(i)];
end

for i=1:length(v)
   
    omegal=omega_ruote(1,i);
    omegar=omega_ruote(2,i);
    omegapl=omegap_ruote(1,i);
    omegapr=omegap_ruote(2,i);
    
    PHIc = [ (omegapl+omegapr)/r  (omegal+omegar)/r    ; ... 
            d*(omegapr-omegapl)/r d*(omegar-omegal)/r  ];

    Pc    = [vp(i)   0 ; ...
             0    wp(i)];
      
         if i==1
             PHI=PHIc;
             P=Pc;
         else  
             PHI=[PHI;PHIc];
             P=[P;Pc];
         end

end

PI=(PHI.'*PHI)\PHI.'*P*p;
J=PI(1);
F=PI(2);
end
