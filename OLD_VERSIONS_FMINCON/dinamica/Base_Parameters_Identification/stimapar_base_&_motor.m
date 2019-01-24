function PI = stimapar(q,dq,dt,tau)

% q is a matrix of measurements of q = [q_meas_1 q_meas_2 q_meas_3 ... q_meas_n]
% qp is is a matrix of measurements of qp = [qp_meas_1 qp_meas_2 qp_meas_3 ... qp_meas_n]
%
% dt is the time increment between measurements (sampling time)
%
%tau is a matrix of input [tau_1 tau_2 ... tau_n]

r=0.1;

d=0.2;


for i=1:size(q,1)
ddq(i,:)=diff(dq(i,:))./dt;
end
q=q(:,2:end);
dq=dq(:,2:end);

for i=1:size(q,2)

ddqc=ddq(:,i);
dqc=dq(:,i);
qc=q(:,i);
tauc=tau(:,i);

x=qc(1); y=qc(2); th=qc(3); xp=dqc(1); yp=dqc(2); thp=dqc(3); xpp=ddqc(1); ypp=ddqc(2); thpp=ddqc(3);

phi=[xpp       -thpp*sin(th)-thp^2*cos(th)     -thpp*cos(th)+thp^2*sin(th)     0      0      0 ;...
     ypp       thpp*cos(th)-thp^2*sin(th)      -thpp*sin(th)-thp^2*cos(th)     0      0      0 ;...
      0         -xpp*sin(th)+ypp*cos(th)         -ypp*sin(th)-xpp*cos(th)      thpp   thpp   thpp  ];

G=[cos(th) 0; ...
   sin(th) 0; ...
     0     1];

S=1/r.*[cos(th) cos(th); ...
        sin(th) sin(th); ...
          -d       d    ];

      
PHIc = (G.'*phi);

Tc = G.'*S*tauc;

if i==1
PHI=[PHIc];
T=[Tc];
else    
PHI=[PHI;PHIc];

T=[T;Tc];

end

end


PI=(PHI.'*PHI)\PHI.'*T;


end
