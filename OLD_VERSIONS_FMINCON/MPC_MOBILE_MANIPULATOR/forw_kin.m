function fw_ee = forw_kin(q)
q = q(1:9);
N=length(q);
[DHtable,A0,CoM_pos,m,In] = parameters_MM(q);
A = rototrasl(DHtable); 
T=cell(N+1,1);
T{1}=A0;
for kk = 1:N
    T{kk+1} = T{kk}*A{kk};
end

p_ee = T{N+1}(1:3,4);

R = T{N+1}(1:3,1:3);
th = atan2(sqrt(R(2,3)^2+R(1,3)^2),R(3,3));
if th >0 && th<pi
    phi = atan2(R(2,3),R(1,3));
    psi = atan2(R(3,2),-R(3,1));
else
    th = atan2(-sqrt(R(2,3)^2+R(1,3)^2),R(3,3));
    phi = atan2(-R(2,3),-R(1,3));
    psi = atan2(-R(3,2),R(3,1));
end
angles_ee = [phi;th;psi];

fw_ee = [p_ee;angles_ee];

end