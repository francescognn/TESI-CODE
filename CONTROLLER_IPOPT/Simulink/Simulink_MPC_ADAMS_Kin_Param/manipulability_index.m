function man = manipulability_index(x)

man=abs(sin(x(6)));
% q5 = x(5); % th2
% q6 = x(6); % th3
% q7 = x(7); % th4
% q8 = x(8); % th5
% 
% t2 = q5;
% t3 = t2.*2.0;
% t4 = q6;
% t5 = t4.*2.0;
% t6 = q7;
% t7 = t6.*2.0;
% t8 = q8;
% t9 = q5;
% t10 = q6;
% t11 = q7;
% t12 = q8;
% t13 = t8.*2.0;
% man = cos(q5+q6+q7-t9-t10-t11+t13).*(-1.0./2.0)-cos(-q5-q6-q7+t9+t10+t11+t13)./2.0+cos(q8-t12).*(3.0./2.0)-cos(t3+t5+t7).*3.0+cos(q8+t3+t5+t7-t12)./2.0+cos(-q8+t3+t5+t7+t12)./2.0+cos(q5+q6+q7-t9-t10-t11).*3.0-cos(t3+t5+t7+t13)+cos(q5+q6+q7+q8-t9-t10-t11-t12)+cos(q5+q6+q7-q8-t9-t10-t11+t12)-cos(t13).*(3.0./2.0)-cos(t3+t5+t7-t8.*2.0);
