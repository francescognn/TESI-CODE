function [fun,fun1] = SCA(q)
%SCA
%    [FUN,FUN1] = SCA(X1,X2,X3,X4)

x1=q(4);
x2=q(5);
x3=q(6);
x4=q(7);
%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    06-Feb-2019 10:41:07

t4 = sin(x2);
t2 = t4.*4.25e4+cos(x2+x3+x4).*9.465e3+sin(x2+x3).*3.9225e4-7.8916e4;
t3 = cos(x1);
t5 = cos(x2);
t6 = cos(x3);
t7 = sin(x3);
t8 = cos(x4);
t9 = sin(x4);
t11 = sin(x1);
t12 = t11.*2.183e3;
t10 = t12-t3.*t5.*8.5e3+t3.*t4.*t7.*7.845e3-t3.*t5.*t6.*7.845e3+t3.*t4.*t6.*t8.*1.893e3-t3.*t4.*t7.*t9.*1.893e3+t3.*t5.*t6.*t9.*1.893e3+t3.*t5.*t7.*t8.*1.893e3;
fun = 8.431488599292535e1.*(-1.0./1.0e2)+sqrt(t2.^2+t10.^2.*2.5e1)./1.0e5-1.0./5.0;
if nargout > 1
    t14 = -x1+x2+x3+x4;
    t15 = x1-x2;
    t16 = x1+x2+x3;
    t17 = -x1+x2+x3;
    t18 = x1+x2+x3+x4;
    t19 = x1+x2;
    t13 = t12-cos(t15).*4.25e3-cos(t16).*3.9225e3-cos(t17).*3.9225e3-cos(t19).*4.25e3+sin(t14).*9.465e2+sin(t18).*9.465e2+4.0e3;
    t20 = t3.*2.183e3-cos(t14).*9.465e2+cos(t18).*9.465e2+sin(t15).*4.25e3+sin(t16).*3.9225e3-sin(t17).*3.9225e3+sin(t19).*4.25e3+1.13e4;
    fun1 = sqrt(t13.^2+t20.^2)./2.0e4-3.3e1./1.0e2;
end