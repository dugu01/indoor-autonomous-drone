function v = qlog_S2_2(q)
q=qnormalize_S2_2(q);nv=norm(q(2:4));
if nv<1e-10,v=2*q(2:4);return;end
a=2*atan2(nv,q(1));if a>pi,a=a-2*pi;end
v=a*q(2:4)/nv;
end
