function q = qmul_S2_2(a,b)
a=qnormalize_S2_2(a);b=qnormalize_S2_2(b);
q=[a(1)*b(1)-dot(a(2:4),b(2:4)), ...
   a(1)*b(2:4)+b(1)*a(2:4)+cross(a(2:4),b(2:4))];
q=qnormalize_S2_2(q);
end
