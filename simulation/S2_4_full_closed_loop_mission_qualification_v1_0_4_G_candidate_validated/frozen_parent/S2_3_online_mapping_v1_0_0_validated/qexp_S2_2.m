function q = qexp_S2_2(v)
v=v(:).';a=norm(v);
if a<1e-10,q=qnormalize_S2_2([1,0.5*v]);else,q=[cos(a/2),sin(a/2)*v/a];end
end
