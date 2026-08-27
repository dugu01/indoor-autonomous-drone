function q = qnormalize_S2_2(q)
q = q(:).';
n = norm(q);
if ~isfinite(n) || n < 1e-12, q = [1 0 0 0]; return; end
q = q/n;
if q(1) < 0, q = -q; end
end
