function value = eval_min_snap_segment_S2_2(coefficients,t,derivativeOrder)
% EVAL_MIN_SNAP_SEGMENT_S2_2 Evaluate a seventh-order 2-D polynomial.
if nargin<3,derivativeOrder=0;end
value=zeros(size(t(:),1),2);
for power=derivativeOrder:7
    scale=factorial(power)/factorial(power-derivativeOrder);
    value=value+(t(:).^(power-derivativeOrder))*(scale*reshape(coefficients(power+1,:),1,2));
end
if isscalar(t),value=value(1,:);end
end
