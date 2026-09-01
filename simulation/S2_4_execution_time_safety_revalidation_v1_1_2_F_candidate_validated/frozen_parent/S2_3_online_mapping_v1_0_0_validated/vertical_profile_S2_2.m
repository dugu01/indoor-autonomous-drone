function ref = vertical_profile_S2_2(z0,zf,duration_s,time_s)
% VERTICAL_PROFILE_S2_2 Seventh-order rest-to-rest vertical profile.
% Position, velocity, acceleration and jerk are zero-continuous at both
% endpoints. The polynomial is the standard seventh-order smootherstep.

T=max(double(duration_s),eps);
tau=min(1,max(0,double(time_s)/T));
dz=double(zf-z0);

h=35*tau^4-84*tau^5+70*tau^6-20*tau^7;
h1=140*tau^3-420*tau^4+420*tau^5-140*tau^6;
h2=420*tau^2-1680*tau^3+2100*tau^4-840*tau^5;
h3=840*tau-5040*tau^2+8400*tau^3-4200*tau^4;

ref=struct('z',z0+dz*h,'vz',dz*h1/T,'az',dz*h2/T^2, ...
    'jz',dz*h3/T^3,'complete',tau>=1);
end
