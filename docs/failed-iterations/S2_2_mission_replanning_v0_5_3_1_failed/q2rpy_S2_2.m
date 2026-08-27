function rpy = q2rpy_S2_2(q)
R=q2R_S2_2(q);p=asin(max(-1,min(1,-R(3,1))));r=atan2(R(3,2),R(3,3));y=atan2(R(2,1),R(1,1));rpy=[r p y];
end
