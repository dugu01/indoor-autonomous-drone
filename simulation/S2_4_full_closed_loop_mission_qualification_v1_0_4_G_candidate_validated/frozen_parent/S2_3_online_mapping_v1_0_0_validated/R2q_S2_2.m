function q = R2q_S2_2(R)
tr=trace(R);
if tr>0
    S=sqrt(tr+1)*2;q=[0.25*S,(R(3,2)-R(2,3))/S,(R(1,3)-R(3,1))/S,(R(2,1)-R(1,2))/S];
else
    [~,i]=max(diag(R));
    if i==1
        S=sqrt(max(1+R(1,1)-R(2,2)-R(3,3),1e-12))*2;
        q=[(R(3,2)-R(2,3))/S,0.25*S,(R(1,2)+R(2,1))/S,(R(1,3)+R(3,1))/S];
    elseif i==2
        S=sqrt(max(1+R(2,2)-R(1,1)-R(3,3),1e-12))*2;
        q=[(R(1,3)-R(3,1))/S,(R(1,2)+R(2,1))/S,0.25*S,(R(2,3)+R(3,2))/S];
    else
        S=sqrt(max(1+R(3,3)-R(1,1)-R(2,2),1e-12))*2;
        q=[(R(2,1)-R(1,2))/S,(R(1,3)+R(3,1))/S,(R(2,3)+R(3,2))/S,0.25*S];
    end
end
q=qnormalize_S2_2(q);
end
