function qdot_out = limit_qdot(qdot, qdotmax)
    qdot_out=qdot;
    for i=1:size(qdotmax,2)    
        if qdot(i)>qdotmax(i)
            qdot_out(i)=qdotmax(i);
            disp('Limited qdot max')
        elseif qdot(i)<-qdotmax(i)
            qdot_out(i)=-qdotmax(i);
            disp('Limited negative qdot max')
        end
    end
end
    
    