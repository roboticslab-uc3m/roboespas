function data_output = get_data_from_pp(data)
    % Given:
    % -data.t
    % -data.pp.breaks
    % -data.pp.coefs.q
    % -data.pp.coefs.qdot
    % -data.pp.coefs.qdotdot
    % Calculate 
    % -data.q
    % -data.qdot
    % -data.qdotdot
    % -data.x.pos, data.x.ori
    % -data.xdot.pos, data.xdot.ori
    data_output=data;
    data_output.q=zeros(7,size(data.t,2));
    data_output.qdot=zeros(7,size(data.t,2));
    data_output.qdotdot=zeros(7,size(data.t,2));
    i=1;
    b_cont=true;
    while b_cont
        t=data.t(i);
        idSeg_all=find(t>=data.pp.breaks, 1, 'last');
        idSeg=idSeg_all(1,1);
        if (idSeg>size(data.pp.coefs.q,2))
            idSeg=size(data.pp.coefs.q,2);
        end
        tSeg=data.pp.breaks(idSeg);
        for idJoint=1:size(data.pp.coefs.q,3)
            data_output.q(idJoint, i)=polyval(data.pp.coefs.q(1:4, idSeg, idJoint), t-tSeg);
            data_output.qdot(idJoint,i)=polyval(data.pp.coefs.qdot(1:3, idSeg, idJoint), t-tSeg);
            data_output.qdotdot(idJoint,i)=polyval(data.pp.coefs.qdotdot(1:2, idSeg, idJoint), t-tSeg);
        end
        i=i+1;
        if (i==size(data.t,2)+1)
            b_cont=false;
        end
    end
    data_output=fill_cartesian_withoutmex(data_output);
end

