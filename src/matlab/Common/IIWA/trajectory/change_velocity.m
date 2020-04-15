function data_output = change_velocity(data, velocity)
% Function that changes the velocity of a given data struct
    % Initialize the output
    data_output=data;
    % Change the joint velocity multiplying by the given velocity factor
    data_output.qdot=velocity.*data.qdot;
    % Change the time stamps so the trajectory is executed faster
    data_output.t=data.t./velocity;
    % Transform the interpolation polynomials taking into account that
    % velocity
    %data_output=bounded_spline(data_output, velocity);

    if (isfield(data, 'pp'))
        data_output.pp.breaks=data.pp.breaks./velocity;
        for idSeg=1:size(data.pp.breaks,2)-1
            for idJoint=1:size(data.pp.coefs.q,3) %7
                % Modify coefficients for joint position
                for idCoef=1:size(data.pp.coefs.q,1) %4
                    % First coefficient c=1 -> t³ should be multiplied by the
                    % factor "velocity"³, c=2 -> ·velocity², ... This means
                    % that each coefficient should be multiplied by the
                    % factor raised to 4 minus the index of the coefficient
                    data_output.pp.coefs.q(idCoef, idSeg, idJoint)=data.pp.coefs.q(idCoef, idSeg, idJoint)*velocity^(4-idCoef);
                end
                % Modify coefficients for joint velocity
                for idCoef=1:size(data.pp.coefs.qdot,1) %3
                    % Same idea, but as the velocity is the derivative of
                    % the position, you should multiply once more by the
                    % "velocity" coefficient
                     data_output.pp.coefs.qdot(idCoef, idSeg, idJoint)=data.pp.coefs.qdot(idCoef, idSeg, idJoint)*velocity^(3-idCoef+1);
                end
                for idCoef=1:2
                    % Same idea, but as the velocity is the derivative of
                    % the position, you should multiply twice more by the
                    % "velocity" coefficient
                    data_output.pp.coefs.qdotdot(idCoef, idSeg, idJoint)=data.pp.coefs.qdotdot(idCoef, idSeg, idJoint)*velocity^(2-idCoef+2);
                end
            end
        end
    end
    data_output=fill_cartesian(data_output);
    data_output.velocity=velocity;
end

