function q_out = limit_q(q, qmax, qmin)
    for i=1:size(qmax,2)
        if abs(q(i))>pi
            q(i) = q(i) - sign(q(i))*2*pi;
        end
        if q(i) >= 0
            if qmax(i) >= q(i)
                q_out(i) = q(i);
            elseif qmin(i) <= q(i)-2*pi
                q_out(i) = q(i)-2*pi;
                disp('Transformed q -2pi');
            elseif (q(i) - qmax(i)) <= (qmin(i) - (q(i)-2*pi))
                q_out(i) = qmax(i);
                disp('Limited qmax');
            else
                q_out(i) = qmin(i);
                disp('Limited qmin');
            end
        else
            if qmin(i) <= q(i)
                q_out(i) = q(i);
            elseif qmax(i) >= q(i)+2*pi
                q_out(i) = q(i)+2*pi;
                disp('Transformed q +2pi')
            elseif (qmin(i) - q(i)) <= ((q(i)+2*pi) - qmax(i))
                q_out(i) = qmin(i);
                disp('Limited qmin');
            else
                q_out(i) = qmax(i);
                disp('Limited qmax');
            end       
        end
    end
end    
    