function [stamps_out, trajectory_out] = equidistant(stamps, trajectory, tsample)
    format long;
    stamps=stamps-stamps(1);
    total_time=stamps(end);
    total_stamps_out=round(total_time/tsample);
    stamps_out=zeros(1,total_stamps_out);
    new_pos=zeros(7,1);
    current_t=0;
    n=2;
    stamps_out(1)=0;
    trajectory_out(:,1)=trajectory(:,1);
    for i=2:size(stamps,2)
        diff=stamps(i)-stamps(i-1);
        current_t=current_t+diff;
        while current_t>(n-1)*tsample
            new_stamp=(n-1)*tsample;
            %Interpolate new_pos from previous pos and next pos
            x=[stamps(i-1), stamps(i)];
            v=[trajectory(:,i-1), trajectory(:,i)];
            for j=1:7 %Predict linearly each of the joint values
                new_pos(j)=interp1(x,v(j,:),new_stamp);
            end
            %Fill output
            stamps_out(n)=new_stamp;
            trajectory_out(:,n)=new_pos;
            %Update variables
            n=n+1;
        end
    end
end

