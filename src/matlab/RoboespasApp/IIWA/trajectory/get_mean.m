function meanOutput = get_mean(data)
% Standard function used for calculating the mean values between different
% arrays. The argument data is a cell array where each cell is a structure. 
% The mean is performed between the corresponding fields of the structures.

    sizes=zeros(1,size(data,2));
    for i=1:size(data,2) %For each repetition
        sizes(i)=size(data{i}.t,2); %Check size of t
    end
    min_size = min(sizes);

    for i = 1:size(data,2)
        for njoint = 1:7
            mean_tmp.q{njoint}(i,:) = data{i}.q(njoint,1:min_size);
            mean_tmp.qdot{njoint}(i,:) = data{i}.qdot(njoint,1:min_size);
            mean_tmp.qdotdot{njoint}(i,:) = data{i}.qdotdot(njoint,1:min_size);
        end
        for ncoord = 1:3
            mean_tmp.x.pos{ncoord} = data{i}.x.pos(ncoord,1:min_size);
            mean_tmp.x.ori{ncoord} = data{i}.x.ori(ncoord,1:min_size);
            mean_tmp.xdot.pos{ncoord} = data{i}.xdot.pos(ncoord,1:min_size);
            mean_tmp.xdot.ori{ncoord} = data{i}.xdot.ori(ncoord,1:min_size);
        end
        mean_tmp.t(i,:) = data{i}.t(1:min_size);
    end
    for njoint = 1:7
        meanOutput.q(njoint,:) = mean(mean_tmp.q{njoint}, 1);
        meanOutput.qdot(njoint,:) = mean(mean_tmp.qdot{njoint}, 1);
        meanOutput.qdotdot(njoint,:) = mean(mean_tmp.qdotdot{njoint}, 1);
    end
    for ncoord = 1:3
        meanOutput.x.pos(ncoord,:) = mean(mean_tmp.x.pos{ncoord}, 1);
        meanOutput.x.ori(ncoord,:) = mean(mean_tmp.x.ori{ncoord}, 1);
        meanOutput.xdot.pos(ncoord,:) = mean(mean_tmp.xdot.pos{ncoord}, 1);
        meanOutput.xdot.ori(ncoord,:) = mean(mean_tmp.xdot.ori{ncoord}, 1);
    end
    meanOutput.t = mean(mean_tmp.t,1);
end

