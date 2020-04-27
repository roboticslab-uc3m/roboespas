function ori_dot = substract_ori_traj(ori, t)
    ori_dot=zeros(3, size(ori,2)-1);
    for i=1:size(ori,2)-1
        ori_dot(:,i)=substract_ori(ori(:,i)', ori(:,i+1)', t(i+1)-t(i));
    end
end

