function init_ros(varargin)
% Los enlaces gracias a los cuales he conseguido guardar la dirección 
% IP en una variable son:
% https://ch.mathworks.com/matlabcentral/answers/428272-get-computer-ip-problem
% https://www.mundotelematico.com/linux-variable-de-entorno-con-direccion-ip-del-equipo/

    if (robotics.ros.internal.Global.isNodeActive)
        rosshutdown;
    end

    if (isunix)
        [~, ros_ip] = system('(ifconfig |grep -A 1 "wlp2s0" |tail -1 |cut -d ":" -f 2| cut -d " " -f 1)');
        ros_ip = ros_ip(1:end-1);
        ros_master_uri = strcat('http://', ros_ip,':11311');
    else
        if (length(varargin)==1)
            ros_ip = varargin{1};
            ros_master_uri = strcat('http://', ros_ip,':11311');
        else
            disp('Add IP to function');
        end
    end

    setenv('ROS_MASTER_URI', ros_master_uri);
    setenv('ROS_IP', ros_ip);
    % Inicializa el nodo global y el ROS master
    rosinit(ros_master_uri, 'NodeName','Matlab');
end

