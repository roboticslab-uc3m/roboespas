function init_ros(varargin)
% Los enlaces gracias a los cuales he conseguido guardar la direcci√≥n 
% IP en una variable son:
% https://ch.mathworks.com/matlabcentral/answers/428272-get-computer-ip-problem
% https://www.mundotelematico.com/linux-variable-de-entorno-con-direccion-ip-del-equipo/

    if (robotics.ros.internal.Global.isNodeActive)
        rosshutdown;
    end
    if (length(varargin)==1)
        ros_ip = varargin{1};
        ros_master_uri = strcat('http://', ros_ip,':11311');
    else
        if (isunix)
            network_name = 'wlp2s0';
            [~, ros_ip] = system(['(ifconfig |grep -A 1 "', network_name, '" |tail -1 |cut -d ":" -f 2| cut -d " " -f 1)']);
            if (isempty(ros_ip))
                network_name = 'enp3s0';
                [~, ros_ip] = system(['(ifconfig |grep -A 1 "', network_name, '" |tail -1 |cut -d ":" -f 2| cut -d " " -f 1)']);
            end
            ros_ip = ros_ip(1:end-1);
            ros_master_uri = strcat('http://', ros_ip,':11311');
        else
            [~, result] = system('ipconfig');
            id_ipv4 = strfind(result, 'IPv4');
            id_nextline = strfind(result, 'M·scara');
            ros_ip = extractBetween(result,id_ipv4(1)+34,id_nextline(1)-5);
            ros_ip = ros_ip{1};
            ros_master_uri = strcat('http://', ros_ip,':11311');
        end
    end

    setenv('ROS_MASTER_URI', ros_master_uri);
    setenv('ROS_IP', ros_ip);
    % Inicializa el nodo global y el ROS master
    rosinit(ros_master_uri, 'NodeName','Matlab');
end

