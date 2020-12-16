function init_ros(varargin)
% Los enlaces gracias a los cuales he conseguido guardar la dirección 
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
            network_name = 'enp3s0';% 'wlp2s0';
            [~, ros_ip] = system('ifconfig');
%             ['(ifconfig |grep -A 1 "', network_name, '" |tail -1 |cut -d ":" -f 2| cut -d " " -f 1)']);
%             if (isempty(ros_ip))
%                 network_name = 'enp3s0';
%                 [~, ros_ip] = system(['(ifconfig |grep -A 1 "', network_name, '" |tail -1 |cut -d ":" -f 2| cut -d " " -f 1)']);
%                 if (isempty(ros_ip))
%                     network_name = 'enp0s25';
%                     [~, ros_ip] = system(['(ifconfig |grep -A 1 "', network_name, '" |tail -1 |cut -d ":" -f 2| cut -d " " -f 1)']);
%                 end
%             end
%             ros_ip = ros_ip(1:end-1);
%             ros_master_uri = strcat('http://', ros_ip,':11311');
        else
            [~, result] = system('ipconfig');
        end
        ros_master_uri = 'http://160.69.69.100:11311';
        if (contains(result, '160.69.69.100'))
            ros_ip = '160.69.69.100';
        elseif (contains(result, '160.69.69.73'))
            ros_ip = '160.69.69.73';
        else
            ME = MException('InitROS:WrongIP', 'Set IP to 160.69.69.100 or 160.69.69.73');
            throw(ME);
        end
    end
    %Fix:Trucado
    setenv('ROS_MASTER_URI', ros_master_uri);
    setenv('ROS_IP', ros_ip);
    % Inicializa el nodo global y el ROS master
    rosinit(ros_master_uri, 'NodeName','Matlab');
end

