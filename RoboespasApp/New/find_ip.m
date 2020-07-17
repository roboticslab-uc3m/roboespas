function ip = find_ip()
    if (isunix)
        [~, str] = system('ifconfig');
        sub_str=extractBetween(str, 'wl', 'Difus.'); %Get the IP after wl
        Q = '((0*(1\d\d|2[0-4]\d|25[0-4]|\d\d|\d)\.){3}0*(1\d\d|2[0-4]\d|25[0-4]|\d\d|\d))';
        ips =regexp(sub_str, Q, 'match');
        ip=ips{1};
        ip=ip{1};
    else
        %Implement
    end
end