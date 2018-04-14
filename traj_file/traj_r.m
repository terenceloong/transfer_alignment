%static or uniform

if t==0
    %-------------- init --------------%
    T = 100;
    n = T/dt*2+1;
	angle = zeros(n,3); %deg
    speed = zeros(n,3); %m/s
    %---------------------------------------------------------------------%
    p0 = [30, 120, 300]; %deg, [lat,lon,h]
    v0 = [200, 0, 40]; %m/s,deg, [horizontal velocity, down velocity, velocity direction]
    att0 = [40, 2, 0]; %deg, [psi,theta,gamma]
    %---------------------------------------------------------------------%
    Cnb = angle2dcm(att0(1)/180*pi, att0(2)/180*pi, att0(3)/180*pi);
    vh = v0(1);
    vd = v0(2);
    vy = v0(3);
    vn0 = [vh*cosd(vy), vh*sind(vy), vd];
    vb0 = vn0*Cnb';
    angle(1,:) = att0;
    speed(1,:) = vn0;
else
    angle(k,:) = angle(k-1,:);
    speed(k,:) = speed(k-1,:);
    
    %-------------- yaw --------------%

    %-------------- pith --------------%

    %-------------- roll --------------%
    if 15<t && t<=20
        angle(k,3) = -10*(t-15)/5;
    elseif 20<t && t<=25
        angle(k,3) = -10*(25-t)/5;
    end
%     if 0<t && t<=5
%         angle(k,3) = -10*(t-0)/5;
%     elseif 5<t && t<=10
%         angle(k,3) = -10*(10-t)/5;
%     end

    %-------------- vh --------------%

    %-------------- vd --------------%

    %-------------- vy --------------%
    
    %*********************************************************************%
    speed(k,:) = [vh*cosd(vy), vh*sind(vy), vd];
end