function [pos, vel, acc, timeVec] = linetraject(p0,pf,T,numPoints)
    tacc = 0.1*T;
    v_mid = (pf - p0)/T;
    timeVec = linspace(0, T, numPoints)';  % 时间向量
    % disp(timeVec);
    a0 = (0.5*(v_mid*T^4 - 2*v_mid*T^3*tacc + 2*v_mid*T*tacc^3 - v_mid*tacc^4 + 2*p0*tacc^3))/tacc^3;
    a1 = -(2*v_mid*T^3 - 3*tacc*v_mid*T^2)/tacc^3;
    a2 = (3*(v_mid*T^2 - tacc*v_mid*T))/tacc^3;
    a3 = -(v_mid*(2*T - tacc))/tacc^3;
    a4 = (0.5000*v_mid)/tacc^3;

    for i = 1:length(timeVec)
        t = timeVec(i);
        if t <= tacc
            pos(i) = 0.5 * v_mid * tacc + p0 + v_mid/(tacc)^2 * t^3 + (-v_mid/(2*(tacc^3)) * t^4);
            % fprintf('%.6f',pos(i));
            vel(i) = 3 * v_mid/(tacc)^2 * t^2 + 4 * (-v_mid/(2*(tacc^3)) * t^3);
            acc(i) = 6 * v_mid/(tacc)^2 * t   + 12* (-v_mid/(2*(tacc^3)) * t^2);
        elseif t > tacc && t <= (T -tacc) 
            pos(i) = p0 + v_mid * t;
            % fprintf('%0.6f',pos(i));
            vel(i) = v_mid;
            acc(i) = 0;
        else     
            pos(i) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4;
            % fprintf('%0.6f',pos(i));
            vel(i) = a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3;
            acc(i) = 2*a2 + 6*a3*t + 12*a4*t^2;
        end
    end
end