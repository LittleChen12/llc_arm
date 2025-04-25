% function [pos, vel, acc, timeVec] = linetraject(p0,pf,T,numPoints)
%     tacc = 0.1*T;
%     v_mid = (pf - p0)/T;
%     timeVec = linspace(0, T, numPoints)';  % 时间向量
%     % disp(timeVec);
%     a0 = (0.5*(v_mid*T^4 - 2*v_mid*T^3*tacc + 2*v_mid*T*tacc^3 - v_mid*tacc^4 + 2*p0*tacc^3))/tacc^3;
%     a1 = -(2*v_mid*T^3 - 3*tacc*v_mid*T^2)/tacc^3;
%     a2 = (3*(v_mid*T^2 - tacc*v_mid*T))/tacc^3;
%     a3 = -(v_mid*(2*T - tacc))/tacc^3;
%     a4 = (0.5000*v_mid)/tacc^3;
% 
%     for i = 1:length(timeVec)
%         t = timeVec(i);
%         if t <= tacc
%             pos(i) = 0.5 * v_mid * tacc + p0 + v_mid/(tacc)^2 * t^3 + (-v_mid/(2*(tacc^3)) * t^4);
%             % fprintf('%.6f',pos(i));
%             vel(i) = 3 * v_mid/(tacc)^2 * t^2 + 4 * (-v_mid/(2*(tacc^3)) * t^3);
%             acc(i) = 6 * v_mid/(tacc)^2 * t   + 12* (-v_mid/(2*(tacc^3)) * t^2);
%         elseif t > tacc && t <= (T -tacc) 
%             pos(i) = p0 + v_mid * t;
%             % fprintf('%0.6f',pos(i));
%             vel(i) = v_mid;
%             acc(i) = 0;
%         else     
%             pos(i) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4;
%             % fprintf('%0.6f',pos(i));
%             vel(i) = a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3;
%             acc(i) = 2*a2 + 6*a3*t + 12*a4*t^2;
%         end
%     end
% end

function [pos, vel, acc, timeVec] = linetraject(p0,pf,tf,numPoints,a)
    if p0>pf %减速
        a = -1 * a;
        if a > (4*(pf - p0) / (tf^2))
            a = (4*(pf - p0) / (tf^2));
        end
        tb1 = (0.5000*a*tf - 0.5000*(a*(a*tf^2 + 4*p0 - 4*pf))^(1/2))/a;
        tb2 = (0.5000*a*tf + 0.5000*(a*(a*tf^2 + 4*p0 - 4*pf))^(1/2))/a;
        if tb1 <= tb2
            tb = tb1;    
        else
            tb = tb2;
        end
    end
    if p0<=pf %加速
        a = 1*a;%加速度为正
        if a < (4*(pf - p0) / (tf^2))
            a = (4*(pf - p0) / (tf^2));
        end
        tb1 = (0.5000*a*tf - 0.5000*(a*(a*tf^2 + 4*p0 - 4*pf))^(1/2))/a;
        tb2 = (0.5000*a*tf + 0.5000*(a*(a*tf^2 + 4*p0 - 4*pf))^(1/2))/a;
        if tb1 <= tb2
            tb = tb1;    
        else
            tb = tb2;
        end 
        
    end   
    fprintf('tb1: %f\n', tb);
    timeVec = linspace(0, tf, numPoints)';  % 时间向量
    % disp(timeVec);

    for i = 1:length(timeVec)
        t = timeVec(i);
        if t <= tb
            pos(i) = p0 + 0.5 * a * t^2;
            % fprintf('%.6f',pos(i));
            vel(i) = a * t;
            acc(i) = a;
        elseif t > tb && t <= (tf - tb) 
            pos(i) = p0 + 0.5 * a * tb^2 + a * tb * (t-tb);
            % fprintf('%0.6f',pos(i));
            vel(i) = a * tb;
            acc(i) = 0;
        else     
            % pos(i) = p0 + 0.5 * a * tb^2 + a * tb * (tf-2*tb) + a * tb * (t - (tf - tb)) - 0.5 * a * (t - (tf - tb))^2;
            pos(i) = p0 + a * tb * (tf - tb) - 0.5 * a * (t - tf) * (t - tf);
            % fprintf('%0.6f',pos(i));
            vel(i) = (a * tb) - a * (t - (tf - tb));
            acc(i) = -a;
        end
    end
end