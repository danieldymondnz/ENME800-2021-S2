function [angle] = wrapToRad(angle, min, max)
%WRAPTORAD Attempt to wrap an angle in radians within a given range
% Inputs:
% angle : Angle in radians to wrap
% min   : Minimum angle in radians
% max   : Maximum angle in radians
% Angle should always follow max > min


    % If angle is too big
    if angle > max
        
        % Remove 2*pi from angle until it is less than max
        newAngle = angle;
        while (newAngle > max) && (newAngle > min)
            newAngle = newAngle - (2*pi);
        end
        
        % If angle doesn't violate min, then set as new angle
        if newAngle >= min
            angle = newAngle;
        end

    % If angle is too small
    elseif angle < min
        
        % Remove 2*pi from angle until it is less than max
        newAngle = angle;
        while (newAngle < min) && (newAngle < max)
            newAngle = newAngle + (2*pi);
        end
        
        % If angle doesn't violate min, then set as new angle
        if newAngle <= max
            angle = newAngle;
        end

    end


%     % Convert angle within the +2pi, -2pi range
%     if abs(angle) > (2*pi)
%         angle = sign(angle) * mod(angle, 2*pi);
%     end
% 
%     % Attempt to generate an angle within the working envelope
%     big = 0; small = 0;
% 
%     % Attempt to fit within working envelope
%     if angle > max
%         big = 1;
%     end
%     if angle < min
%         small = 1;
%     end
% 
%     if big == 1
%         angle = angle - (2*pi);
%     elseif small == 1
%         angle = angle + (2*pi);
%     end

end

