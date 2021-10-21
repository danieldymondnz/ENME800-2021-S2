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

end