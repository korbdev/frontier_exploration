function [theta, radius] = transformCartesianToPolar(p)
radius = norm(p);
theta = 0;
if(p(1) >= 0)
    theta = acos(p(2)/radius);
else
    theta = 2*pi - acos(p(2)/radius);
end
end