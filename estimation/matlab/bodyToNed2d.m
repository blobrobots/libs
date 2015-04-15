% transform from body to earth NED frame in 2D
function [efx, efy] = bodyToNed2d(yaw, bfx, bfy)

efx = bfx*cos(yaw) - bfy*sin(yaw);
efy = bfx*sin(yaw) + bfy*cos(yaw);

end