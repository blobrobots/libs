% transform from body to earth NED frame in 3D
function [efx, efy, efz] = bodyToNed(roll, pitch, yaw, bfx, bfy, bfz)

efx =  bfx*cos(pitch)*cos(yaw) + bfy*(sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw)) + bfz*(cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw));
efy =  bfx*cos(pitch)*sin(yaw) + bfy*(sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw)) + bfz*(cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw));
efz = -bfx*sin(pitch)          + bfy*sin(roll)*cos(pitch)                                 + bfz*cos(roll)*cos(pitch);
 
end