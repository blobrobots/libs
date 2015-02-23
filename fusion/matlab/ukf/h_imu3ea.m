function output = h_imu3ea(x)
% x = [roll, pitch, yaw, gbx, gby, gbx]
% z = [ax, ay, az]

roll = x(1); pitch = x(2); yaw = x(3);

g = -1; % ned2frd

% estimated direction of gravity
output = [ -g*sin(pitch);              % ax
            g*sin(roll)*cos(pitch);    % ay
            g*cos(pitch)*cos(roll); ]; % az
end

% ROTACION NED2BODY:
% *bfx = efx*cosf(pitch)*cosf(yaw)                                     + efx*cosf(pitch)*sinf(yaw)                                     - efz*sinf(pitch);
% *bfy = efx*(sinf(roll)*sinf(pitch)*cosf(yaw) - cosf(roll)*sinf(yaw)) + efy*(sinf(roll)*sinf(pitch)*sinf(yaw) + cosf(roll)*cosf(yaw)) + efz*sinf(roll)*cosf(pitch);
% *bfz = efx*(cosf(roll)*sinf(pitch)*cosf(yaw) + sinf(roll)*sinf(yaw)) + efy*(cosf(roll)*sinf(pitch)*sinf(yaw) - sinf(roll)*cosf(yaw)) + efz*cosf(pitch)*cosf(roll);
  