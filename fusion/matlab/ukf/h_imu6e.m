function output = h_imu6e(x)
% x = [roll, pitch, yaw, gbx, gby, gbx]
% z = [ax, ay, az, mx, my, mz]

roll = x(1); pitch = x(2); yaw = x(3);

g = -1; % ned2frd
h = 1;

% estimated direction of gravity and flux
output = [ -g*sin(pitch);             % ax
            g*sin(roll)*cos(pitch);   % ay
            g*cos(pitch)*cos(roll);   % az
            h*cos(pitch)*cos(yaw);    % mx
            h*(sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw));    % my
            h*(cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw)); ]; % mz
end

% TODO: Falta declinacion magnetica

% ROTACION NED2BODY:
% *bfx = efx*cosf(pitch)*cosf(yaw)                                     + efx*cosf(pitch)*sinf(yaw)                                     - efz*sinf(pitch);
% *bfy = efx*(sinf(roll)*sinf(pitch)*cosf(yaw) - cosf(roll)*sinf(yaw)) + efy*(sinf(roll)*sinf(pitch)*sinf(yaw) + cosf(roll)*cosf(yaw)) + efz*sinf(roll)*cosf(pitch);
% *bfz = efx*(cosf(roll)*sinf(pitch)*cosf(yaw) + sinf(roll)*sinf(yaw)) + efy*(cosf(roll)*sinf(pitch)*sinf(yaw) - sinf(roll)*cosf(yaw)) + efz*cosf(pitch)*cosf(roll);
  