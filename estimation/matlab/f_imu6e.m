function output = f_imu6e(x, u, dt)
% x = [roll, pitch, yaw, gbx, gby, gbz]
% u = [gx, gy, gz]

roll = x(1); pitch = x(2); yaw = x(3); gbx = x(4); gby = x(5); gbz = x(6);
gx = u(1)-gbx; gy = u(2)-gby; gz = u(3)-gbz;

rollRate  = gx + gy*sin(roll)*tan(pitch) + gz*cos(roll)*tan(pitch);
pitchRate = gy*cos(roll) - gz*sin(roll);
yawRate = 0;
if(abs(pitch) < pi/2)
    yawRate = gy*sin(roll)/cos(pitch) + gz*cos(roll)/cos(pitch);
end

% predict new state
output = [ roll + gx*dt;
           pitch + gy*dt;
           yaw + gz*dt;
           gbx;
           gby;
           gbz;                  ];
end
