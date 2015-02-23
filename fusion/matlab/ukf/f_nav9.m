function output = f_nav9(x, u, dt)
% x = [px, py, pz, vx, vy, vz, abx, aby, abz]
% u = [roll, pitch, yaw, ax, ay, az]

px = x(1); py = x(2); pz = x(3); vx = x(4); vy = x(5); vz = x(6); abx = x(7); aby = x(8); abz = x(9); 
roll = u(1); pitch = u(2); yaw = u(3); ax = u(4); ay = u(5); az = u(6);

[Ax, Ay, Az] = bodyToNed(roll, pitch, yaw, ax-abx, ay-aby, az-abz);

vx = vx + Ax*dt; 
vy = vy + Ay*dt; 
vz = vz + Az*dt;
px = px + vx*dt; 
py = py + vy*dt; 
pz = pz + vz*dt;
           
% predict new state
output = [ px; py; pz; vx; vy; vz; abx; aby; abz;];

end


