function output = h_imu3qa(x)
% x = [q0, q1, q2, q3, gbx, gby, gbx]
% z = [ax, ay, az]

% normalize
q0 = x(1); q1 = x(2); q2 = x(3); q3 = x(4);

% estimated direction of gravity and flux (NED)
output = [ 2*(q0*q2 - q1*q3);               % ax
          -2*(q0*q1 + q2*q3);               % ay
          -q0*q0 + q1*q1 + q2*q2 - q3*q3;]; % az
end


% notas:
% quaternion conjugado:
% q' = (q0 - q1*i - q2*j - q3*k)
% multiplicacion quaterniones (Hamilton Product) 
% http://en.wikipedia.org/wiki/Quaternion#Hamilton_product):
% q = (q0 + q1*i + q2*j + q3*k)
% p = (p0 + p1*i + p2*j + p3*k)
% r = (r0 + r1*i + r2*j + r3*k)
% q = p x r ->
% q0 = (p0*r0 - p1*r1 - p2*r2 - p3*r3) = (r0*p0 - r1*p1 - r2*p2 - r3*p3)
% q1 = (p0*r1 + p1*r0 + p2*r3 - p3*r2) = (r0*p1 + r1*p0 - r2*p3 + r3*p2) 
% q2 = (p0*r2 - p1*r3 + p2*r0 + p3*r1) = (r0*p2 + r1*p3 + r2*p0 - r3*p1)
% q3 = (p0*r3 + p1*r2 - p2*r1 + p3*r0) = (r0*p3 - r1*p2 + r2*p1 + r3*p0)
% 
% calculo de medidas:
% medida de la gravedad normalizada en ejes de navegacion: [0 0 1] (ENU)
% quaternion equivalente: g = (0 + 0*i + 0*j + 1*k)
% medida esperada de los acelerometros normalizados (rotacion a ejes cuerpo):
% qa = q'*g*q = 
% 0 + 2*(q1*q3 - q0*q2)*i + 2*(q0*q1 + q2*q3)*j + (q0*q0 - q1*q1 - q2*q2 + q3*q3)*k;
%  
% medida de la gravedad normalizada en ejes de navegacion: [0 0 -1] (NED)
% quaternion equivalente: g = (0 + 0*i + 0*j - 1*k)
% medida esperada de los acelerometros normalizados (rotacion a ejes cuerpo):
% qa = q'*g*q =  +-q3 --q2 +-q1 -q0 = (-q3 +q2 -q1 -q0)*q =
% -q3*q0 - q2*q1 + q1*q2 + q0*q3
% -q3*q1 + q2*q0 - q1*q3 + q0*q2
% -q3*q2 - q2*q3 - q1*q0 - q0*q1
% -q3*q3 + q2*q2 + q1*q1 - q0*q0
% 0 + 2*(q0*q2 - q1*q3)*i - 2*(q0*q1 + q2*q3)*j + (-q0*q0 + q1*q1 + q2*q2 - q3*q3)*k;