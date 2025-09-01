function [A,B,C] = discretize_model(P, vx_ref, dt)
% Very small-signal linearization around vx_ref with bicycle model
m = P.m; Iz=P.Iz; lf=P.lf; lr=P.lr; Cf=P.Cf; Cr=P.Cr;
vx = max(vx_ref, 1.0);
A_c = [  0      1      0   0     0    0;
         0      0      0   0     0    0;
         0      0      0   1    0     0;
         0      0      0  -(Cf+Cr)/(m*vx)   0   (lr*Cr - lf*Cf)/(m*vx);
         0      0      0    0     0    1;
         0      0      0   (lr*Cr - lf*Cf)/(Iz*vx) 0  -(lr^2*Cr + lf^2*Cf)/(Iz*vx)];
B_c = [  0     1/m;
         0       0;
         0       0;
         0     Cf/m;
         0   lf*Cf/Iz;
         0       0];

% Tustin (bilinear) or simple forward Euler (stable enough for dt=0.05)
A = eye(6) + dt*A_c;
B = dt*B_c;

% output Y = [X vy theta]^T
C = [0 0 1 0 0 0;
     0 1 0 0 0 0;
     0 0 0 0 1 0];
end
