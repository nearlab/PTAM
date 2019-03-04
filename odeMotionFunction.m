function [Xdot] = odeMotionFunction(t,X,M,F,T)

%initialize Xdot
Xdot = zeros(18,1);

%construct R intertial to body
R = reshape(X(7:15), 3, 3);

%change in position
Xdot(1:3) = X(4:6);

%change in velocity
Xdot(4:6) = R'*F/M.m;

%change in attitude
%R_dot = -[omegaBx]*RBI
omegaBx = crossProductEquivalent(X(16:18));
R_dot = -omegaBx*R;
Xdot(7:15) = reshape(R_dot,9,1);

%change in angular rate
Xdot(16:18) = M.J^(-1)*T;

end