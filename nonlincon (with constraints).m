function [c,ceq] = nonlincon(X)

% Unpack optimization variables
q = X(:,1:2);
dq = X(:,3:4);
u = X(:,5:6);

q1 = q(:,1);
q2 = q(:,2);
dq1 = dq(:,1);
dq2 = dq(:,2);
u1 = u(:,1);
u2 = u(:,2);
N = size(X,1);

% Define constants (length of pendulum, time step)
L = 1;
dt = 0.02;

% Define desired initial and final states
q0Des = [-pi/2;0];
dq0Des = [0;0];
qfDes = [pi/2;0];
dqfDes = [0;0];
qmax = deg2rad(120);
qmin = -qmax;

% define torque limit
umax = 30;
umin = -umax;
torqueLimits = zeros(N,4);

% Define boundary constraints
constBoundary = [q(1,:)' - q0Des               % Initial Position
    dq(1,:)' - dq0Des                          % Initial Velocity
    q(end,:)' - qfDes                          % Final Position
    dq(end,:)' - dqfDes];                      % Final Velocity

% Compute acceleration dynamics
ddq = [];
for i = 1:N
    [Mbar,Cbar,Nbar,Y] = computeDynamicMatrices(q(i,:)',dq(i,:)',u(i,:)');
    ddq(i,:) = (Mbar\(Y-Cbar*dq(i,:)'-Nbar))';
    
    % Compute joint limit constraint functions
    constJointLimits(i,:) = [q1(i) - qmax, -q1(i) + qmin, q2(i) - qmax, -q2(i) + qmin];
    torqueLimits(i,:) = [u1(i)-umax, -u1(i)+umin, u2(i)-umax, -u2(i)+umin];
end

% Define collocation constraints
constFirstOrderCollocation = zeros(N-1,2);
constSecondOrderCollocation = zeros(N-1,2);
for i = 1:length(X)-1
    constFirstOrderCollocation(i,:) = 0.5*dt*(dq(i+1,:) + dq(i,:)) - (q(i+1,:) - q(i,:));
    constSecondOrderCollocation(i,:) = 0.5*dt*(ddq(i+1,:) + ddq(i,:)) - (dq(i+1,:) - dq(i,:));
end

% Stack everything in a vector
ceq = [constBoundary;constFirstOrderCollocation(:);constSecondOrderCollocation(:)];
% c = [];
c = [constJointLimits(:); torqueLimits(:)];
% c = [torqueLimits(:)];
% c = [constJointLimits(:)];

end
