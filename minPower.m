function cost = minPower(X)
% objective function that minimize power over time

q = X(:,1:2);
dq = X(:,3:4);
u = X(:,5:6);
q1 = q(:,1);
q2 = q(:,2);
dq1 = dq(:,1);
dq2 = dq(:,2);

dt = 0.02;

cost = 0;
for i = 1:length(X)-1   
    % Trapezoidal integration of torque times angular velovity => Power
    cost = cost + 0.5*dt*(abs(u(i+1,1)*dq(i+1,1))+abs(u(i+1,2)*dq(i+1,2))...
        + abs(u(i,1)*dq(i,1))+abs(u(i,2)*dq(i,2)));
    
end

end