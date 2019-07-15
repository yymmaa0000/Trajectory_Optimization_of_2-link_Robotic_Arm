function [] = DynamicEquationGenerator(L1,L2,m1,m2,mo)
% Declare decision variables
syms q1 q2 dq1 dq2 real
syms tau_1 tau_2 real

q = [q1;q2];
dq = [dq1;dq2];
tau = [tau_1;tau_2];

% derive important parameters
CM2 = (mo*L2 + 1/2*m2*L2)/(mo+m2);
I2 = 1/12*m2*L2^2 + m2*(CM2-L2/2)^2 + mo*(L2-CM2)^2;
g = 9.81;

% Compute generalized inertia matrices for each link
M1 = [m1 0 0;
    0 m1 0;
    0 0 1/12*m1*L1^2];
M2 = [m2+mo 0 0;
    0 m2+mo 0;
    0 0 I2];

% Compute body Jacobians for each link
Jb_sL1 = [0 0;
    L1/2 0;
    1 0];
Jb_sL2 = [L1*sin(q2) 0;
    L1*cos(q2)+CM2 CM2;
    1 1];

% Compute mass matrix
M = simplify(Jb_sL1'*M1*Jb_sL1 + Jb_sL2'*M2*Jb_sL2);

% Compute Coriolis matrix
C  = sym(zeros(length(q),length(q)));
for ii = 1:length(q)
    for jj = 1:length(q)
        for kk = 1:length(q)
            C(ii,jj) = C(ii,jj) + 1/2*(diff(M(ii,jj),q(kk)) + diff(M(ii,kk),q(jj)) - diff(M(jj,kk),q(ii)))*dq(kk);
        end
    end
end
C = simplify(C);

% Compute nonlinear and applied force terms (no constraints)
V = simplify(m1*g*L1/2*sin(q1) + (m2+mo)*g*(L1*sin(q1) + CM2*sin(q1+q2)));
N = jacobian(V, q)';
Y = [tau_1;tau_2];

% Export as Matlab Function
matlabFunction(M,C,N,Y, 'File', 'DynamicEquation', 'Outputs', {'M','C','N','Y'}, 'Vars', {q,dq,tau});
end
