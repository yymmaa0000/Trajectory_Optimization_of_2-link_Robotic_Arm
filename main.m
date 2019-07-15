%{
24-785 Project
Team 7 
Peter Li, Alex Sun, Bo Tian, XingYu Wang, Yufan Zhang
05/07/2019
%}
close all;clear;clc;

% parameters to change
end_time = 1.5;
m1 = 10;
m2 = 1;
mo = 1;
L1 = 1;
L2 = 1;
obj_fun = 1; % not 0 = minTorque, 0 = minPower

% setup problem
DynamicEquationGenerator(L1,L2,m1,m2,mo);
if obj_fun ~= 0
    obj_fun = @minTorque;
    obj_fun_name = "minTorque";
else
    obj_fun = @minPower;
    obj_fun_name = "minPower";
end

x0 = [0;0;0;0];
u0 = [0;0];

dt = 0.02;
tspan = 0:dt:end_time;
N = length(tspan);
vars = [x0;u0];

% initial guess
X0 = zeros(N,numel(vars));

% solve the potimization
options = optimoptions('fmincon','Display', 'iter', 'MaxFunctionEvaluations', 1e5,'PlotFcn','optimplotfval');
[sol, cost] = fmincon(obj_fun, X0, [],[],[],[],[],[],@nonlincon, options);

% animate the result and save
animate(sol,dt,L1,L2,m1,m2,mo,cost,obj_fun_name,1);
title_name = sprintf("%d_%d_%d_%d_%d_%s",L1,L2,m1,m2,mo,obj_fun_name);
save(title_name);