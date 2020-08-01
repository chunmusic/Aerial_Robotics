function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls
Kvz = 20;
Kpz = 80;
Kvy = 5;
Kpy = 20;
Kvp = 10;
Kpp = 1000;

y_t_dd = des_state.acc(1);
y_t_d = des_state.vel(1);
y_t = des_state.pos(1);
y_d = state.vel(1);
y = state.pos(1);

g = params.gravity;
m = params.mass;
zt_dd = des_state.acc(2);
zt_d = des_state.vel(2);
z_d = state.vel(2);
zt = des_state.pos(2);
z = state.pos(2);
ixx = params.Ixx;

phi_d = state.omega;
phi = state.rot;

phi_t_d = 0;
phi_t_dd = 0;
phi_t = (-1.0/g)*(y_t_dd + Kvy*(y_t_d-y_d) + Kpy*(y_t-y));

u1 = m * ( g + zt_dd + Kvz*(zt_d - z_d) + Kpz*(zt - z));
u2 = ixx * (phi_t_dd + Kvp*(phi_t_d-phi_d) + Kpp*(phi_t-phi));

% FILL IN YOUR CODE HERE

end

