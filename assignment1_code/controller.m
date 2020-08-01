function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

e = s_des(1) - s(1);
e_diff = s_des(2) - s(2);



Kv = 1000
Kp = 10000


u = params.mass * (Kp*e + Kv*e_diff + params.gravity);






% FILL IN YOUR CODE HERE


end

