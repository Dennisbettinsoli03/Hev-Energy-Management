function [shaftSpd, shaftTrq, vehPrf] = hev_drivetrain(vehSpd, vehAcc, GN, veh)
%hev_drivetrain drivetrain sub-model for the p2 HEV powertrain model
%   Implements a backward quasistatic model from the driving cycle to the
%   gearbox input, including the vehicle longitudinal dynamics, wheels,
%   final drive and gearbox.

%% Vehicle longitudinal dynamics
% Tractive effort (N)
vehResForce =  veh.body.f0 + veh.body.f1 .* vehSpd + veh.body.f2 .* vehSpd.^2;
vehForce = (vehSpd~=0) .* (vehResForce + veh.body.mass.*vehAcc);

%% Wheels
% Wheel speed (rad/s)
wheelSpd  = vehSpd ./ veh.wh.radius;
% Wheel torque (Nm)
wheelTrq = vehForce .* veh.wh.radius;
% Apply braking regulation
wheelTrq(vehForce<0) = 0.6 .* wheelTrq(vehForce<0);
% Final Drive
% Final drive input speed (rad/s)
fdSpd = veh.fd.spdRatio .* wheelSpd;
% Final drive input torque (Nm)
fdTrq  = wheelTrq ./ veh.fd.spdRatio;

%% Gearbox
gbSpRatio = veh.gb.spdRatio(GN);

% Crankshaft speed (rad/s)
shaftSpd  = gbSpRatio .* fdSpd;
% Gearbox efficiency (-)
gbEff = veh.gb.effMap(GN);
% Crankshaft torque (Nm)
shaftTrq  = (fdTrq>0) .* ( fdTrq ./ ( gbSpRatio .* gbEff ) )  ...
    + (fdTrq<=0) .* ( fdTrq ./ gbSpRatio ) .* gbEff  ;

%% Pack additional outputs
vehPrf.vehForce = vehForce;
vehPrf.shaftSpd = shaftSpd;
vehPrf.reqTrq = shaftTrq;