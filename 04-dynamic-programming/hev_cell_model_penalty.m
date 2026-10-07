function [x_next, stageCost, unfeas, engPrf, emPrf, battPrf, vehPrf] = hev_cell_model_penalty(x, u, w, veh, c1, c2)
%hev_cell_model_penalty p2 HEV powertrain model with drivability penalties
%   Same powertrain equations as hev_cell_model, extended for the second
%   part of Lab 04: two extra states store the previous gear and the
%   previous engine state, and the stage cost is augmented with penalty
%   terms charging c1 on every gear shift and c2 on every engine start:
%
%       L = fuelFlwRate*dt + c1*(u{1} ~= x{2}) + c2*(u{2} > 0 & x{3} == 0)
%
%   Both penalties add to a fuel mass in grams, so c1 and c2 act as
%   gram-equivalent costs: one gear shift weighs like c1 grams of fuel.
%
% Input arguments
% ---------------
% x : cell
%   current value of the state variable(s)
% u : cell
%   current value of the control variable(s)
% w : cell
%   current value of the exogenous input(s)
% veh : struct
%   structure containing all parameters for the powertrain components
% c1 : double
%   gear-shift penalty, charged when the chosen gear u{1} differs from
%   the previous gear x{2}
% c2 : double
%   engine-start penalty, charged when the engine is on now (u{2} > 0)
%   while it was off at the previous stage (x{3} == 0)
%
% Outputs
% ---------------
% x_next : cell
%   value of the state variable(s) at the end of the current timestep
% stageCost : double
%   stage (running) cost incurred
% unfeas : logical
%   when set to true, at least one of the constraints was violated
% engPrf, emPrf, battPrf, vehPrf : struct
%   data structures to return additional quantities of interest for
%   visualization/postprocessing
%
% Model details
% ---------------
% State variables
%   x{1}    Battery SOC, -
%   x{2}    Gear number at the previous stage, -
%   x{3}    Engine state at the previous stage (0 = off, 1 = on), -
% Control variables
%   u{1}    Gear number, -
%   u{2}    Engine torque-split factor, -
% Exogenous inputs
%   w{1}    Vehicle speed, m/s
%   w{2}    Vehicle acceleration, m/s^2
% Stage cost
%   stageCost   fuel mass over the timestep (g) plus the two drivability
%               penalty terms
% State dynamics of the added states
%   x_next{2} = u{1}        the gear chosen now becomes the previous gear
%   x_next{3} = (u{2} > 0)  the engine state now becomes the previous one
%
% Usage with DynaProg
%   The function is not called explicitly; DynaProg calls it as f(x,u,w).
%   The extra parameters are bound in the function handle:
%       SysName = @(x,u,w) hev_cell_model_penalty(x, u, w, veh, c1, c2)

%% Driveline (Vehicle + Final Drive + Transmission)
[shaftSpd, demTrq, vehPrf] = hev_drivetrain(w{1}, w{2}, u{1}, veh);

%% Torque split
% Torque provided by engine
engTrq  = u{2} .* demTrq; % Nm
% Torque provided by electric motor
emShaftTrq  = ( 1 - u{2} ) .* demTrq; % Nm

%% Engine
% Engine state 
% Assume the engine is turned off when the engine torque is null
engineState = engTrq > 0;

% Fuel mass flow rate
engSpd = shaftSpd.*ones(size(engTrq)); % rad/s
fuelFlwRate = veh.eng.fuelMap(engSpd, engTrq); % g/s
fuelFlwRate( engineState == 0 ) = 0;

% Maximum engine torque
engMaxTrq = veh.eng.maxTrq(engSpd); % Nm

% Costraints
engSpdUnfeas = ( engineState == 1 ) & ( ( engSpd < veh.eng.idleSpd & u{1}~=1 ) | ( engSpd > veh.eng.maxSpd ) );
engTrqUnfeas = ( engTrq > engMaxTrq ) | ( engTrq < 0 ); 
engUnfeas =  ( engSpdUnfeas | engTrqUnfeas );

%% EM
% Torque-coupling device (ideal)
emSpd = shaftSpd .* veh.em.tcSpdRatio;
emTrq = emShaftTrq ./ veh.em.tcSpdRatio; % Nm

% Electric motor efficiency
emSpd = emSpd.*ones(size(emTrq));
emEff = (shaftSpd~=0) .* veh.em.effMap(emSpd, emTrq) + (shaftSpd==0);

% Limit Torque
emMaxTrq = veh.em.maxTrq(emSpd); % Nm
emMinTrq = veh.em.minTrq(emSpd); % Nm

% Saturate regen braking torque
emTrq = max(emTrq, emMinTrq); % Nm

% Calculate electric power consumption
emElPwr = (emTrq<0) .* emSpd.*emTrq.*emEff + (emTrq>=0) .* emSpd.*emTrq./emEff; % W

% Constraints
emSpdUnfeas = ( emSpd > veh.em.maxSpd ) & ( emTrq ~= 0 );
emTrqUnfeas = ( emTrq < emMinTrq ) | ( emTrq > emMaxTrq );
emUnfeas = ( emSpdUnfeas | emTrqUnfeas );

%% Battery
battPwr = emElPwr; % W

% columbic efficiency
battColumbicEff = (battPwr>0) + (battPwr<=0) .* veh.batt.coulombic_eff;
% Battery internal resistance
battR = veh.batt.eqRes(x{1}); % ohm
% Battery voltage
battVoltage = veh.batt.ocv(x{1}); % V

% Battery current
battCurr = (battVoltage-sqrt(battVoltage.^2 - 4.*battR.*battPwr))./(2.*battR); % A
battCurr = real(battCurr);
% Current limits
maxChrgBattCurr = veh.batt.minCurr; % A
maxDisBattCurr = veh.batt.maxCurr; % A
% Saturate charge current in regen braking
battCurr = max(battCurr, maxChrgBattCurr);

% New battery state of charge
x_next{1}  = - battColumbicEff .* battCurr ./ (veh.batt.nomCap * 3600) .* veh.dt + x{1};
% Finalize the next state variables
x_next{2} = u{1};     % Update gear number
x_next{3} = u{2} > 0; % Update engine state
% Constraints
battUnfeas = ( battPwr <= 0 & battCurr < maxChrgBattCurr ) | ( battPwr > 0 & battCurr > maxDisBattCurr );

%% Stage cost
% add c1 and c2 penalty to reduce gear shiftings and engine starts
stageCost  = fuelFlwRate .* veh.dt + c1 * (u{1} ~= x{2}) + c2 * (u{2} > 0 & x{3} == 0);

%% Unfeasibilites
% Force pure electric mode when braking (regenerative braking)
pwtUnfeas = ( demTrq < 0 & emTrq > 0 );

% Combine unfeasibilities
unfeas = ( pwtUnfeas | engUnfeas | emUnfeas | battUnfeas );

%% Powerflow (operating mode)
pwrFlw = repmat("", size(engTrq));
pwrFlw( u{2} == 0 ) = 'pe';
pwrFlw( u{2} > 0 & u{2} < 1 ) = 'ps';
pwrFlw( u{2} == 1 ) = 'pt';
pwrFlw( u{2} > 1 ) = 'bc';

%% Pack additional outputs
if isscalar(x{1}) && isscalar(u{1})
    engPrf.fuelFlwRate = fuelFlwRate;
    engPrf.engSpd = engSpd;
    engPrf.engTrq = engTrq;
    engPrf.engSpdUnfeas = engSpdUnfeas;
    engPrf.engTrqUnfeas = engTrqUnfeas;
    engPrf.engineState = engineState;

    emPrf.emSpd = emSpd;
    emPrf.emTrq = emTrq;
    emPrf.emElPwr = emElPwr;
    emPrf.emSpdUnfeas = emSpdUnfeas;
    emPrf.emTrqUnfeas = emTrqUnfeas;

    battPrf.battSOC = x{1};
    battPrf.battCurr = battCurr;
    battPrf.battVolt = battVoltage;
    battPrf.battRes = battR;

    vehPrf.gearNumber = u{1};
    vehPrf.engAlpha = u{2};
    vehPrf.pwrFlw = pwrFlw;
    vehPrf.vehSpd = w{1};
    vehPrf.vehAcc = w{2};
    vehPrf.engUnfeas = engUnfeas;
    vehPrf.emUnfeas = emUnfeas;
    vehPrf.battUnfeas = battUnfeas;
    vehPrf.pwtUnfeas = pwtUnfeas;
    vehPrf.unfeas = unfeas;
end