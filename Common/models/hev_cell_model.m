function [x_next, stageCost, unfeas, engPrf, emPrf, battPrf, vehPrf] = hev_cell_model(x, u, w, veh)
%hev_model p2 HEV powertrain model, vectorized version
%   This model implements the same equations as hev_model in a vectorized
%   form that is suitable for usage with DynaProg (for Lab 04).
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
%   structure containing all parameters for the powertain components
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
% Control variables
%   u{1}    Gear number, -
%   u{2}    Engine torque-split factor, -
% Exogenous inputs
%   w{1}    Vehicle speed, m/s
%   w{2}    Vehicle acceleration, m/s^2
% Stage cost
%   stageCost   fuel flow rate, g/s
%
% Usage examples
% ---------------
% Batch (vectorized) state update
%   You can batch evaluate the SOC at the next timestep given the current
%   SOC (scalar), the current vehSpd and vehAcc (scalars), and a whole set
%   of GN and alpha_eng (vectors or matrices) as:
%       [SOC_next, fuelFlwRate, unfeas] = hev_model({SOC}, {GN, alpha_eng}, {vehSpd, vehAcc}, veh)
%   GN and alpha_eng should have the same size.
%   SOC_next will be a cell; the cell will contain a vector or matrix with 
%   the same size as GN and alpha_eng.
%
% Usage with DynaProg
%   If the function is to be used with DynaProg, you will not call it
%   explicitly; the solver will do it for you.
%   Check the toolbox documentation to learn how to tell DynaProg how to
%   use the model function.

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
% Constraints
battUnfeas = ( battPwr <= 0 & battCurr < maxChrgBattCurr ) | ( battPwr > 0 & battCurr > maxDisBattCurr );

%% Stage cost
stageCost  = fuelFlwRate .* veh.dt;

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
