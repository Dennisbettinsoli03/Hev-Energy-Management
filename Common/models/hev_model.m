function [x_next, stageCost, unfeas, engPrf, emPrf, battPrf, vehPrf, engTrq, engMaxTrq, emMaxTrq, emMinTrq] = hev_model(x, u, w, veh)
%hev_model p2 HEV powertrain model
%   Implements a backward quasistatic model for a p2 parallel HEV,
%   advancing the simulation by one timestep.
%
% Input arguments
% ---------------
% x : double
%   current value of the state variable(s)
% u : double
%   current value of the control variable(s)
% w : double
%   current value of the exogenous input(s)
% veh : struct
%   structure containing all parameters for the powertain components
%
% Outputs
% ---------------
% x_next : double
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
%   x(1)    Battery SOC, -
% Control variables
%   u(1)    Gear number, -
%   u(2)    Engine torque-split factor, -
% Exogenous inputs
%   w(1)    Vehicle speed, m/s
%   w(2)    Vehicle acceleration, m/s^2
% Stage cost
%   stageCost   fuel flow rate, g/s
%
% Usage examples
% ---------------
% Basic usage
%   If SOC, GN, alpha_eng, vehSpd, vehAcc are all scalars, you can find the
%   SOC at the next timestep, fuel consumption, unfeasibilty flag as:
%       [SOC_next, fuelFlwRate, unfeas] = hev_model(SOC, [GN, alpha_eng], [vehSpd, vehAcc], veh)
%   SOC_next, fuelFlwRate and unfeas will be scalars.
%
% Returning additional time profiles
%   Also return the fourth to last outputs:
%       [SOC_next, fuelFlwRate, unfeas, engPrf, emPrf, battPrf, vehPrf] = hev_model( ... )
%   engPrf, emPrf, battPrf, vehPrf will be structures with additional time
%   profiles, (engine torque, battery current, ...).
%
%   If you are using this in a for loop, you will probably write something
%   like:
%       [SOC_next(k), fuelFlwRate(k), unfeas(k), engPrf(k), emPrf(k), battPrf(k), vehPrf(k)] = hev_model( ... )
%   In this case, engPrf, emPrf, battPrf, vehPrf will become a non-scalar
%   structure, which you may find hard to manipulate. In this case, convert
%   them to scalar structures containing arrays with structArray2struct:
%       engPrf = structArray2struct(engPrf)

%% Driveline (Vehicle + Final Drive + Transmission)
[shaftSpd, demTrq, vehPrf] = hev_drivetrain(w(1), w(2), u(1), veh);

%% Torque split
% Torque provided by engine
engTrq  = u(2) .* demTrq; % Nm
% Torque provided by electric motor
emShaftTrq  = ( 1 - u(2) ) .* demTrq; % Nm

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
engSpdUnfeas = ( engineState == 1 ) & ( ( engSpd < veh.eng.idleSpd & u(1)~=1 ) | ( engSpd > veh.eng.maxSpd ) );
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
battR = veh.batt.eqRes(x(1)); % ohm
% Battery voltage
battVoltage = veh.batt.ocv(x(1)); % V

% Battery current
battCurr = (battVoltage-sqrt(battVoltage.^2 - 4.*battR.*battPwr))./(2.*battR); % A
battCurr = real(battCurr);
% Current limits
maxChrgBattCurr = veh.batt.minCurr; % A
maxDisBattCurr = veh.batt.maxCurr; % A
% Saturate charge current in regen braking
battCurr = max(battCurr, maxChrgBattCurr);

% New battery state of charge
x_next(1)  = - battColumbicEff .* battCurr ./ (veh.batt.nomCap * 3600) .* veh.dt + x(1);
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
pwrFlw( u(2) == 0 ) = 'pe';
pwrFlw( u(2) > 0 & u(2) < 1 ) = 'ps';
pwrFlw( u(2) == 1 ) = 'pt';
pwrFlw( u(2) > 1 ) = 'bc';

%% Pack additional outputs
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
battPrf.battSOC = x(1);
battPrf.battCurr = battCurr;
battPrf.battVolt = battVoltage;
battPrf.battRes = battR;
vehPrf.gearNumber = u(1);
vehPrf.engAlpha = u(2);
vehPrf.pwrFlw = pwrFlw;
vehPrf.vehSpd = w(1);
vehPrf.vehAcc = w(2);
vehPrf.engUnfeas = engUnfeas;
vehPrf.emUnfeas = emUnfeas;
vehPrf.battUnfeas = battUnfeas;
vehPrf.pwtUnfeas = pwtUnfeas;
vehPrf.unfeas = unfeas;
