function [veh] = scaleVehData(veh, engPwr, emPwr, battEn)
%scaleVehData
% Rescale the engine, e-machine and battery data.
%
% Input arguments
% ---------------
% veh : struct
%   structure containing all parameters for the original powertain components
% engPwr : double
%   Rated power (W) of the rescaled engine.
% emPwr : double
%   Rated power (W) of the rescaled e-machine.
% batt : double
%   Battery Nominal Energy (Wh) of the rescaled battery.
%
% Outputs
% ---------------
% veh : struct
%   structure containing all parameters for the scaled powertain components

%% Run some checks
if nargout == 0
    warning("You did not return any output; therefore, you are not overwiting your eng, em and/or batt strcutures. Are you sure this is intended?")
end

if engPwr > 10e5
    warning("You specified an engine power of more than 100 kW. Are you sure this is intended?")
end
if engPwr < 1e4
    warning("You specified an engine power of less than 10 kW. Are you sure this is intended?")
end

if emPwr > 10e5
    warning("You specified an e-machine power of more than 100 kW. Are you sure this is intended?")
end
if emPwr < 1e3
    warning("You specified an e-machine power of less than 1 kW. Are you sure this is intended?")
end

if battEn > 10e3
    warning("You specified a battery nominal energy of more than 10 kWh. Are you sure this is intended?")
end
if battEn < 1e3
    warning("You specified a battery nominal energy of less than 1 kWh. Are you sure this is intended?")
end

%% Rescale the components
veh.body.mass = veh.body.mass - ( veh.eng.mass + veh.em.mass + veh.batt.mass );
veh.eng = scaleEngData(veh.eng, engPwr);
veh.em = scaleEmData(veh.em, emPwr);
veh.batt = scaleBattData(veh.batt, battEn);
veh.body.mass = veh.body.mass + ( veh.eng.mass + veh.em.mass + veh.batt.mass );

end

function eng = scaleEngData(eng, rescaled_engPwr)

    % Scaling factor
    scalingFactor = rescaled_engPwr/eng.maxPwr;
    
    % main data
    eng.maxPwr = rescaled_engPwr;
    if isfield(eng, 'inertia')
        eng.inertia = eng.inertia * scalingFactor;
    end
    if isfield(eng, 'mass')
        eng.mass = eng.mass * scalingFactor;
    end
    % Torque limit map
    eng.maxTrq.Values = eng.maxTrq.Values * scalingFactor;
    if isfield(eng, 'motTrq')
        eng.motTrq.Values = eng.motTrq.Values * scalingFactor;
    end
    if isfield(eng, 'oolTrq')
        eng.oolTrq.Values = eng.oolTrq.Values * scalingFactor;
    end
    
    % Map data
    eng.fuelMap.GridVectors{2} = eng.fuelMap.GridVectors{2} * scalingFactor;
    eng.fuelMap.Values = eng.fuelMap.Values * scalingFactor;
    eng.bsfcMap.GridVectors{2} = eng.bsfcMap.GridVectors{2} * scalingFactor;
end

function em = scaleEmData(em, rescaled_emPwr)
    
    % Scaling factor
    scalingFactor = rescaled_emPwr/em.maxPwr;
    
    % main data
    em.maxPwr = rescaled_emPwr;
    if isfield(em, 'inertia')
        em.inertia = em.inertia * scalingFactor;
    end
    if isfield(em, 'mass')
        em.mass = em.mass * scalingFactor;
    end
    
    % Torque limit maps
    em.maxTrq.Values = em.maxTrq.Values * scalingFactor;
    em.minTrq.Values = em.minTrq.Values * scalingFactor;
    
    % Map data
    em.effMap.GridVectors{2} = em.effMap.GridVectors{2} * scalingFactor;
end

function batt = scaleBattData(batt, rescaled_battEgy)
    % scaleBattMap
    % Scale the batt map, assuming scaling means adding/removing units
    % in parallel
    
    % Scaling factor
    scalingFactor = rescaled_battEgy / batt.nomEnergy;
    
    % Rescale main data
    batt.nomEnergy = rescaled_battEgy;
    batt.nomCap = batt.nomCap * scalingFactor;
    if isfield(batt, 'mass')
        batt.mass = batt.mass * scalingFactor;
    end
    
    % Rescale Req and limit current and power characteristics
    batt.eqRes.Values = batt.eqRes.Values ./ scalingFactor;
    batt.maxCurr = batt.maxCurr .* scalingFactor;
    batt.minCurr = batt.minCurr .* scalingFactor;
end
