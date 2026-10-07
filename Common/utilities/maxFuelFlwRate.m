function fuelFlwRate_max = maxFuelFlwRate(eng)
%maxFuelFlwRate evaluate maximum fuel flow rate of the engine eng
%
% Input arguments
% ---------------
% eng : struct
%   data structure containing the engine data
%
% Outputs
% ---------------
% fuelFlwRate_max : double
%    maximum feasible fuel flow rate in the engine map

% Set up pattern search
% Initial guess
engSpd0 = 0.8 * eng.maxSpd;
engTrq0 = 0.8 * eng.maxTrq(engSpd0);

% Bounds
engSpdLo = 0;
engSpdUp = max(eng.maxSpd);
engTrqLo = 0;
engTrqUp = max(eng.maxTrq.Values);

% Objective function
fcFun = @(x) - eng.fuelMap(x(1), x(2));

% Solve minimization
[~, fuelFlwRate_max] = fmincon(fcFun, [engSpd0; engTrq0], [],[],[],[], [engSpdLo; engTrqLo], [engSpdUp; engTrqUp], @(x) mycon(x, eng), optimoptions('fmincon','Display','off'));
fuelFlwRate_max = - fuelFlwRate_max;

end

% Nonlinear constraints
function [c,ceq] = mycon(x, eng)

    engSpd = x(1);
    engTrq = x(2);

    % Compute inequality constraint (limit torque curve)
   trqConstr = engTrq - eng.maxTrq(engSpd); % <= 0
    c = trqConstr;
    
    % Compute equality constraints (none)
    ceq = [];
end