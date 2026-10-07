function [fig, ax] = emMapWithPF(em, prof, powerflows)
arguments
    em struct
    prof struct
    powerflows string {mustBeText, mustBeMember(powerflows, ["all", "pe", "pt", "ps", "bc"])}  = ["pe", "ps", "bc"]
end
%emMapWithPF
% Plot the e-machine efficiency map and the operating points, colored based
% on the powerflow.
%
% Input arguments
% ---------------
% em : struct
%   E-machine data structure.
% prof : struct
%   Time profiles data structure. It must contain at least the emPrf 
%   and vehPrf time profiles structures.
% powerflows : string, optional
%   Specify one or more powerflows to represent in a string array. Specify:
%       "pe"  for pure electric;
%       "pt"  for pure thermal;
%       "ps"  for power split;
%       "bc"  for battery charging;
%       "all" for all powerflows.
%   The default is ["pe", "ps", "bc"].
%
% Outputs
% ---------------
% fig : Figure
%   Figure handle of the plot.
% ax : Axes
%   Axes handle of the plot.
%
% Notes
% ---------------
% The powerflow control variable must be named either prof.vehPrf.engAlpha, 
% prof.vehPrf.engTau or prof.vehPrf.emAlpha.

%% Draw the EM efficiency map
[fig, ax] = emMapPlot(em);

% Check that the relevant profiles were provided
if ~isfield(prof, 'emPrf')
    error("The profiles structure does not contain 'emPrf'.")
end
if ~isfield(prof, 'vehPrf')
    error("The profiles structure does not contain 'vehPrf'.")
end

% Ensure all structures are scalar structures
prof.emPrf = structArray2struct(prof.emPrf);
prof.vehPrf = structArray2struct(prof.vehPrf);

% Some compatibilty operations
if isfield(prof.vehPrf, "engAlpha")
    pwrFlwCvName = "\alpha_{eng}";
    pwrFlwPrf = prof.vehPrf.engAlpha;
elseif isfield(prof.vehPrf, "engTau")
    pwrFlwCvName = "\tau_{eng}";
    pwrFlwPrf = prof.vehPrf.engTau;
elseif isfield(prof.vehPrf, "emAlpha")
    pwrFlwCvName = "\alpha_{em}";
    pwrFlwPrf = prof.vehPrf.emAlpha;
else
    pwrFlwCvName = "missing";
    pwrFlwPrf = nans(size(time));
end

%% Operating points scatter plot
if strcmp(powerflows, 'all')
    powerflows = ["pe", "pt", "ps", "bc"];
end

for n = 1:length(powerflows)
    powerflow = powerflows(n);
    idx = ismember(prof.vehPrf.pwrFlw, powerflow);
    switch powerflow
        case 'pe'
            color = 'g';
        case 'pt'
            color = 'r';
        case 'ps'
            color = 'b';
        case 'bc'
            color = 'k';
        otherwise
            continue
    end
    s = scatter(prof.emPrf.emSpd(idx) .* 30/pi, prof.emPrf.emTrq(idx), color, 'x', 'DisplayName', powerflow);
    
    % Custom datatip
    s.DataTipTemplate.DataTipRows(1).Label = 'Speed';
    s.DataTipTemplate.DataTipRows(2).Label = 'Torque';
    time = 0:1:(length(prof.vehPrf.vehSpd)-1);
    s.DataTipTemplate.DataTipRows(end+1) = dataTipTextRow('Time, s', time(idx));
    s.DataTipTemplate.DataTipRows(end+1) = dataTipTextRow('Gear Number', double(prof.vehPrf.gearNumber(idx)));
    s.DataTipTemplate.DataTipRows(end+1) = dataTipTextRow(pwrFlwCvName, pwrFlwPrf(idx));
end


%% Finalize figure
title(['EM operating points for ' strjoin(powerflows, ', ') ' powerflows'])

end