function engMapWithPF(eng, prof, contour_type, powerflows)
arguments
    eng struct
    prof struct
    contour_type {mustBeTextScalar, mustBeMember(contour_type, ["fc", "bsfc", "eff"])}  = "bsfc"
    powerflows string {mustBeText, mustBeMember(powerflows, ["all", "pe", "pt", "ps", "bc"])}  = ["pt", "ps", "bc"]
end
%engMapPlot
% Plot the engine fuel consumption, brake specific fuel consumption or
% efficiency map.
%
% Input arguments
% ---------------
% eng : struct
%   Engine data structure.
% prof : struct
%   Time profiles data structure. It must contain at least the engPrf 
%   and vehPrf time profiles structures.
% contour_type : string, optional
%   Specify: "fc" to plot the fuel flow rate map;
%            "bsfc" (default) for brake specific fuel consumption;
%            "eff" for fuel conversion efficiency.
% powerflows : string, optional
%   Specify one or more powerflows to represent in a string array. Specify:
%       "pe"  for pure electric;
%       "pt"  for pure thermal;
%       "ps"  for power split;
%       "bc"  for battery charging;
%       "all" for all powerflows.
%   The default is ["pt", "ps", "bc"].
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

%% Plot engine map
[fig, ax] = engMapPlot(eng, contour_type);

% Check that the relevant profiles were provided
if ~isfield(prof, 'engPrf')
    error("The profiles structure does not contain 'engPrf'.")
end
if ~isfield(prof, 'vehPrf')
    error("The profiles structure does not contain 'vehPrf'.")
end

% Ensure all structures are scalar structures
prof.engPrf = structArray2struct(prof.engPrf);
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
if any(strcmp(powerflows, 'all'))
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
    s = scatter(prof.engPrf.engSpd(idx) .* 30/pi, prof.engPrf.engTrq(idx), color, 'x', 'DisplayName', powerflow);
    
    % Custom datatip
    s.DataTipTemplate.DataTipRows(1).Label = 'Speed';
    s.DataTipTemplate.DataTipRows(2).Label = 'Torque';
    time = 0:1:(length(prof.vehPrf.vehSpd)-1);
    s.DataTipTemplate.DataTipRows(end+1) = dataTipTextRow('Time, s', time(idx));
    s.DataTipTemplate.DataTipRows(end+1) = dataTipTextRow('Gear Number', double(prof.vehPrf.gearNumber(idx)));
    s.DataTipTemplate.DataTipRows(end+1) = dataTipTextRow(pwrFlwCvName, pwrFlwPrf(idx));
end


%% Finalize figure
title(['Engine operating points for ' strjoin(powerflows, ', ') ' powerflows'])

end