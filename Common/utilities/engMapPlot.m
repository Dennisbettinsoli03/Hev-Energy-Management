function [fig, ax] = engMapPlot(eng, contour_type)
arguments
    eng struct
        contour_type {mustBeTextScalar, mustBeMember(contour_type, ["fc", "bsfc", "eff"])}  = "bsfc"
end
%engMapPlot
% Plot the engine fuel consumption, brake specific fuel consumption or
% efficiency map.
%
% Input arguments
% ---------------
% eng : struct
%   engine data structure.
% contour_type : string, optional
%   Specify: 'fc' to plot the fuel flow rate map;
%            'bsfc' (default) for brake specific fuel consumption;
%            'eff' for fuel conversion efficiency.
%
% Outputs
% ---------------
% fig : Figure
%   Figure handle of the plot.
% ax : Axes
%   Axes handle of the plot.

%%
% Create fc, bsfc and fuel conv eff contour plot data
spdBrk = linspace(eng.idleSpd, eng.maxSpd, 200);
trqBrk = linspace(eng.fuelMap.GridVectors{2}(1), eng.fuelMap.GridVectors{2}(end), 220);
[spdMesh, trqMesh] = ndgrid(spdBrk, trqBrk);

fc = eng.fuelMap(spdMesh, trqMesh); % g/s

% Remove contour plot data above WOT curve
maxTrq = eng.maxTrq(spdMesh);
fc(trqMesh>maxTrq) = nan;

bsfc = eng.bsfcMap(spdMesh, trqMesh);
LHV = 43.4e6; % MJ/kg, gasoline
eff = 1 ./ (bsfc .* LHV) .*3.6e9;

switch contour_type
    case 'fc'
        contour_data = fc;
        levels = linspace(min(fc(:)), max(fc(:)), 20);
        levels = round(levels, 2);
        contour_legend_string = 'Fuel consumption, g/s';
        colorScale = [levels(1) levels(end)];
    case 'bsfc'
        contour_data = bsfc;
        levels = [200:10:250,275:25:375,400,450,500,600,1000];
        contour_legend_string = 'bsfc, g/kWh';
        colorScale = [min(bsfc(:)) 400];
    case 'eff'
        contour_data = eff;
        levels = [0.5, 0.1, 0.2, 0.25, 0.28:0.02:0.4];
        contour_legend_string = 'Efficiency';
        colorScale = [0.1 max(eff(:))];
end

%% Plot engine map
fig = figure;
ax = axes;
hold on
grid on

spdBrkRPM = spdBrk .* 30/pi;
spdMeshRPM = spdMesh .* 30/pi;

% draw WOT
maxTrq = eng.maxTrq(spdBrk);
plot(ax, spdBrkRPM, maxTrq, 'k', 'LineWidth', 2, 'HandleVisibility', 'off')

% null torque line
plot(ax, [min(spdBrkRPM) max(spdBrkRPM)], [0, 0], 'k', 'LineWidth', 1, 'HandleVisibility', 'off')

% motoring curve
if isfield(eng, 'motTrq')
    minTrq = eng.motTrq(spdBrk);
else
    minTrq = zeros(size(spdBrk));
end
plot(spdBrkRPM, minTrq, 'k', 'LineWidth', 2, 'HandleVisibility', 'off')

% contour plot
[c, h] = contour(ax, spdMeshRPM, trqMesh, contour_data, levels, 'ShowText', 'on', 'LabelSpacing', 1440);
clabel(c, h, 'LabelSpacing', 1440, 'color', 'k', 'FontWeight', 'bold')
caxis(colorScale)

% draw OOL
oolTrq = eng.oolTrq(spdBrk);
plot(ax, spdBrkRPM, oolTrq, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 2)

%% Finalize figure
xlim([eng.idleSpd eng.maxSpd].*30/pi)
ylim([min(minTrq) max(trqBrk)])
    
ylims = ylim;
fill([spdBrkRPM spdBrkRPM(end) spdBrkRPM(1)], [maxTrq ylims(2) ylims(2)], 0.9*ones(1,3))
fill([spdBrkRPM spdBrkRPM(end) spdBrkRPM(1)], [minTrq ylims(1) ylims(1)], 0.9*ones(1,3)) 
xlabel('Speed, RPM')
ylabel('Torque, Nm')
legend(contour_legend_string, 'OOL')

end