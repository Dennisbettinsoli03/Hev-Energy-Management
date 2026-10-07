function [fig, ax] = emMapPlot(em)
arguments
    em struct
end
%emMapPlot
% Plot the e-machine efficiency map.
%
% Input arguments
% ---------------
% em : struct
%   e-machine data structure.
%
% Outputs
% ---------------
% fig : Figure
%   Figure handle of the plot.
% ax : Axes
%   Axes handle of the plot.

%%
% Create fc, bsfc and fuel conv eff contour plot data
spdBrk = linspace(0, em.maxSpd, 200);
trqBrk = linspace(min(em.minTrq.Values), max(em.maxTrq.Values), 200);
[spdMesh, trqMesh] = ndgrid(spdBrk, trqBrk);

eta = em.effMap(spdMesh, trqMesh);

% Remove contour plot data outside limit curves
minTrq = em.minTrq(spdMesh);
maxTrq = em.maxTrq(spdMesh);

eta(trqMesh>maxTrq) = nan;
eta(trqMesh<minTrq) = nan;

contour_data = eta;
levels = [0.1:0.2:0.6 0.7 0.8 0.85 0.87 0.9 0.92:0.01:0.96 0.97:0.01:1];
contour_legend_string = 'Efficiency';
   

%% Plot em map
fig = figure;
ax = axes;
hold on
grid on

spdBrkRPM = spdBrk .* 30/pi;
spdMeshRPM = spdMesh .* 30/pi;

% contour plot
[c, h] = contour(ax, spdMeshRPM, trqMesh, contour_data, levels, 'ShowText', 'on', 'LabelSpacing', 1440);
clabel(c, h, 'LabelSpacing', 1440, 'color', 'k', 'FontWeight', 'bold')
caxis([0.7 1]);

% draw Tmax
minTrq = em.minTrq(spdBrk);
plot(ax, spdBrkRPM, minTrq, 'k', 'LineWidth', 2, 'HandleVisibility', 'off')

% draw Tmin
maxTrq = em.maxTrq(spdBrk);
plot(ax, spdBrkRPM, maxTrq, 'k', 'LineWidth', 2, 'HandleVisibility', 'off')

% null torque line
plot(ax, [min(spdBrkRPM) max(spdBrkRPM)], [0, 0], 'k', 'LineWidth', 1, 'HandleVisibility', 'off')

%% Finalize figure
xlabel('Speed, RPM')
ylabel('Torque, Nm')
legend(contour_legend_string)
ylim([min(minTrq) max(maxTrq)])
end