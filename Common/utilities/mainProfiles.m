function [fig, t] = mainProfiles(prof)
% mainProfiles(prof) 
%  draw post-processing plots.
%
% Input arguments
% ---------------
% prof : struct
%   data structures for the time profiles.
%
% Notes
% ---------------
% The powerflow control variable must be named either prof.vehPrf.engAlpha, 
% prof.vehPrf.engTau or prof.vehPrf.emAlpha.

%% Load info
% Retrieve time profiles
vehPrf = prof.vehPrf;
engPrf = prof.engPrf;
battPrf = prof.battPrf;

% Check that the relevant profiles were provided
if ~isfield(prof, 'engPrf')
    error("The profiles structure does not contain 'engPrf'.")
end
if ~isfield(prof, 'battPrf')
    error("The profiles structure does not contain 'battPrf'.")
end
if ~isfield(prof, 'vehPrf')
    error("The profiles structure does not contain 'vehPrf'.")
end

% Ensure all structures are scalar structures
engPrf = structArray2struct(engPrf);
battPrf = structArray2struct(battPrf);
vehPrf = structArray2struct(vehPrf);

time = 0:1:(length(vehPrf.vehSpd)-1);
time = time(:);

% Some compatibilty operations
if isfield(prof.vehPrf, "engAlpha")
    pwrFlwCvName = "\alpha_{eng}, -";
    pwrFlwPrf = prof.vehPrf.engAlpha;
elseif isfield(prof.vehPrf, "engTau")
    pwrFlwCvName = "\tau_{eng}, -";
    pwrFlwPrf = prof.vehPrf.engTau;
elseif isfield(prof.vehPrf, "emAlpha")
    pwrFlwCvName = "\alpha_{em}, -";
    pwrFlwPrf = prof.vehPrf.emAlpha;
else
    pwrFlwCvName = "missing";
    pwrFlwPrf = nans(size(time));
end


%% Speed, SOC, fuel consumption
fig = figure;
t = tiledlayout(5,1);
ax1 = nexttile;
plot(time, vehPrf.vehSpd, 'LineWidth', 1.5, 'Color', '#7f7f7f')
grid on
ylabel("Vehicle speed, m/s")

ax2 = nexttile;
plot(time, battPrf.battSOC, 'LineWidth', 1.5, 'Color', '#17becf')
grid on
ylabel("\sigma, -")

ax3 = nexttile;
plot(time, vehPrf.gearNumber, 'LineWidth', 1.5, 'Color', '#ff7f0e')
grid on
ylabel("Gear number, -")

ax4 = nexttile;
plot(time, pwrFlwPrf, 'LineWidth', 1.5, 'Color', '#2ca02c')
grid on
ylabel(pwrFlwCvName)

ax5 = nexttile;
plot(time, cumtrapz(time, engPrf.fuelFlwRate), 'LineWidth', 1.5, 'Color', '#8c564b')
grid on
ylabel("Fuel consumption, g")

xlabel(t, "Time, s", 'FontSize', 12)
linkaxes([ax1 ax2 ax3 ax4 ax5], 'x')
