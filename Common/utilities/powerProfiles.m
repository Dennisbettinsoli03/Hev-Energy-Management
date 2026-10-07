function fig = powerProfiles(prof, components)
arguments
    prof struct
    components string {mustBeText, mustBeMember(components, ["all", "veh", "eng_em", "eng", "em", "em_mech", "em_el", "batt"])}  = ["veh", "eng", "em_mech"]
end
%simulationAnalysis 
% draw post-processing plots
%
% Input arguments
% ---------------
% prof : struct
%   data structures for the time profiles.
% components : string, optional
%   Specify one or more components to represent in a string array. Specify:
%       "veh" for the vehicle level (driving force * vehicle speed);
%       "eng" for the engine;
%       "em_mech" for the e-machine (mechanical power);
%       "em_el" for the e-machine (electrical power);
%       "eng_em" for the engine and e-machine (mechanical power) combined;
%       "batt" for the battery;
%       "all" for all components;
%   The default is ["veh", "eng", "em_mech"].

%% Some validation 
if ismember("all", components)
    components = ["veh", "eng_em", "eng", "em_mech", "em_el", "batt"];
end
if ismember("em", components)
    components(strcmp(components == "em")) = "em_mech";
end

%% Load info
% Retrieve time profiles
vehPrf = prof.vehPrf;
engPrf = prof.engPrf;
emPrf = prof.emPrf;
battPrf = prof.battPrf;

% Check that the relevant profiles were provided
if ~isfield(prof, 'engPrf')
    error("The profiles structure does not contain 'engPrf'.")
end
if ~isfield(prof, 'emPrf')
    error("The profiles structure does not contain 'emPrf'.")
end
if ~isfield(prof, 'battPrf')
    error("The profiles structure does not contain 'battPrf'.")
end
if ~isfield(prof, 'vehPrf')
    error("The profiles structure does not contain 'vehPrf'.")
end

% Ensure all structures are scalar structures
engPrf = structArray2struct(engPrf);
emPrf = structArray2struct(emPrf);
battPrf = structArray2struct(battPrf);
vehPrf = structArray2struct(vehPrf);

time = 0:1:(length(vehPrf.vehSpd)-1);
time = time(:);

%% Power profiles
fig = figure;
t = tiledlayout(2,1);

ax1 = nexttile;
grid on
hold on
powerflows = ["pe", "pt", "ps", "bc"];
colors = ["#2ca02c", "#d62728", "#1f77b4", "#7f7f7f"];
timePFs = reshape([time(1:end-1)'; time(2:end)'-eps], (length(time)-1)*2, 1);
vehSpdPFs = reshape([vehPrf.vehSpd(1:end-1)'; vehPrf.vehSpd(2:end)'], (length(time)-1)*2, 1);
powerflowPFs = reshape([vehPrf.pwrFlw'; vehPrf.pwrFlw'], (length(vehPrf.pwrFlw))*2, 1);
powerflowPFs(end-1:end) = [];
for n = 1:length(powerflows)
    vehSpd.(powerflows(n)) = vehSpdPFs;
    vehSpd.(powerflows(n))(~strcmp(powerflowPFs, powerflows(n))) = nan;
    plot(timePFs, vehSpd.(powerflows(n)), 'Color', colors(n), 'LineWidth', 1.5);
end

ylabel("Vehicle speed, m/s")
legend(powerflows, 'Location', 'best')

ax2 = nexttile;
hold on
grid on
if ismember("veh", components)
    plot(time, vehPrf.vehSpd .* vehPrf.vehForce, 'LineWidth', 1.5, 'Color', '#7f7f7f', 'DisplayName', "vehicle")
end
if ismember("eng_em", components)
    plot(time, engPrf.engSpd .* engPrf.engTrq + emPrf.emSpd .* emPrf.emTrq, 'LineWidth', 1.5, 'Color', '#ff7f0e', 'DisplayName', "engine + em")
end
if ismember("eng", components)
    plot(time, engPrf.engSpd .* engPrf.engTrq, 'LineWidth', 1.5, 'Color', '#d62728', 'DisplayName', "engine")
end
if ismember("em_mech", components)
    plot(time, emPrf.emSpd .* emPrf.emTrq, 'LineWidth', 1.5, 'Color', '#2ca02c', 'DisplayName', "em mechanical")
end
if ismember("em_el", components)
    plot(time, emPrf.emElPwr, 'LineWidth', 1.5, 'Color', '#1f77b4', 'DisplayName', "em electrical")
end
if ismember("batt", components)
    plot(time, battPrf.battVolt .* battPrf.battCurr, 'LineWidth', 1.5, 'Color', '#17becf', 'DisplayName', "battery")
end
% plot(time, engPrf.engSpd .* vehPrf.reqTrq, '--', 'LineWidth', 1.5, 'Color', '#9467bd')

ylabel("Power, W")
legend('Location', 'best')

xlabel(t, "Time, s", 'FontSize', 9)
linkaxes([ax1,ax2], 'x')