function GN = gearControl(vehSpd, GN0, upSpd, downSpd)
%gearControl rule-based transmission controller
%   Implements a simple gear shift schedule based on the vehicle speed.
%
% Input arguments
% ---------------
% vehSpd : double
%   current value of the vehicle speed (m/s)
% GN0 : double
%   previous value of the gear number
% upSpd : double
%   the vector of upshift speeds, e.g. [v_{1->2}, v_{2->3}, ...]
% upSpd : double
%   the vector of downshift speeds, e.g. [v_{2->1}, v_{3->2}, ...]
%
% Outputs
% ---------------
% GN : double
%   new value of the gear number
%
% Usage example
% ---------------
% Load the upSpd and downSpd vectors from "transmControlData.mat"; then,
% simply evaluate
%   GN = gearControl(vehSpd, GN0, upSpd, downSpd)
% where vehSpd and GN0 are scalars; GN will be a scalar as well.

if vehSpd > upSpd(GN0)
    GN = GN0+1;
elseif vehSpd < downSpd(GN0)
    GN = GN0-1;
else
    GN = GN0;
end

end