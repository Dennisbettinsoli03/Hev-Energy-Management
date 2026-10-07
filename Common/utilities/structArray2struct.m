function S2 = structArray2struct(S1)
%structArray2struct converts non-scalar to scalar struct
% Converts a non-scalar structure into a scalar structure containg arrays.
% For example:
%   engPrf = structArray2struct(engPrf)

fields = string( fieldnames(S1) );

if isscalar(S1)
    % The input is already a scalar struct; simply ensure alla fields are
    % column vectors
    for n = 1:length(fields)
        vec = S1.(fields(n));
        S2.(fields(n)) = vec(:);
    end
else
    % The input is a nonscalar struct, and needs to be converted
    for n = 1:length(fields)
        vec = [S1.(fields(n))];
        S2.(fields(n)) = vec(:);
    end

end