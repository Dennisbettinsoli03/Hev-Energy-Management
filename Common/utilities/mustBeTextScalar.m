function mustBeTextScalar(text)
%mustBeTextScalar Validate that value is a single piece of text
%   mustBeTextScalar(TEXT) throws an error if TEXT is not a single piece of
%   text.
%
%   For string, a single piece of text is a 1x1 scalar. Empty double
%   quotes, "", and missing string are text scalars.
%
%   For char, a single piece of text is a row vector or the special case of
%   empty single quotes, ''.
%
%   See also mustBeText, mustBeNonzeroLengthText, validators.

% Copyright 2020-2021 The MathWorks, Inc.
% Taken from ver 2021b for backwards compatibility

    if ~isCharRowVector(text) && ~(isstring(text) && isscalar(text))
        throwAsCaller(MException(message("MATLAB:validators:mustBeTextScalar")))
    end

end

function tf = isCharRowVector(text)
    tf = ischar(text) && (isrow(text) || isequal(size(text),[0 0]));
end
