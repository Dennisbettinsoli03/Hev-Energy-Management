function mustBeText(text)
%mustBeText Validate that value is string, char, or cellstr
%   mustBeText(TEXT) throws an error if TEXT is not a string, char, or
%   cellstr. For char, the shape must be row vector. 
%
% Copyright 2020 The MathWorks, Inc.
% Taken from ver 2021b for backwards compatibility

    if ~istext(text)
        throwAsCaller(MException(message("MATLAB:validators:mustBeText")))
    end

end

function tf = istext(text)
    tf = isCharRowVector(text) || isstring(text) || ...
         iscell(text) && matlab.internal.datatypes.isCharStrings(text);
end

function tf = isCharRowVector(text)
    tf = ischar(text) && (isrow(text) || isequal(size(text),[0 0]));
end