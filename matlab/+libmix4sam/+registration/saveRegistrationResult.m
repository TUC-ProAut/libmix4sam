function [] = saveRegistrationResult(p, estimation, timing)
%SAVEREGISTRATIONPROBLEM Save the data for a registration problem.
%   
%   See also LOADREGISTRATIONRESULT

% @author Sven Lange

% This file is part of
% libmix4sam - Mixtures for Smoothing and Mapping Library
%
% Copyright (C) 2020 Chair of Automation Technology / TU Chemnitz
% For more information see https://mytuc.org/mix
%
% libmix4sam is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% libmix4sam is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with this software.  If not, see <http://www.gnu.org/licenses/>.
%
% Contact Information: Sven Lange (sven.lange@etit.tu-chemnitz.de)

% The following properties are not used for calculating the checksum for
% the filename:
workingFolder = p.workingFolder;
resultName = p.resultName;
p = rmfield(p,{'useCached','workingFolder','resultName'}); 

if ~isempty(workingFolder)
    if ~exist(workingFolder,'dir'), mkdir(workingFolder); end
    assert(~isempty(resultName),'Could not save the results, because no name is given!')
    name = resultName;
    fname = fullfile(workingFolder, libmix4sam.utils.getVariableChecksum(p));
    save([fname '.mat'],'name','p','timing','estimation');
    % Have the properties human readable too.
    pp = p; pp.resultName = resultName; % We don't want the resultName within the checksum, but we want to have it in the json for convenience.
    libmix4sam.utils.saveAsJson([fname '.json'],pp);
end

end

