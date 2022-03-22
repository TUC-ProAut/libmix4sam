function [estimation, timing] = loadRegistrationResult(p)
%LOADREGISTRATIONRESULT Load previously saved data for a specific 
%   registration problem.
%   
%   See also SAVEREGISTRATIONRESULT.

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

estimation = [];
timing = [];

% We dont want to save the following parameters as they may be changed
% without having influence on the generated data
useCached = p.useCached; 
workingFolder = p.workingFolder;
p = rmfield(p,{'useCached','workingFolder','resultName'}); 

if ~isempty(workingFolder)
    fname = fullfile(workingFolder, [libmix4sam.utils.getVariableChecksum(p) '.mat']);
    if exist(fname, 'file')
        if strcmp(useCached,'ask')
            answer = questdlg('Experiment already done. Do you want to redo it?', ...
                'Redo experiment?', ...
                'Redo', 'Skip','Skip');
        end
        if strcmp(useCached,'always') || strcmp(answer,'Skip')
            warning('Experiment already done. Loading old results.');
            data = load(fname);
            estimation = data.estimation;
            timing = data.timing;
            return; 
        end
    end
end

end

