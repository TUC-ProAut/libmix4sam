function [experiment, experiments, experimentProperties] = loadRegistrationExperiment(workingFolder, varargin)
%LOADREGISTRATIONEXPERIMENT Load the data for the registration experiment
%   as well as the results itself.
%
%   The data is generated by the gen**Problem functions and stored
%   afterwards within the experiments folder named "setup_*".
%   This data will be loaded.
%
%   Additionally, the function checks for available results within the
%   given folder and loads them too.
%   In only specific results shal be loaded, use the function
%   LOADREGISTRATIONRESULT.
%
%   See also LOADREGISTRATIONRESULT.

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

p = inputParser;
% Include the used optimization properties into the results' structure?
addParameter(p,'includeProperties', false, @islogical);
% Include the experiment's timestamps also into the result structure?
% This affects only the current measurements, not the previous.
addParameter(p,'includeTimestamps', false, @islogical);
% Include cycle times into the result structure?
% For some real world datasets they may be rounded to kompensate for
% transmission problems.
addParameter(p,'includeCycleTimes', false, @islogical);
parse(p, varargin{:}); p = p.Results;

%% Locate the experiment data.
files = dir(fullfile(workingFolder,'*.json'));
idx = find(contains({files.name},'setup_'));
assert(numel(idx) <= 1,'To many experiments are within the folder! Only one should be present!');

%% Load experiment, if available within folder.
if isempty(idx)
    experiment = [];
    warning('No experiment data found!');
else
    [~,fname] = fileparts(files(idx).name);
    load(fullfile(files(idx).folder,fname), 'experiment');
    if nargout > 2
        experimentProperties = load(fullfile(files(idx).folder,fname), 'p'); 
        experimentProperties = experimentProperties.p;
    end
    files(idx) = [];
end

%% Load the optimization results.

% Initialize struct for results
experiments = struct('name',{},'data',[],'timing',[]);

% Loop over available files
for iFile=1:numel(files)
    [~,fname] = fileparts(files(iFile).name);
    l = load(fullfile(files(iFile).folder,fname));
    
    % Correct wrong dimensionalities
    if size(l.estimation(1).mean,2) > 1
        tmp = num2cell(reshape([l.estimation.mean],2,[]),1); % Zeilen- zu Spaltenvektor
        [l.estimation.mean] = tmp{:};
    end
    
    if ~isa(l.estimation(1).iterations,'uint64')
        tmp = num2cell(uint64([l.estimation.iterations]));
        [l.estimation.iterations] = tmp{:};
    end
    
    experiments(iFile).data = l.estimation;
    if isfield(l,'name')
        experiments(iFile).name = l.name;
    else
        experiments(iFile).name = 'N.N.';
    end
    if isfield(l,'timing')
        experiments(iFile).timing = l.timing;
    else
        experiments(iFile).timing = 0;
    end
    if p.includeProperties && isfield(l,'p')
        experiments(iFile).properties = l.p;
    end
    if p.includeTimestamps && ~isempty(experiment)
        tmp = num2cell(arrayfun(@(x)x.current.Time, experiment));
        [experiments(iFile).data.time] = tmp{:}; 
    end
    if p.includeCycleTimes && ~isempty(experiment)
        tmp = num2cell(arrayfun(@(x)x.current.CycleTime, experiment));
        [experiments(iFile).data.dt] = tmp{:}; 
    end
    % Take name from json, if available
    jsonPropFile = fullfile(files(iFile).folder,[fname '.json']);
    if exist(jsonPropFile,'file')
        jsonProps = jsondecode(fileread(jsonPropFile));
        experiments(iFile).name = jsonProps.resultName;
    end
end

%% Restructure for specific properties.

% If numeric calculation is selected, create an additional experiment for
% the numeric Hessian results
for iResult = 1:numel(experiments)
    if isfield(experiments(iResult).data, 'numericalCovar')
        experiments(end+1) = experiments(iResult);
        experiments(end).name = [experiments(end).name 'Num Hessian'];
        [experiments(end).data.covar] =  experiments(end).data.numericalCovar;
    end
end
