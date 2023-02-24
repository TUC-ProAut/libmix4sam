function [] = configurePath(varargin)
%CONFIGUREPATH Add gtsam and libmix4sam to matlab path.
%   General function to add path of gtsam and libmix4sam to matlab path.
%   Usually needed in every startup.m file!
%
% We don't use some install directories here. We expect to have the
% binaries of libmix4sam within a build folder in the source tree. 
% We expect to find the sources of gtsam in a folder parallel to the build
% folder: 
%   <somedir>/<somePrefix>build<someSuffix> ... build folder
%   <somedir>/<somePrefix>src<someSuffix>   ... source folder
%
% INPUT:
%  libSrcPath ... Directory to the source tree of libmix4sam.
% INPUT (optional) Argumen-Value:
%  useLddPath ... enable/disable automatic gtsam detection
%  useUnstable ... enable/disable unstable branch of gtsam

% @author Sven Lange (TU Chemnitz, ET/IT, Prozessautomatisierung)

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
addOptional(p, 'libSrcPath', fullfile(getenv('HOME'),'workspace','libisf2','libmix4sam'), @(x)exist(x,'Dir'));
addParameter(p, 'useLddPath', true, @islogical);
addParameter(p, 'useUnstable', true, @islogical);
parse(p,varargin{:});
p = p.Results;

% Where to find libmix4sam sources
libSrcPath = p.libSrcPath;

% Add binaries and automatically generated scripts (Matlab-Wrapper)
% of libmix4sam to path
customAddPath( fullfile(libSrcPath,'build','wrap','libmix4sam') );
customAddPath( fullfile(libSrcPath,'build','wrap','libmix4sam_mex') );

% Do the same with the unstable branch, if available
unstablePath = fullfile(libSrcPath,'build','wrap','libmix4sam_unstable_mex');
if exist(unstablePath, 'Dir'), customAddPath( unstablePath ); end
unstablePath = fullfile(libSrcPath,'build','wrap','libmix4sam_unstable');
if exist(unstablePath, 'Dir'), customAddPath( unstablePath ); end


% Find out, to which gtsam the current libmix4sam is linked to:
[gtsamSrcFolder, gtsamBuildFolder] = findGtsamPath(p.useLddPath);

% We include gtsam via it's build and source folder.
customAddPath( fullfile(gtsamBuildFolder,'wrap','gtsam_mex') );
customAddPath( fullfile(gtsamBuildFolder,'wrap','gtsam') );
customAddPath( fullfile(gtsamSrcFolder,'matlab') );
customAddPath( fullfile(gtsamSrcFolder,'matlab','gtsam_examples') );

if p.useUnstable
    unstablePath1 = fullfile(gtsamBuildFolder,'gtsam_unstable','wrap','gtsam_unstable_mex');
    unstablePath2 = fullfile(gtsamBuildFolder,'gtsam_unstable','wrap','gtsam_unstable');
    if exist(unstablePath1, 'Dir') && exist(unstablePath2, 'Dir')
        customAddPath( fullfile(gtsamSrcFolder,'matlab','unstable_examples') );
        customAddPath( unstablePath1 );
        customAddPath( unstablePath2 );
    end
end

% Add further depending toolboxes from libmix4sam's matlab path:
    
end

function customAddPath(p)
    fprintf('libmix4sam.configurePath\n --> adding to path: %s\n',p);
    path(path,p);
end

function [src,build] = findGtsamPath(autodetect)
    [~, ldd_out] = system(['ldd ' which('libmix4sam_wrapper')]); 
    build = string(regexp(ldd_out, 'libgtsam.*=> (.*)gtsam/libgtsam', 'tokens','once') );
    if ~isempty(build) && autodetect
        % We expect, that the source is in parallel to the build folder!
        src = strrep(build, 'build', 'src');
    else
        if autodetect, warning('configurePath was called with automatic detection, but finding the gtsam path did not work! Trying default path instead.'); end
        % Try default folders
        workspacePath = fullfile(getenv('HOME'),'workspace');
        build = fullfile(workspacePath,'gtsam','current_build');
        src = fullfile(workspacePath,'gtsam','current_src');
    end
end
