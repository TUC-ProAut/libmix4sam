classdef CredibilityTests < matlab.mixin.Copyable
%CREDIBILITYTESTS Class for handling different credibility tests.
%
%   Naming of variables and Examples are based on:
%      "Measuring Estimator's Credibility: Noncredibility Index" (X. Rong Li)
%      "https://ieeexplore.ieee.org/document/4086056"
%
%   See also CREDIBILITYTESTSPLOT.

% @author Karim Haggag (TU Chemnitz, ET/IT, Prozessautomatisierung)
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
    
    properties
        Name            = [];   % Name of dataset or something alike.
        DegreeOfFreedom = [];   % For the nees test 
        Description     = [];   % Optional, long description
    end
    
    properties(SetAccess = protected, Hidden = true)
        IterationUpperBound   = [];
        IterationLowerBound   = [];
        TotalUpperBound       = [];
        TotalLowBound         = [];
        TotalNEES             = [];
        IterationValidity     = [];
        TotalTestValidity     = [];
    end
    
    properties (SetAccess = protected)
        % Read Only Properties

        GroundTruth     = [];   %                                 (N x Dim)
        EstimationMean  = [];   % Estimator's Estimate            (N x Dim)
        EstimationMSE   = [];   % Estimator's Covariance (MSE) P  (Dim x Dim x N)
        EstimatorBias   = [];   % Estimator's Bias \mu_0          (1/N x Dim)
        ActualBias      = [];   % Actual Bias \mu*                (1/N x Dim)
        ActualMSE       = [];   % Actual Covariance (MSE) P*      (Dim x Dim x 1/N)
        EstimationError = [];   % Estimation Error \tilde x       (N x Dim)
        
        NEES4Estimator            = [];
        NEES4CredibleEstimator    = [];
        IterationCredibilityRatio = []; % equation (2)
        TotalCredibilityRatio     = []; % geometric average
        Mean4ActualMSE            = [];

        % Results
        NCIndex                   = [];
        Inclination               = [];        
    end
    
    properties (Access = protected)
        UnmatchedArguments = {};   % For Superclass creation
        CopyMode = false;
    end
    
    methods
        
        function self = CredibilityTests(varargin)
        %CREDIBILITYTESTS Constructor.
            
            if nargin > 0 && isa(varargin{1}, 'CredibilityTests')
                % Construct by copy of given Instance of this class.
                % (using the copy method of matlab.mixin.Copyable will not
                % work within constructor!)
                C = metaclass(varargin{1});
                P = C.Properties;
                for k = 1:length(P)
                    if ~P{k}.Dependent
                        try % Unfortunately everytime, the getMethod for the property will be called, which won't work everytime.
                            self.(P{k}.Name) = varargin{1}.(P{k}.Name);
                        end
                    end
                end
            else
                p = inputParser;
                addOptional(p,'EstimationMean',[],@isnumeric);
                addOptional(p,'GroundTruth',[],@isnumeric);
                addParameter(p,'EstimatorMSE',[],@isnumeric);
                addParameter(p,'EstimationError',[],@isnumeric);
                addParameter(p,'ActualMSE',[],@isnumeric);
                addParameter(p,'ActualBias',[],@isnumeric);
                addParameter(p,'EstimatorBias',[],@isnumeric);
                addParameter(p,'Name',[],@ischar);
                p.KeepUnmatched = true;
                parse(p,varargin{:});
                self.UnmatchedArguments = reshape(horzcat(fieldnames(p.Unmatched),struct2cell(p.Unmatched))',1,[]);
                
                p = p.Results;

                self.EstimationMean = p.EstimationMean;
                self.GroundTruth = p.GroundTruth;
                self.EstimationMSE = p.EstimatorMSE;
                self.EstimationError = p.EstimationError;
                self.ActualMSE = p.ActualMSE;
                self.Name = p.Name;

                if isempty(p.ActualBias)
                    self.ActualBias = zeros(1,size(self.EstimationError,2));
                else
                    self.ActualBias = p.ActualBias;
                end
                if isempty(p.EstimatorBias)
                    self.EstimatorBias = zeros(1,size(self.EstimationError,2));
                else
                    self.EstimatorBias = p.EstimatorBias;
                end
            end
        end
        
        function [noncredibilityIndex, inclinationIndicator] = nci(self)
        %NCI Calculate noncredibility index and inclination indicator.
            
            if length(self) > 1 % check if  we have more than one instance 
                noncredibilityIndex = zeros(length(self),1);
                inclinationIndicator = zeros(length(self),1);
                for iSelf = 1:length(self)
                    [noncredibilityIndex(iSelf), inclinationIndicator(iSelf)] = self(iSelf).nci;
                end
            else
                epsilon = cellfun(@(xi,Pi) (xi*inv(Pi)*xi'), ...
                    num2cell(self.EstimationError - self.EstimatorBias ,2), ...
                    reshape(num2cell(self.EstimationMSE,[1 2]),[],1));
                epsilon_star = cellfun(@(xi) ((xi(:) - self.ActualBias(:))' * inv(self.Mean4ActualMSE)*(xi(:) - self.ActualBias(:))), ...
                    num2cell(self.EstimationError,2));
                rho = epsilon ./ epsilon_star;
                noncredibilityIndex = 10/length(rho) * sum(abs(log10(rho)));
                inclinationIndicator = 10/length(rho) * sum(log10(rho));
            end
        end
        
        function [anees, bounds] = ANEES(self, interval)
        %ANEES Calculate the ANEES and return bounds.
            if nargin < 2, interval = 0.95; end % Default: 95% interval
            dof = self.DegreeOfFreedom;
            nees = self.NEES4Estimator;
            N = length(nees);
            anees = 1/(dof*N) * sum(nees);
            int = (1-interval)/2;
            bounds = chi2inv([int 1-int], dof*N)/N/dof; 
        end

    end

    methods
        % - 1
        function [ret] = get.EstimationError(self)
            % Return / compute the error between the estimation and the ground truth.
            
            if isempty(self.GroundTruth) || isempty(self.EstimationMean)
                % Either ground truth or estimation mean is not available,
                % check if estimation error was given instead!
                if isempty(self.EstimationError)
                    error('Can not calculate estimation error!')
                end
            else
                if isempty(self.EstimationError)
                    % Estimation error not yet calculated. Do it now!
                    self.EstimationError = self.EstimationMean - self.GroundTruth;
                end
            end
            ret = self.EstimationError;
        end
        % - 2
        function ret = get.ActualMSE(self)
            % Return / compute the Mean Square Error 
            if isempty(self.ActualMSE)
                self.ActualMSE = cell2mat(reshape(cellfun(@(xi) (xi'*xi),num2cell(self.EstimationError,2),'UniformOutput', false),1,1,[]));
            end
            ret = self.ActualMSE ;
        end
        % - 3
        function ret = get.Mean4ActualMSE(self)
            % Return / compute the sample MSE 
            
            self.Mean4ActualMSE = mean(self.ActualMSE,3);
            ret = self.Mean4ActualMSE;
        end
        % - 4
        function ret = get.NEES4Estimator(self)
            % Return / compute epslion, which represents the NEES for the estimator.
            
            epsilon = cellfun(@(xi,Pi) (xi*inv(Pi)*xi'), ...
                num2cell(self.EstimationError,2), ...
                reshape(num2cell(self.EstimationMSE,[1 2]),[],1));        
            % Update NEES For the Estimator
            self.NEES4Estimator = epsilon; 
            ret = self.NEES4Estimator;
        end
        % - 5
        function ret = get.NEES4CredibleEstimator(self)
            % Return / compute epslion, which represents the NEES for a Credible estimator.
            
            x = self.EstimationError - self.ActualBias;
            P_inv = inv(self.Mean4ActualMSE);
            epsilon_star = cellfun(@(xi) (xi*P_inv*xi'), num2cell(x,2));
            
            % Update NEES For the Credible Estimator
            self.NEES4CredibleEstimator = epsilon_star;
            
            ret = self.NEES4CredibleEstimator;
        end
        % - 6
        function ret = get.IterationCredibilityRatio(self)
            % Return / compute the credibility ratio for each run of the Monte Carlo test.
            
            self.IterationCredibilityRatio = self.NEES4Estimator./self.NEES4CredibleEstimator;
            ret = self.IterationCredibilityRatio;
        end
        % - 7
        function ret = get.TotalCredibilityRatio(self)
            % Return / compute the credibility ratio for the whole Monte Carlo test.
            self.TotalCredibilityRatio = geomean(self.IterationCredibilityRatio);
            ret = self.TotalCredibilityRatio;
        end
        % - 8 noncredibility index (NCI)
        function ret = get.NCIndex(self)
            % Return / compute noncredibility index (NCI) -> equation(1.1)
            self.NCIndex   = 10 * mean (abs(log10(self.IterationCredibilityRatio)));
            ret = self.NCIndex;
        end
        % - 9
        function ret = get.Inclination(self)
            % Return / compute  inclination indicator (I2)  -> equation(1.2)
            self.Inclination = 10 * mean(log10(self.IterationCredibilityRatio));
            ret = self.Inclination;
        end
        
        function ret = get.DegreeOfFreedom(self)
            if isempty(self.DegreeOfFreedom)
                warning('Property "DegreeOfFreedom" was not set, using size of EstimationError.');
                self.DegreeOfFreedom = size(self.EstimationError,2);
            end
            ret = self.DegreeOfFreedom;
        end
        
       function ret = get.TotalTestValidity(self)
       %TOTALTESTVALIDITY Estimator efficiency test.
       %   See: Bar-Shalom, Y., Li, X.R. and Kirubarajan, T. (2001)
       %   Estimation with Applications to Tracking and Navigation:  Theory
       %   Algorithms and Software. 1st edn. New York, NY, USA: John Wiley
       %   & Sons, Inc.
       %   On page 168 for further details.
           
           % Check first if the DegreeOfFreedom is empty 
           if isempty(self.DegreeOfFreedom)
               error('You need to give the degree of freedom of you problem');
           end
           % Bound based on degree of freedom from the problem 
           self.IterationUpperBound = chi2inv(0.95,self.DegreeOfFreedom);
           self.IterationLowerBound = chi2inv(0.05,self.DegreeOfFreedom);
           
           % Bound based on degree of freedom from the problem and the number of Monte Carlo run
           % Compute Monte Carlo run from the size of the data 
           if isempty(self.EstimationMean)
               NumOfRuns = size(self.EstimationError,1);
           else
               NumOfRuns = size(self.EstimationMean,1);
           end
           
           self.TotalUpperBound = chi2inv(0.95,self.DegreeOfFreedom*NumOfRuns);
           self.TotalLowBound   = chi2inv(0.05,self.DegreeOfFreedom*NumOfRuns);
           
           % NEES for each iteration
           self.IterationValidity = cellfun(@(xi)(xi >  self.IterationLowerBound && xi < self.IterationUpperBound),...
               num2cell(self.NEES4Estimator,2));
           
           % Total NEES 
           self.TotalNEES = sum(self.NEES4Estimator);
           % Total NEES Test 
           self.TotalTestValidity = ( self.TotalNEES > self.TotalLowBound && self.TotalNEES < self.TotalUpperBound);
           
           ret = self.TotalTestValidity;
       end
       
    end

end

