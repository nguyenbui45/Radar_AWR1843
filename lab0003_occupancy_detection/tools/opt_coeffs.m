%
% Copyright (c) 2018 Texas Instruments Incorporated
%
% All rights reserved not granted herein.
% Limited License.
%
% Texas Instruments Incorporated grants a world-wide, royalty-free,
% non-exclusive license under copyrights and patents it now or hereafter
% owns or controls to make, have made, use, import, offer to sell and sell ("Utilize")
% this software subject to the terms herein.  With respect to the foregoing patent
% license, such license is granted  solely to the extent that any such patent is necessary
% to Utilize the software alone.  The patent license shall not apply to any combinations which
% include this software, other than combinations with devices manufactured by or for TI ("TI Devices").
% No hardware patent is licensed hereunder.
%
% Redistributions must preserve existing copyright notices and reproduce this license (including the
% above copyright notice and the disclaimer and (if applicable) source code license limitations below)
% in the documentation and/or other materials provided with the distribution
%
% Redistribution and use in binary form, without modification, are permitted provided that the following
% conditions are met:
%
%             * No reverse engineering, decompilation, or disassembly of this software is permitted with respect to any
%               software provided in binary form.
%             * any redistribution and use are licensed by TI for use only with TI Devices.
%             * Nothing shall obligate TI to provide you with source code for the software licensed and provided to you in object code.
%
% If software source code is provided to you, modification and redistribution of the source code are permitted
% provided that the following conditions are met:
%
%   * any redistribution and use of the source code, including any resulting derivative works, are licensed by
%     TI for use only with TI Devices.
%   * any redistribution and use of any object code compiled from the source code and any resulting derivative
%     works, are licensed by TI for use only with TI Devices.
%
% Neither the name of Texas Instruments Incorporated nor the names of its suppliers may be used to endorse or
% promote products derived from this software without specific prior written permission.
%
% DISCLAIMER.
%
% THIS SOFTWARE IS PROVIDED BY TI AND TI'S LICENSORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
% BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
% IN NO EVENT SHALL TI AND TI'S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
% OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
% OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.

% NOTE: For copyright reasons, we cannot distribute the MatLab function fmincg(). It can be freely
% downloaded here:
%
% https://www.mathworks.com/matlabcentral/mlc-downloads/downloads/submissions/56393/versions/1/previews/mathwork/fmincg.m/index.html
%
% Please insert this function from the above location, and build this tool using MatLab 2017a or compatible
% release.

% -----------------------------------------------------------------------
% Optimize the coefficients for zone-based occupancy detection
% using the measurement data. This generates an output file which
% contains optimized coefficients.
%
% dataListFile: a file containing zone definitions and the list of saved .mat files to process
% zonePair: For 4 or more zones, this is the "zone pair" number, else 0
% winLen: window length used in feature extraction
% lambda: regularization parameter used in optimization routine
%
% -----------------------------------------------------------------------
function [] = opt_coeffs(dataListFile, zonePair, windowLen, lambda)
  if (nargin ~= 4)
    error('Incorrect number of arguments\n');
  end

z_pair  = str2num(zonePair);
winLen  = str2num(windowLen);
lambda  = str2num(lambda);

%% load data
[X, y]  = build_data(dataListFile, windowLen);

% number of examples
m       = size(X, 1);

%% Feature normalization
[X, mu, sigma] = featureNormalize(X);

% Add ones for bias term
X       = [ones(m, 1), X];

classTags = unique(y);
all_theta = zeros(size(X,2), length(classTags));

if length(classTags)~=4 % for two zones
  error('Some occupancy state(s) missing. There are four occupancy states for two zones. Expect all the occupancy states are covered.');
end

%% Regularized Logistic Regression ========================
for iClass = 1:length(classTags)

    % fprintf('logRegress_all: lambda = %f, winLen = %d, ClassTag = %d\n', lambda, winLen, classTags(iClass));

    % one vs all
    y_new           = (y==classTags(iClass));

    % Initialize fitting parameters
    initial_theta   = zeros(size(X, 2), 1);

    % Set options for fmincg
    options.GradObj = 'on';
    options.MaxIter = 200;

    % Create "short hand" for the cost function to be minimized
    costFunction    = @(t) costFunctionReg(t, X, y_new, lambda);

    % Run fmincg to obtain the optimal theta
    all_theta(:,iClass) = fmincg(costFunction, initial_theta, options);

end

%% save the coefficients into .txt file
%outFilename = sprintf('coeffs_winLen=%d_lambda=%.2f.txt', winLen, lambda);
%fid = fopen(outFilename, 'w');
for classIdx = 1:size(all_theta,2)
    fprintf('coeffMatrixRow  %d  %d  %s\n', z_pair, classIdx-1, num2str(all_theta(:,classIdx).', '%10.6f  '));
end
fprintf('meanVector  %d  %s\n', z_pair, num2str(mu,    '%10.6f  '));
fprintf('stdVector   %d  %s\n', z_pair, num2str(sigma, '%10.6f  '));
%fclose(fid);
%fprintf('%s saved\n', outFilename);

end

% -----------------------------------------------------------------------
function [J, grad] = costFunctionReg(theta, X, y, lambda)
% COSTFUNCTIONREG Compute cost and gradient for logistic regression
% with regularization

% initialize
m       = length(y); % number of training examples
J       = 0;
grad    = zeros(size(theta));

% calculate cost
h       = sigmoid(X*theta);
J       = -1/m * sum( y .* log(h) + (1-y) .* log(1 - h) ) + ...
            lambda/(2*m) * sum(theta(2:end).^2);

% calculate gradient
grad        = (1/m) * (X' * (h - y));
grad(2:end) = grad(2:end) + lambda/m*theta(2:end);

end

% -----------------------------------------------------------------------
function g = sigmoid(z)
% SIGMOID Compute sigmoid function

g = 1 ./ (1 + exp(-z));

end

% -----------------------------------------------------------------------
function [X_norm, mu, sigma] = featureNormalize(X)
% FEATURENORMALIZE Normalizes the features in X

mu      = mean(X);
X_norm  = bsxfun(@minus, X, mu);

sigma   = std(X_norm);
X_norm  = bsxfun(@rdivide, X_norm, sigma);

end

% -----------------------------------------------------------------------
function [X, fX, i] = fmincg(f, X, options, P1, P2, P3, P4, P5)
% Minimize a continuous differentialble multivariate function. Starting point
% is given by "X" (D by 1), and the function named in the string "f", must
% return a function value and a vector of partial derivatives. The Polack-
% Ribiere flavour of conjugate gradients is used to compute search directions,
% and a line search using quadratic and cubic polynomial approximations and the
% Wolfe-Powell stopping criteria is used together with the slope ratio method
% for guessing initial step sizes. Additionally a bunch of checks are made to
% make sure that exploration is taking place and that extrapolation will not
% be unboundedly large. The "length" gives the length of the run: if it is
% positive, it gives the maximum number of line searches, if negative its
% absolute gives the maximum allowed number of function evaluations. You can
% (optionally) give "length" a second component, which will indicate the
% reduction in function value to be expected in the first line-search (defaults
% to 1.0). The function returns when either its length is up, or if no further
% progress can be made (ie, we are at a minimum, or so close that due to
% numerical problems, we cannot get any closer). If the function terminates
% within a few iterations, it could be an indication that the function value
% and derivatives are not consistent (ie, there may be a bug in the
% implementation of your "f" function). The function returns the found
% solution "X", a vector of function values "fX" indicating the progress made
% and "i" the number of iterations (line searches or function evaluations,
% depending on the sign of "length") used.
%
% Usage: [X, fX, i] = fmincg(f, X, options, P1, P2, P3, P4, P5)
%
% See also: checkgrad
%
% Copyright (C) 2001 and 2002 by Carl Edward Rasmussen. Date 2002-02-13
%
%
% (C) Copyright 1999, 2000 & 2001, Carl Edward Rasmussen
%
% Permission is granted for anyone to copy, use, or modify these
% programs and accompanying documents for purposes of research or
% education, provided this copyright notice is retained, and note is
% made of any changes that have been made.
%
% These programs and documents are distributed without any warranty,
% express or implied.  As the programs were written for research
% purposes only, they have not been tested to the degree that would be
% advisable in any important application.  All use of these programs is
% entirely at the user's own risk.
%
% [ml-class] Changes Made:
% 1) Function name and argument specifications
% 2) Output display
%

% Read options
if exist('options', 'var') && ~isempty(options) && isfield(options, 'MaxIter')
    length = options.MaxIter;
else
    length = 100;
end

RHO = 0.01;                            % a bunch of constants for line searches
SIG = 0.5;       % RHO and SIG are the constants in the Wolfe-Powell conditions
INT = 0.1;    % don't reevaluate within 0.1 of the limit of the current bracket
EXT = 3.0;                    % extrapolate maximum 3 times the current bracket
MAX = 20;                         % max 20 function evaluations per line search
RATIO = 100;                                      % maximum allowed slope ratio

argstr = ['feval(f, X'];                      % compose string used to call function
for i = 1:(nargin - 3)
  argstr = [argstr, ',P', int2str(i)];
end
argstr = [argstr, ')'];

if max(size(length)) == 2, red=length(2); length=length(1); else red=1; end
S=['Iteration '];

i = 0;                                            % zero the run length counter
ls_failed = 0;                             % no previous line search has failed
fX = [];
[f1 df1] = eval(argstr);                      % get function value and gradient
i = i + (length<0);                                            % count epochs?!
s = -df1;                                        % search direction is steepest
d1 = -s'*s;                                                 % this is the slope
z1 = red/(1-d1);                                  % initial step is red/(|s|+1)

while i < abs(length)                                      % while not finished
  i = i + (length>0);                                      % count iterations?!

  X0 = X; f0 = f1; df0 = df1;                   % make a copy of current values
  X = X + z1*s;                                             % begin line search
  [f2 df2] = eval(argstr);
  i = i + (length<0);                                          % count epochs?!
  d2 = df2'*s;
  f3 = f1; d3 = d1; z3 = -z1;             % initialize point 3 equal to point 1
  if length>0, M = MAX; else M = min(MAX, -length-i); end
  success = 0; limit = -1;                     % initialize quanteties
  while 1
    while ((f2 > f1+z1*RHO*d1) || (d2 > -SIG*d1)) && (M > 0)
      limit = z1;                                         % tighten the bracket
      if f2 > f1
        z2 = z3 - (0.5*d3*z3*z3)/(d3*z3+f2-f3);                 % quadratic fit
      else
        A = 6*(f2-f3)/z3+3*(d2+d3);                                 % cubic fit
        B = 3*(f3-f2)-z3*(d3+2*d2);
        z2 = (sqrt(B*B-A*d2*z3*z3)-B)/A;       % numerical error possible - ok!
      end
      if isnan(z2) || isinf(z2)
        z2 = z3/2;                  % if we had a numerical problem then bisect
      end
      z2 = max(min(z2, INT*z3),(1-INT)*z3);  % don't accept too close to limits
      z1 = z1 + z2;                                           % update the step
      X = X + z2*s;
      [f2 df2] = eval(argstr);
      M = M - 1; i = i + (length<0);                           % count epochs?!
      d2 = df2'*s;
      z3 = z3-z2;                    % z3 is now relative to the location of z2
    end
    if f2 > f1+z1*RHO*d1 || d2 > -SIG*d1
      break;                                                % this is a failure
    elseif d2 > SIG*d1
      success = 1; break;                                             % success
    elseif M == 0
      break;                                                          % failure
    end
    A = 6*(f2-f3)/z3+3*(d2+d3);                      % make cubic extrapolation
    B = 3*(f3-f2)-z3*(d3+2*d2);
    z2 = -d2*z3*z3/(B+sqrt(B*B-A*d2*z3*z3));        % num. error possible - ok!
    if ~isreal(z2) || isnan(z2) || isinf(z2) || z2 < 0 % num prob or wrong sign?
      if limit < -0.5                               % if we have no upper limit
        z2 = z1 * (EXT-1);                 % the extrapolate the maximum amount
      else
        z2 = (limit-z1)/2;                                   % otherwise bisect
      end
    elseif (limit > -0.5) && (z2+z1 > limit)         % extraplation beyond max?
      z2 = (limit-z1)/2;                                               % bisect
    elseif (limit < -0.5) && (z2+z1 > z1*EXT)       % extrapolation beyond limit
      z2 = z1*(EXT-1.0);                           % set to extrapolation limit
    elseif z2 < -z3*INT
      z2 = -z3*INT;
    elseif (limit > -0.5) && (z2 < (limit-z1)*(1.0-INT))  % too close to limit?
      z2 = (limit-z1)*(1.0-INT);
    end
    f3 = f2; d3 = d2; z3 = -z2;                  % set point 3 equal to point 2
    z1 = z1 + z2; X = X + z2*s;                      % update current estimates
    [f2 df2] = eval(argstr);
    M = M - 1; i = i + (length<0);                             % count epochs?!
    d2 = df2'*s;
  end                                                      % end of line search

  if success                                         % if line search succeeded
    f1 = f2; fX = [fX' f1]';
    % fprintf('%s %4i | Cost: %4.6e\r', S, i, f1);
    s = (df2'*df2-df1'*df2)/(df1'*df1)*s - df2;      % Polack-Ribiere direction
    tmp = df1; df1 = df2; df2 = tmp;                         % swap derivatives
    d2 = df1'*s;
    if d2 > 0                                      % new slope must be negative
      s = -df1;                              % otherwise use steepest direction
      d2 = -s'*s;
    end
    z1 = z1 * min(RATIO, d1/(d2-realmin));          % slope ratio but max RATIO
    d1 = d2;
    ls_failed = 0;                              % this line search did not fail
  else
    X = X0; f1 = f0; df1 = df0;  % restore point from before failed line search
    if ls_failed || i > abs(length)          % line search failed twice in a row
      break;                             % or we ran out of time, so we give up
    end
    tmp = df1; df1 = df2; df2 = tmp;                         % swap derivatives
    s = -df1;                                                    % try steepest
    d1 = -s'*s;
    z1 = 1/(1-d1);
    ls_failed = 1;                                    % this line search failed
  end
  if exist('OCTAVE_VERSION')
    fflush(stdout);
  end
end
% fprintf('\n');

end
