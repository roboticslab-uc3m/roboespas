function [q_out, qdot_out, qdotdot_out, tKnot, q_coefs, qdot_coefs, qdotdot_coefs] = boundedspline(t, q, velocity, smoothing)
    if (isempty(q) || isempty(t))
        q_out=[];
        qdot_out=[];
        qdotdot_out=[];
        tKnot=[];
        q_coefs=[];
        qdot_coefs=[];
        qdotdot_coefs=[];
        return
    end
    ppsec=3;
    %points_per_seg=200/velocity;
    %nSeg=ceil(size(t,2)/points_per_seg);
    nSeg=ceil(ppsec*t(end)*velocity); %%Cambiar
    nKnot=nSeg+1;
    tKnot=linspace(t(1), t(end), nKnot);
    
    %% Run main function
    [H, f] = computeQuadraticCostMatrix(t', q', tKnot', smoothing);
    nState = size(q,1);
    [A1, b1] = computeEqCstContinuity(tKnot, nState);
    lowBnd.pos = q(:,1)';
    lowBnd.vel = zeros(1, size(q,1));
    lowBnd.acc = [];
    uppBnd.pos = q(:,end)';
    uppBnd.vel = zeros(1, size(q,1));
    uppBnd.acc = [];
    [A2, b2] = computeEqCstBoundary(lowBnd, uppBnd, tKnot);
    coeff = solveQpEq(H,f,[A1;A2],[b1;b2]);

    nPoly = 4;
    pp.form = 'pp';
    pp.breaks = tKnot;
    pp.coefs = zeros(nSeg*nState, nPoly);  % 4 = cubic coeff count
    pp.pieces = nSeg;
    pp.order = nPoly;
    pp.dim = nState;

    for iSeg = 1:nSeg
        for iDim = 1:nState
          idx_pp_row = nState*(iSeg-1) + iDim;
          idx_coef_cols = nPoly*(iSeg-1) + (1:nPoly);
          pp.coefs(idx_pp_row,:) = fliplr(coeff(idx_coef_cols,iDim)');
        end
    end

    xpp = mkpp(pp.breaks, pp.coefs, pp.dim);
    vpp = ppDer(xpp);
    app = ppDer(vpp);

    %% Fill output
    q_out=ppval(xpp, t);
    qdot_out=ppval(vpp, t);
    qdotdot_out=ppval(app, t);
    
    q_coefs=zeros(xpp.order, size(xpp.breaks,2)-1, xpp.dim);
    qdot_coefs=zeros(vpp.order, size(vpp.breaks,2)-1, vpp.dim);
    qdotdot_coefs=zeros(app.order, size(app.breaks,2)-1, app.dim);
    for i=1:xpp.dim
        q_coefs(:,:,i)=xpp.coefs(i:xpp.dim:end,:)';
        qdot_coefs(:,:,i)=vpp.coefs(i:vpp.dim:end,:)';
        qdotdot_coefs(:,:,i)=app.coefs(i:app.dim:end,:)';
    end
end
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%The following code has been obtained from MatlabFileExchange
%(https://ww2.mathworks.cn/matlabcentral/fileexchange/64119-fitsplinetodata?s_tid=prof_contriblnk)
% Copyright (c) 2017, Matthew Kelly
% All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
% 
% * Redistributions of source code must retain the above copyright notice, this
%   list of conditions and the following disclaimer.
% 
% * Redistributions in binary form must reproduce the above copyright notice,
%   this list of conditions and the following disclaimer in the documentation
%   and/or other materials provided with the distribution
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
% DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
% FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
% DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
% SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
% CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
% OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
% OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
function dpp = ppDer(pp)
% dpp = ppDer(pp)
%
% Computes the time-derivative of piece-wise polynomial (PP) struct
%
% INPUTS:
%   pp = a PP struct containing a trajectory of interest
%
% OUTPUTS:
%   dpp = a new PP struct that is the time-derivative of pp
%
% NOTES:
%   --> a pp struct is typically created by matlab functions such as
%   spline, pchip, or pwch and evaluated using ppval.
%     pp=mkpp(breaks,coefs);
%     n = pp.order;
%     nRows = size(pp.coefs,1);
%     coefs = zeros(nRows,n-1);
%     for i=1:n-1
%        coefs(:,i) = (n-i)*pp.coefs(:,i);
%     end
%     dpp=mkpp(pp.breaks, coefs);
    
    n = pp.order;
    nRows = size(pp.coefs,1);
    dpp.form = pp.form;
    dpp.breaks = pp.breaks;
    dpp.coefs = zeros(nRows,n-1);
    for i=1:n-1
       dpp.coefs(:,i) = (n-i)*pp.coefs(:,i);
    end
    dpp.pieces = pp.pieces;
    dpp.order = pp.order-1;
    dpp.dim = pp.dim;
end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

function [H, f] = computeQuadraticCostMatrix(tData, xData, tKnot, smooth)
% [H, b] = computeQuadraticCostMatrix(tData, xData, tKnot, smooth)
%
% This function computes the least-squares cost terms for fitting a
% cubic spline to time-series data. Includes a term for minimizing the
% integral of jerk-squared as well.
%
% INPUTS:
%   tData = [nTime, 1] = time-series data
%   xData = [nTime, nState] = state data at each point
%   tKnot = [nKnot, 1] = knot points for cubic spline
%   smooth = weight on jerk-squared
%
% OUTPUTS:
%   H = [4*(nKnot-1), 4*(nKnot-1)] = quadratic cost matrix
%   f = [4*(nKnot-1), nState] = linear cost matrix
%
% NOTES:
%
%   nSeg = nKnot - 1;
%   z = [4*nKnot, nState];
%   z = [C0; C1; ... CN] coefficient vector
%   CI = [aI; bi; ci; di];  x = a + b*t +c*t^2 + d*t^3
%   J = 0.5*z'*H*z + f'*z;  % cost function
%

% Figure out which data points belong to which segments
nKnot = length(tKnot);
nSeg = nKnot - 1;
[~, binIdx] = histc(tData, tKnot);
binIdx(binIdx == nKnot) = nKnot;  % include points at endpoint in last segment
nState = size(xData,2);

% Accumulate cost matrix:
beta = (tData(end) - tData(1))/length(tData);
H = zeros(nSeg*4, nSeg*4);
f = zeros(nSeg*4, nState);
for iBin = 1:nSeg
   tBin = tData(binIdx==iBin);
   xBin = xData(binIdx==iBin, :);
   tZero = tKnot(iBin);
   idx = 4*(iBin-1) + (1:4);
   for iData = 1:length(tBin)
       [hTmp, fTmp] = singleDataPoint(tBin(iData) - tZero, xBin(iData,:));
     H(idx, idx) = H(idx, idx) +  hTmp;
     f(idx,:) = f(idx,:) + fTmp;
   end
   hSeg = tKnot(iBin+1) - tKnot(iBin);
   H(idx, idx) = beta*H(idx, idx) + smooth*minJerk(hSeg);
end
f = beta*f;

end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

function [H, f] = singleDataPoint(t, x)
% [H, f] = singleDataPoint(t, x)
%
% Computes the quadratic cost associated with the fitting error between a
% single point and a cubic segment.

t0 = 1;
t1 = t;
t2 = t1*t1;
t3 = t2*t1;
t4 = t2*t2;
t5 = t2*t3;
t6 = t3*t3;

H = [
    t0, t1, t2, t3;
    t1, t2, t3, t4;
    t2, t3, t4, t5;
    t3, t4, t5, t6;
    ];

f = [
    -x*t0;
    -x*t1;
    -x*t2;
    -x*t3;
    ];

end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

function H = minJerk(h)

H = zeros(4,4);
H(4,4) = 36*h;

end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

function [A, b] = computeEqCstBoundary(low, upp, tKnot)
% [A, b] = computeEqCstBoundary(low, upp, nKnot)
%
% This function computes the equality constraints that enforce the
% boundary position, velocity, and acceleration.
%
% INPUTS:
%   low.pos = [1, nState] = position at lower boundary
%   low.vel = [1, nState] = velocity at lower boundary
%   low.acc = [1, nState] = acceleration at lower boundary
%   upp.pos = [1, nState] = position at upper boundary
%   upp.vel = [1, nState] = velocity at upper boundary
%   upp.acc = [1, nState] = acceleration at upper boundary
%   nKnot = [scalar] = number of knot points on the spline
%
% OUTPUTS:
%   A = [3*nSeg, 4*nSeg] = linear constraint matrix
%   b = zeros(3*nSeg, nState) = constant terms in constraint
%
% NOTES:
%
%   Set any boundary constraint to [] to ignore it.
%
%   nSeg = nKnot - 1;
%   z = [4*nKnot, nState];
%   z = [C0; C1; ... CN] coefficient vector
%   CI = [aI; bi; ci; di];  x = a + b*t +c*t^2 + d*t^3
%   A*z = b;  % cost function
%

b = [
    low.pos;
    low.vel;
    low.acc;
    upp.pos;
    upp.vel;
    upp.acc;
    ];

nGrid = length(tKnot);
nDecVar = 4*(nGrid-1);

if ~isempty(low.pos)
    aLowPos = [1,0,0,0, zeros(1, nDecVar-4)];
else
    aLowPos = [];
end
if ~isempty(low.vel)
    aLowVel = [0,1,0,0, zeros(1, nDecVar-4)];
else
    aLowVel = [];
end
if ~isempty(low.acc)
    aLowAcc = [0,0,2,0, zeros(1, nDecVar-4)];
else
    aLowAcc = [];
end

% Upper edge of the last segment
h1 = tKnot(end) - tKnot(end-1); 
h2 = h1*h1;
h3 = h2*h1;

if ~isempty(upp.pos)
    aUppPos = [zeros(1, nDecVar-4), 1,   h1,    h2,      h3];
else
    aUppPos = [];
end
if ~isempty(upp.vel)
    aUppVel = [zeros(1, nDecVar-4), 0,   1,   2*h1,    3*h2];
else
    aUppVel = [];
end
if ~isempty(upp.acc)
    aUppAcc = [zeros(1, nDecVar-4), 0,   0,     2,     6*h1];
else
    aUppAcc = [];
end

A = [
    aLowPos;
    aLowVel;
    aLowAcc;
    aUppPos;
    aUppVel;
    aUppAcc;
    ];

end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

function [A, b] = computeEqCstContinuity(tKnot, nState)
% [A, b] = computeEqCstContinuity(tKnot)
%
% This function computes the equality constraints that enforce position,
% velocity, and acceleration continuity.
%
% INPUTS:
%   tKnot = [nKnot, 1] = knot points for cubic spline
%
% OUTPUTS:
%   A = [3*nSeg, 4*nSeg] = linear constraint matrix
%   b = zeros(3*nSeg, nState) = constant terms in constraint
%
% NOTES:
%
%   nSeg = nKnot - 1;
%   z = [4*nKnot, nState];
%   z = [C0; C1; ... CN] coefficient vector
%   CI = [aI; bi; ci; di];  x = a + b*t +c*t^2 + d*t^3
%   A*z = b;  % cost function
%

nKnot = length(tKnot);
nSeg = nKnot - 1;
b = zeros(3*(nSeg-1), nState);

A = zeros(3*(nSeg-1), 4*nSeg);


for iSeg = 1:(nSeg-1)
  rowIdx = 3*(iSeg-1) + (1:3);
  colIdx = 4*(iSeg-1) + (1:4);
  hSeg = tKnot(iSeg+1) - tKnot(iSeg);
  [upp, low] = continuityConstraint(hSeg);
  A(rowIdx, colIdx) = A(rowIdx, colIdx) + upp;
  A(rowIdx, colIdx+4) = A(rowIdx, colIdx+4) - low;
end

end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

function [upp, low] = continuityConstraint(h)

h2 = h*h;
h3 = h2*h;

% upper edge of the lower segment:
upp = [...
    1,   h,    h2,    h3;
    0,   1,  2*h,   3*h2;
    0,   0,    2,   6*h];
% lower edge of the upper segment:
low = [...
    1,   0,  0,  0;
    0,   1,  0,  0;
    0,   0,  2,  0];

end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

function [zSoln, lambda] = solveQpEq(H,f,A,b)
%
%     min 0.5*x'*H*x + f'*x   subject to:  A*x = b
%

%%% Linear solve formulation:
% [H, Aeq'; Aeq, 0] * [z;w] = [-f;beq]
% MM*xx = cc
%

s = norm(H);
H = H/s;
f = f/s;

s = norm(A);
A = A/s;
b = b/s;

nCst = size(A,1);
nDecVar = size(H,1);
MM = [H, A'; A, zeros(nCst)];
cc = [-f; b];
xx = MM \ cc;
zSoln = xx(1:nDecVar,:);
lambda = xx((nDecVar+1):end,:);

end