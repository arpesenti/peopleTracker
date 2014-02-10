function [ prediction ] = KFpredict( previousPrediction, sigmaAcc, DT )
%KFPREDICT apply prediction step of Kalman Filter
%   previousPrediction = struct('x','P') containing state prediction x and
%                        error covariance P.

% DT time interval between two measures;

% state transition model
F = [1 0 DT 0 0;
     0 1 0 DT 0;
     0 0 1 0 0;
     0 0 0 1 0;
     0 0 0 0 1]; 

% covariance of noise on state transition model

G = [DT^2/2;
     DT];
Q1 = G*G'*sigmaAcc^2;
Q2 = Q1;

Q = [Q1(1,1)  0       Q1(1,2)  0        0;
     0        Q2(1,1)     0    Q2(1,2)  0;
     Q1(2,1)  0        Q1(2,2) 0        0;
     0        Q2(2,1)     0    Q2(2,2)  0;
     0        0           0    0        0];
     
if isempty(previousPrediction)
  L = 1000000;
  prediction = struct('x',zeros(size(F,1),1), 'P', diag(ones(size(F,1),1) * L));
  return;
end

prediction = previousPrediction;

prediction.x = F * previousPrediction.x;
prediction.P = F * previousPrediction.P * F' + Q;

end

