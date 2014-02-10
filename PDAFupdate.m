function [ updatedPrediction, inGate, createNewTrack ] = PDAFupdate( prediction, measures, sigmaZ, sigmaP, useAllMeasure )
%KFUPDATE Summary of this function goes here
%   prediction = struct('x','P') containing state prediction x and
%                  error covariance P.
%   measure = observation measure of the true state


if nargin < 5
    useAllMeasure = false;
end

measures(:,1:2) = measures(:,1:2) ./ 1000;   % meters

H = [1 0 0 0 0;
     0 1 0 0 0;
     0 0 0 0 1]; % observation model

R = [sigmaZ^2 0 0;
     0 sigmaZ^2 0;
     0  0   sigmaP^2]; % covariance of noise on observation model

S = H*prediction.P*H' + R;    % innovation covariance 
S = tril(S,-1)+tril(S)'; % force simmetry of S
Spos = S(1:2,1:2);

c = pi; % dimensional unit hypersphere
n = 2; % dimension of measurement
% measurement validation
gamma = 20; % "radius" of gate region in meters
gammaNewTrack = 200; % "radius" to create a new track in meters
    
if useAllMeasure
    
    if isempty(measures)
        updatedPrediction = prediction;
        inGate = [];
        createNewTrack = [];
        return;
    end

    nMeasures = size(measures,1);
    zHat = H*prediction.x;
    zHatPos = zHat(1:2);
    
    Zerror = measures(:,1:2)-repmat(zHatPos',nMeasures,1);
    k = ones(nMeasures,1);
    for i=1:nMeasures
        k(i) = (Spos\(Zerror(i,:)'))'*(Zerror(i,:)')/gamma;
    end
    
    %Spos = 10*max(k).*Spos;
    
    measuresValidated = ones(nMeasures,1);
    inGate = ones(nMeasures,1);
    createNewTrack = zeros(nMeasures,1);
    
    V = c*gamma^(n/2)*sqrt(det(Spos));
else 

    V = c*gamma^(n/2)*sqrt(det(Spos)); % volume of validation region

    if isempty(measures)
        updatedPrediction = prediction;
        inGate = [];
        createNewTrack = [];
        return;
    end


    nMeasures = size(measures,1);
    zHat = H*prediction.x;
    zHatPos = zHat(1:2);
    measuresValidated = zeros(nMeasures,1);
    inGate = zeros(nMeasures,1);
    createNewTrack = zeros(nMeasures,1);
    
    for i=1:nMeasures
        z = measures(i,1:2)';
        a = (Spos\(z-zHatPos))'*(z-zHatPos);

        if a <= gamma
            measuresValidated(i) = 1;
            inGate(i) = 1;
        elseif a>=gammaNewTrack
            createNewTrack(i) = 1;
        end
    end
end

measures = measures(measuresValidated==1,:);
m = size(measures,1);
if m == 0
    % no measurements on validation region
    updatedPrediction = prediction;
    return;
end

%data association
Pg = 0.8; % probability that gate contains the true measure if detected (corresponding to gamma)
Pd = 0.7; % target detection probability
likelihood = mvnpdf(measures(:,1:2),zHatPos',Spos).*Pd./(m/V);
normalizer = 1-Pd*Pg + sum(likelihood); 
beta0 = (1-Pd*Pg)/normalizer;
betas = likelihood./normalizer;

updatedPrediction = prediction;

yi = (measures - repmat(zHat',m,1)); % innovations
y = sum(yi.*repmat(betas,1,size(measures,2)),1)';   % combined innovation

K = (S' \ (prediction.P*H')')';    % Kalman gain
updatedX = prediction.x + K*y;

Pc = prediction.P - K*S*K'; % covariance of the state updated with correct measurements
Ptilde = K*((repmat(betas',size(measures,2),1).*yi')*yi - y*y')*K';
updatedP = beta0*prediction.P + (1-beta0)*Pc + Ptilde;

% if prediction.x(5)>0.8 && sqrt(prediction.P(5,5))/2<0.05
%     updatedX(5) = max(prediction.x(5),updatedX(5));
%     updatedP(5,5) = min(prediction.P(5,5),updatedP(5,5));
% end

updatedPrediction.P = updatedP;
updatedPrediction.x = updatedX;

end

