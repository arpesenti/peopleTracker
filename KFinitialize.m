function [ prediction ] = KFinitialize( initialPosition, initialLegProbability )
%KFPREDICT initialize Kalman Filter

     
P = diag([0.3 0.3 60 60 1]);
x = [initialPosition ./ 1000; 0; 0; initialLegProbability];
prediction = struct('x', x, 'P', P);

end

