function xfeature=freqFeatures(F)
% Returns frequency-domain features of FFT(x)
% input:    F, 1-d vector
% output:   xFeature, table form


% Create table variable xfeature
xfeature = table;
N=length(F);


% Frequency Center
% YOUR CODE GOES HERE
xfeature.fc= 1/N * sum(F);

% RMS frequency
% YOUR CODE GOES HERE
xfeature.rmsf= sqrt(1/N *sum(F.^2));

% Root variance frequency
% YOUR CODE GOES HERE
xfeature.rvf= sqrt(1/N*sum((F-xfeature.fc).^2));