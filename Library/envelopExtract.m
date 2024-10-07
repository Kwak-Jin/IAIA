function [z, inst_amplitude] = envelopExtract(x)
%
% returns constructed analytic signal
% x is a real-valued record of length N, where N is even 
% returns: the analytic signal z[n]
%          instant amplitude m[n]  


x = x(:); %serialize
N = length(x);

%%% Part 1. z = hilbert(x)
% FFT of x

X = zeros(N,1);

% Create P[m]=Z[m]  from m=1 to N
P= fft(x);        %YOUR CODE


X(1) = P(1);
X(2:N/2) = 2*P((2:N/2));
X(N/2+1) = P(N/2+1);

% Create z(t)=Zr+j(Zi) from ifft(P)
z = ifft(X,N);       %YOUR CODE

% Part 2. Envelope extraction
inst_amplitude = abs(z); 

end