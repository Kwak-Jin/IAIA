function P1=getFFT(x,L)
% @ Returns: single-sided spectrum of vector X
% @ Input  :    x, 1-d vector
% @ Output :   power spectrum of a signal in frequency domain
% @ Note   :
% ```MATLAB
% fmax=Fs/2; 
% fres=Fs/L; 
% f = 0:fres:fmax;
% figure, plot(f,P1), title('FFT X(t)'), xlabel('f (Hz)'), ylabel('|P(f)|')
% xlim([0 200]);
% ```


%%% Apply FFT of x
Y = fft(x,L);

%%% Compute the two-sided spectrum P2
% Then compute the single-sided spectrum P1 based on P2 and the even-valued signal length L.
% Zero frequency P1(0), and the Nyquist frequency P(end) do not occur twice. 

P2 = abs(Y/L);
P1 = P2(1:floor(L/2)+1);
% YOUR CODE GOES HERE
P1(2:end-1) = 2*P1(2:end-1);

end
