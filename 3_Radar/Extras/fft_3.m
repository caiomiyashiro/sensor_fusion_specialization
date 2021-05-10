Fs = 1000;            % Sampling frequency                    
T = 1/Fs;             % Sampling period       
L = 1500;             % Length of signal
t = (0:L-1)*T;        % Time vector

% TODO: Form a signal containing a 77 Hz sinusoid of amplitude 0.7 and a 43Hz sinusoid of amplitude 2.
f1 = 77;
A1 = 0.7;
S1 = A1*sin(2*pi*f1*t);

f2 = 43;
A2 = 2;
S2 = A2*sin(2*pi*f2*t);

S = S1 + S2;

% Corrupt the signal with noise 
X = S + 2*randn(size(t));

% Plot the noisy signal in the time domain. It is difficult to identify the frequency components by looking at the signal X(t). 
plot(1000*t(1:50) ,X(1:50))
title('Signal Corrupted with Zero-Mean Random Noise')
xlabel('t (milliseconds)')
ylabel('X(t)')

% TODO : Compute the Fourier transform of the signal. 
signal_fft = fft(X);
signal_fft = abs(signal_fft/L); % if you want the true amplitude, divide by (L/2)

% TODO : Compute the two-sided spectrum P2. Then compute the single-sided spectrum P1 based on P2 and the even-valued signal length L.
signal_fft  = signal_fft(1:L/2+1) ;

% Plotting
f = Fs*(0:(L/2))/L;
plot(f,signal_fft) 
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')