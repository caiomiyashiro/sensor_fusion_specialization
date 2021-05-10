# CFAR implementation

1. Signal:
<img src="images/1 - Signal.png" width="779" height="414" />

2. Slide Window through the complete Range Doppler Map  

```Matlab
%Slide Window through the complete Range Doppler Map
%Select the number of Training Cells in both the dimensions.
Tr = 10;
Td = 8;

%Select the number of Guard Cells in both dimensions around the Cell under
%test (CUT) for accurate estimation
Gr = 4;
Gd = 4;

% offset the threshold by SNR value in dB
offset = 6;

%Create a vector to store noise_level for each iteration on training cells
noise_level = zeros(1,1);
RDM_pow = db2pow(RDM);
Nr = Nr/2;
```

3. Simulation Loop
Simulate Target movement and calculate the beat or mixed signal for every timestamp.

```Matlab
for i = Tr+Gr+1:Nr-(Tr+Gr)
    for j = Td+Gd+1:Nd-(Td+Gd)
        noise_level_matrix1 = RDM_pow(i-Tr-Gr:i+Tr+Gr,j-Td-Gd:j+Td+Gd);
        noise_level_matrix2 = RDM_pow(i-Gr:i+Gr,j-Gd:j+Gd);
        noise_level = sum(noise_level_matrix1(:)) - sum(noise_level_matrix2(:));
        threshold = offset + pow2db(noise_level/(2*(Tr+Gr+1)*2*(Td+Gd+1) - ...
            2*(Gr+1)*2*(Gd+1)));
        CUT = RDM(i,j);
        if(CUT < threshold)
            RDM(i,j) = 0;
        else
            RDM(i, j) = 1;
        end        
    end
end
```

4. Range FFT (1st FFT)
Implement the Range FFT on the Beat or Mixed Signal and plot the result.  

```Matlab
Mix = reshape(Mix, Nr, Nd);

%run the FFT on the beat signal along the range bins dimension (Nr) and
%normalize.
range_FFT = fft(Mix,Nr);
range_FFT =range_FFT / (Nr/2);

% Take the absolute value of FFT output
range_FFT = abs(range_FFT);

% Output of FFT is double sided signal, but we are interested in only one side
% of the spectrum.Hence we throw out half of the samples.
rFFT = range_FFT(1:Nr/2+1, :);

%plotting the Mix
figure ('Name','Range from First FFT')
subplot(2,1,1)
plot(Mix(:, 1));
axis ([0 200 -2.0 2.0]);
title('Mix Signal(IF signal)');
xlabel('Samples Time');
%ylabel('Ampltitude')

% plot FFT output
subplot(2, 1, 2);
plot(rFFT(:, 1));
axis ([0 200 0 1.0]);
title('Range FFT of Mix Signal(IF signal)');
xlabel('Range(Frequency)')
```

5. 2D CFAR
Implement the 2D CFAR process on the output of 2D FFT operation, i.e the Range Doppler Map.

Determine the number of Training cells for each dimension. Similarly, pick the number of guard cells.

```Matlab
%Slide Window through the complete Range Doppler Map
%Select the number of Training Cells in both the dimensions.
Tr = 10;
Td = 8;

%Select the number of Guard Cells in both dimensions around the Cell under
%test (CUT) for accurate estimation
Gr = 4;
Gd = 4;

% offset the threshold by SNR value in dB
offset = 6;

%Create a vector to store noise_level for each iteration on training cells
noise_level = zeros(1,1);
RDM_pow = db2pow(RDM);
Nr = Nr/2;

%For every iteration sum the signal level within all the training
%cells. To sum convert the value from logarithmic to linear using db2pow
%function. Average the summed values for all of the training
%cells used. After averaging convert it back to logarithimic using pow2db.
%Further add the offset to it to determine the threshold. Then, compare the
%signal under CUT with this threshold. If the CUT level > threshold assign
%it a value of 1, else equate it to 0.
for i = Tr+Gr+1:Nr-(Tr+Gr)
    for j = Td+Gd+1:Nd-(Td+Gd)
        noise_level_matrix1 = RDM_pow(i-Tr-Gr:i+Tr+Gr,j-Td-Gd:j+Td+Gd);
        noise_level_matrix2 = RDM_pow(i-Gr:i+Gr,j-Gd:j+Gd);
        noise_level = sum(noise_level_matrix1(:)) - sum(noise_level_matrix2(:));
        threshold = offset + pow2db(noise_level/(2*(Tr+Gr+1)*2*(Td+Gd+1) - ...
            2*(Gr+1)*2*(Gd+1)));
        CUT = RDM(i,j);
        if(CUT < threshold)
            RDM(i,j) = 0;
        else
            RDM(i, j) = 1;
        end        
    end
end
```
<img src="images/2 - Doppler Response.png" width="779" height="414" />  


```Matlab
% The process above will generate a thresholded block, which is smaller
%than the Range Doppler Map as the CUT cannot be located at the edges of
%matrix. Hence,few cells will not be thresholded. To keep the map size same
% set those values to 0.
RDM(1:Tr+Gr,:)=0;
RDM(Nr-(Tr+Gr):Nr,:)=0;
RDM(:,1:Td+Gd)=0;
RDM(:,Nd-(Td+Gd):Nd)=0;

%display the CFAR output using the Surf function like we did for Range
%Doppler Response output.
```
<img src="images/3 - CFAR.png" width="779" height="414" />
