# Model-Agnostic Meta-Learning for Adaptive Filter

This document describes the process and findings of testing the MAML algorithm for enhancing the convergence of the FxLMS algorithm in noise cancellation applications, specifically targeting aircraft noise.

## Introduction

The purpose of this test is to evaluate the Modified MAML (Model-Agnostic Meta-Learning) algorithm's effectiveness in initializing control filters for noise cancellation.

## Code Explanation
- `Main_tst_function.m`: the main function is utilized to test the proposed modified MAML algorithm.  
- `MAML_Nstep_forget.m`: the matlab code of the modified MAML algorithm.
- `FxLMS.m`: the matlab code of the FxLMS algorithm.
- `707_Sound_for_Simulation.mat`: the raw data of an aircrat noise.
- `\path`: the measured primary and secondary paths. 

### The explanation of `Main_tst_function.m`

The following code snippet offers a concise overview of `Main_tst_function.m`, which serves as the main function for evaluating the effectiveness of the proposed MAML method. For this numerical simulation, three distinct broadband sounds are utilized to train the MAML algorithm and obtain a single initial control filter. Subsequently, the aforementioned initial control is employed within an FxLMS algorithm to effectively eliminate actual aircraft noise. In comparison to zero initialization, the MAML technique can significantly enhance the convergence speed of the conventional FxLMS approach.  

### Contents:
- [Clean the memory and worksapace](#clean-the-memory-and-worksapace)
- [Configure the system simulation condition](#configure-the-system-simulation-condition)  
- [Build the broad band noise for training set](#build-the-broad-band-noise-for-training-set)
- [Radomly sampling the noise tracks to build dataset for the MAML algorithm](#radomly-sampling-the-noise-tracks-to-build-dataset-for-the-maml-algorithm)
- [Using Modified MAML algorithm to get the best initial control filter](#using-modified-maml-algorithm-to-get-the-best-initial-control-filter)
- [Testing aircraft noise cancellation by using MAML initial control filter](#testing-aircraft-noise-cancellation-by-using-maml-initial-control-filter)

#### Clean the memory and worksapace
Code snippet to clean workspace and set initial conditions.
```matlab
%% Clean the memory and worksapace 
close all ;
clear     ;
clc       ;
```
#### Configure the system simulation condition

```matlab
%% Configure the system simulation condition 
fs  =   16000    ; % The system sampling rate.
T   =   3        ; % The duration of the simulation.
t   =   0:1/fs:T ; 
N   =   length(t); % The number of the data.

Len_N = 512      ; % Seting the length of the control filter.
%<<===Progress bar===>> 
f = waitbar(0,'Please wait...');
pause(.5)
```

#### Build the broad band noise for training set
```matlab
%% Build the broad band noise for training set
%<<===Progress bar===>> 
waitbar(0.25,f,'Build the broad band noise for training set');
pause(1)
%<<===Progress bar===>> 
% Loading path 
load('path\P1.mat')    ;
load('path\S11.mat')   ;
Pri_path = conv(P1,S11);

Track_num = 3         ; % Seting the number of the track for the trainning noise. 
if exist('Primary_noise.mat', 'file') == 2
    disp('Primary_noise exists in the current path.\n');
    % Loading the primary noise 
    load('Primary_noise.mat');
    load('Disturbance.mat')  ;
    load('Reference.mat')    ;
else
Noise     = randn(N,1);
% filter 
filter_1 = fir1(512,[0.05 0.25]);
filter_2 = fir1(512,[0.20 0.55]) ;
filter_3 = fir1(512,[0.5,0.75]) ;
% Primary noise 
Pri_1 = filter(filter_1,1,Noise) ;
Pri_2 = filter(filter_2,1,Noise) ;
Pri_3 = filter(filter_3,1,Noise) ;
% Drawing fiture 
data = [Pri_1,Pri_2,Pri_3];
figure ;
len_fft = length(Pri_1)   ;
len_hal = round(len_fft/2);
title('Frequency spectrum of primary noises')
for ii = 1:3
    freq = 20*log(abs(fft(data(:,ii))));
    subplot(3,1,ii);
    plot(0:(fs/len_fft):(len_hal-1)*(fs/len_fft), freq(1:len_hal));
    grid on   ;
    title("The "+num2str(ii)+"th primary noise")
    xlabel('Frequency (Hz)')
end
% Save primary noise into workspace 
save('Primary_noise.mat','Pri_1','Pri_2','Pri_3');
% Generating Distrubance 
Dis_1 = filter(Pri_path,1,Pri_1);
Dis_2 = filter(Pri_path,1,Pri_2);
Dis_3 = filter(Pri_path,1,Pri_3);
% Save distrubancec into workspace 
save('Disturbance.mat','Dis_1','Dis_2','Dis_3');
% Genrating Filtered reference signal 
Rf_1 = filter(S11,1,Pri_1);
Rf_2 = filter(S11,1,Pri_2);
Rf_3 = filter(S11,1,Pri_3);
% Save filter reference signal into workspace 
save('Reference.mat','Rf_1','Rf_2','Rf_3');
end
```

![Primary noises](Images/Main_tst_function_03.png)  
Figure S1: The frequency spectrum of the primary noises for the MAML algorithm.

#### Radomly sampling the noise tracks to build dataset for the MAML algorithm
```matlab
%% Radomly sampling the noise tracks to build dataset for the MAML algorithm
%<<===Progress bar===>> 
waitbar(0.5,f,'Radomly sampling the noise tracks to build dataset for the MAML algorithm');
pause(1)
%<<===Progress bar===>> 
if exist('Sampe_data_N_set.mat', 'file') == 2
    disp('Sampe_data_N_set in the current path.\n');
    load('Sampe_data_N_set.mat');
else
N_epcho  = 4096 * 80                   ; % Setting the number of the epcho 
Trac     = randi(Track_num,[N_epcho,1]); % Randomly choosing the different tracks. 
Len_data = length(Dis_1)               ;
% Seting the N steps 
len   = 2*Len_N -1 ;
Fx_data = zeros(Len_N,N_epcho);
Di_data = zeros(Len_N,N_epcho);
Ref_data = [Rf_1,Rf_2,Rf_3]   ;
Dis_data = [Dis_1,Dis_2,Dis_3];
for jj = 1:N_epcho
    End = randi([len,Len_data]);
    Di_data(:,jj) = Dis_data(End-511:End,Trac(jj));
    Fx_data(:,jj) = Ref_data(End-511:End,Trac(jj));
end
save('Sampe_data_N_set.mat','Di_data','Fx_data');
end
```

#### Using Modified MAML algorithm to get the best initial control filter
```matlab
%<<===Progress bar===>> 
waitbar(0.75,f,'Using Modified MAML algorithm to get the best initial control filter');
pause(1)
%<<===Progress bar===>> 
if exist('Weigth_initiate_Nstep_forget.mat', 'file') == 2
    disp('Weigth_initiate_Nstep_forget in the current path.\n');
    load('Weigth_initiate_Nstep_forget.mat');
else
% Create a MAML algorithm
a  = MAML_Nstep_forget(Len_N);
N  = size(Di_data,2)         ; % The number of the sample in training set.
Er = zeros(N,1)              ; % Residual error vector
% Seting the step size for the embeded FxLMS algorithm 
mu    = 0.0003              ;
% Seting the forget factor 
lamda = 0.99                ;
% Seting the learning for MAML 
epslon = 0.5 ;
% Runing the MAML algorithm 
for jj = 1:N
    [a, Er(jj)] = a.MAML_initial(Fx_data(:,jj),Di_data(:,jj),mu,lamda,epslon);
end
% Drawing the residual error of the Modified MAML algorihtm 
figure    ;
plot(Er)  ;
grid on   ;
title('Leanring curve of the modified MAML algorithm');
xlabel('Epoch');
ylabel('Residual error');
% Getting the optimal intial control filter
Wc = a.Phi ;
% Saving the best initial control filter into workspace
save('Weigth_initiate_Nstep_forget.mat','Wc');
end
```

![Learning curve of the MAML algorithm](Images/Main_tst_function_06.png)   
Figure S2: The learning curve of the modified MAML algorithm.

#### Testing aircraft noise cancellation by using MAML initial control filter
```matlab
%% Testing aircraft noise cancellation by using MAML initial control filter 
%<<===Progress bar===>> 
waitbar(0.95,f,'Testing aircraft noise cancellation by using MAML initial control filter');
pause(1)
%<<===Progress bar===>> 
% Loading aricrat noise data 
aircrat = load('707_Sound_for_Simulation.mat');
% Building primary noise
Pri_1   = aircrat.PilingNoise(1:153945)       ; 
% Generating the disturbacne 
Dis_1   = filter(Pri_path,1,Pri_1)            ;
% Generating the filter reference             
Rf_1    = filter(S11,1,Pri_1)                 ;
% Runging the FxLMS with the zero-initialization control filter    
Wc_initial = zeros(Len_N,1);
muw        = 0.00001 ; % Step size of All FxLMS algorithms. 
Er         = FxLMS(Len_N, Wc_initial, Dis_1, Rf_1, muw);
% Runging the FxLMS with the MAML-initialization control filter    
Wc_initial = Wc;
Er1        = FxLMS(Len_N, Wc_initial, Dis_1, Rf_1, muw);
figure 
% Drawing the figures of the MAML and FxLMS 
plot((0:length(Dis_1)-1)*(1/fs),Dis_1,(0:length(Er)-1)*(1/fs),Er,(0:length(Er1)-1)*(1/fs),Er1);
title('Aircraft noise cancellation')
xlabel('Time (seconds)')
ylabel('Error signal')
legend({'ANC off','FxLMS with zero-initialization','FxLMS with MAML-initialization'})
grid on;
%<<===Progress bar===>> 
waitbar(1,f,'Finishing');
pause(1)
%<<===Progress bar===>> 
%-------------------------------end----------------------------------------
```

![Noise reduction performance](Images/Main_tst_function_08.png)  
Figure S3: Aircraft noise reduction performance of the FxLMS algorithms with the zero and MAML initializations. 

## Conclusion 

A step-by-step guide to preparing the broadband noise for the training set, simulating the noise environment, and applying the MAML algorithm.

## Reference 
``` bibtex
@article{shi2021fast,
  title={Fast adaptive active noise control based on modified model-agnostic meta-learning algorithm},
  author={Shi, Dongyuan and Gan, Woon-Seng and Lam, Bhan and Ooi, Kenneth},
  journal={IEEE Signal Processing Letters},
  volume={28},
  pages={593--597},
  year={2021},
  publisher={IEEE}
}
```


