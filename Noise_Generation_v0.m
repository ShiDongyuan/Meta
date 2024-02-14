%% Clean the memory and worksapace 
close all ;
clear     ;
clc       ;
%% Configure the system simulation condition 
fs  =   16000 ; % The system sampling rate.
T   =   3     ; % The duration of the simulation.
t   =   0:1/fs:T ; 
N   =   length(t); % The number of the data ;

%% Construct the broad band noise ;
Noise = randn(N,1);
% filter 
filter_1 = fir1(512,[0.05 0.25]);
filter_2 = fir1(512,[0.20 0.55]) ;
filter_3 = fir1(512,[0.5,0.75]) ;
% Primary noise 
Pri_1 = filter(filter_1,1,Noise) ;
Pri_2 = filter(filter_2,1,Noise) ;
Pri_3 = filter(filter_3,1,Noise) ;

%% Drawing fiture 
data = [Pri_1,Pri_2,Pri_3];
for ii = 1:3
    figure ;
    freq = 20*log(abs(fft(data(:,ii))));
    plot(freq);
    grid on   ;
end

%% Save data into workspace 
save('Primary_noise.mat','Pri_1','Pri_2','Pri_3');

%% Loading path 
load('path\P1.mat') ;
load('path\S11.mat');
Pri_path = conv(P1,S11);
% Distrubance 
Dis_1 = filter(Pri_path,1,Pri_1);
Dis_2 = filter(Pri_path,1,Pri_2);
Dis_3 = filter(Pri_path,1,Pri_3);

%% Drawing fiture 
data = [Dis_1,Dis_2,Dis_3];
for ii = 1:3
    figure ;
    freq = 20*log(abs(fft(data(:,ii))));
    plot(freq);
    grid on   ;
end

%% Save data into workspace 
save('Disturbance.mat','Dis_1','Dis_2','Dis_3');

% Filtered reference signal 
Rf_1 = filter(S11,1,Pri_1);
Rf_2 = filter(S11,1,Pri_2);
Rf_3 = filter(S11,1,Pri_3);

%% Save data into workspace 
save('Reference.mat','Rf_1','Rf_2','Rf_3');
