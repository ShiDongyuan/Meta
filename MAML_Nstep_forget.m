%     __  ___          ___ _____          __   __  ______     __  _____
%    /  |/  /___  ____/ (_) __(_)__  ____/ /  /  |/  /   |   /  |/  / /
%   / /|_/ / __ \/ __  / / /_/ / _ \/ __  /  / /|_/ / /| |  / /|_/ / /
%  / /  / / /_/ / /_/ / / __/ /  __/ /_/ /  / /  / / ___ | / /  / / /___
% /_/  /_/\____/\__,_/_/_/ /_/\___/\__,_/  /_/  /_/_/  |_|/_/  /_/_____/

%% Title: Modified MAML algorithm for intializing the FxLMS algorithm 
% Author: DONGYAN SHI(DSHI003@ntu.edu.sg)
% Date  : 2020-10-1
%--------------------------------------------------------------------------
%% Introduction
% This modified MAML algorithm is used to get the initiailization for
% the FxLMS algorthm, and it can significanly improve the convergence of 
% of the conventional FxLMS algorithm. The code refers to the Table 1
% in [1]. Note: the Randomly sampling processing have been done in 
% Dataset preparing stage.  
%--------------------------------------------------------------------------
%% Reference 
% [1] Shi, Dongyuan, Woon-Seng Gan, Bhan Lam, and Kenneth Ooi. 
% "Fast adaptive active noise control based on modified model-agnostic meta-learning algorithm." 
% IEEE Signal Processing Letters 28 (2021): 593-597.

%% Modified MAML algorithm
classdef MAML_Nstep_forget
    properties 
        Phi % The initial control filter 
    end
    methods 
        function obj = MAML_Nstep_forget(len_c)
            % len_c : the lenght of control filter 
            obj.Phi = zeros(len_c,1);
        end
        function [obj,Er] = MAML_initial(obj,Fx,Di,mu,lamda,epslon)
            % Fx : the filtered reference vector 
            % Di : the disturbance vector  
            % mu : the step size 
            % lamda : the forget factor 
            Fx   = flipud(Fx);
            Dis  = flipud(Di); 
            Grad = 0; % Temporal gradiant accumlator 
            Er   = 0; % Training error signal 
            Li   = length(obj.Phi) ; % The length of the control filter in the FxLMS algorithm. 
			%<-4-> Get the error signal based on the initial control filter. 
			e    = Dis(1)  - obj.Phi'*Fx; 
			%<-5-> Obtain the control filter
            Wo   = obj.Phi + mu*e*Fx    ; % One-step updation for the assumed optimal control filter.
            for jj = 1:Li
                Fd   = [Fx(jj:end);zeros(jj-1,1)];
				%<-6-> Get the error signal based on the new control filter.
                e    = Dis(jj) - Wo'*Fd          ; 
				% Get the gradints based on the assumed optimal control filter 
                Grad = Grad    + epslon*(mu/Li)*e*Fd*(lamda^(jj-1)); 
                if jj == 1
                    Er =  e ;
                end
            end
			%%<-7-> Upate the initial value 
            obj.Phi = obj.Phi + Grad ;
        end
    end
    
end
%------------------------- end --------------------------------------------