classdef MAML_Nstep_forget
    properties 
        Wi 
    end
    methods 
        function obj = MAML_Nstep_forget(N)
            obj.Wi = zeros(N,1);
        end
        function [obj,Er] = MAML_initial(obj,Fx,Di,mu,lamda)
            Fx  = flipud(Fx);
            Dis = flipud(Di); 
            Len = size(Dis,1);
            Grad = 0;
            Er   = 0;
            Li   = 512 ;
            for ii = 1:Li
                Fd = [Fx(ii:end);zeros(ii-1,1)];
                e  = Dis(ii) - obj.Wi'*Fd      ;
                Grad = Grad + (mu/Li)*e*Fd*(lamda^(ii-1)) ;
            end
            Wo = obj.Wi + Grad ;
            Grad = 0; 
            for jj = 1:Li
                Fd = [Fx(jj:end);zeros(jj-1,1)];
                e  = Dis(jj) - Wo'*Fd          ;
                Grad = Grad + 0.25*(mu/Li)*e*Fd*(lamda^(jj-1)); 
                if jj == 1
                    Er =  e ;
                end
            end
            obj.Wi = obj.Wi + Grad ;
        end
    end
    
end