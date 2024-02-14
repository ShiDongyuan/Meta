%  __________   ___  __      .___  ___.      _______.
% |   ____\  \ /  / |  |     |   \/   |     /       |
% |  |__   \  V  /  |  |     |  \  /  |    |   (----`
% |   __|   >   <   |  |     |  |\/|  |     \   \
% |  |     /  .  \  |  `----.|  |  |  | .----)   |
% |__|    /__/ \__\ |_______||__|  |__| |_______/

%% Title : The single-channel FxLMS algorithm 
% Author: DONGYAN SHI(DSHI003@ntu.edu.sg)
% Date  : 2020-10-1

%% single-channel FxLMS algorithm 
function Er = FxLMS(Len_Filter, Wc_initial, Dis, Rf, muw)
% Len_Filter : the length of the control filter 
% Wc_initial : the initial control filter 
% Dis        : the disturbance 
% Rf         : the filtered reference vector 
% muw        : the step size 
N   = Len_Filter ;
Wc  = Wc_initial ;
XD  = zeros(N,1) ;
Er  = zeros(length(Rf),1);
    for tt = 1:length(Rf) 
        XD   = [Rf(tt);XD(1:end-1)];
        Rf_i = XD'         ;
        Rf_i = Rf_i'       ;
        y_t  = Wc'*Rf_i    ;
        e    = Dis(tt)-y_t ;
        Er(tt) = e         ;
        Wc     = Wc + muw*e*Rf_i;
    end
end
%-------------------------end-----------------------------