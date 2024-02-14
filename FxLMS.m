function Er = FxLMS(Len_Filter, Wc_initial, Dis, Rf,muw)
N   = Len_Filter  ;
% Dis = Dis_1;
% Rf  = Rf_1 ;
Wc  = Wc_initial;
XD  = zeros(N,1);
% Num = length(Rf(Len_Filter:end,1));
    Er  = zeros(length(Rf),1);
    for tt = 1:length(Rf) 
        XD   = [Rf(tt);XD(1:end-1)];
      %  Rf_i = fliplr(Rf(tt-511:tt)');
        Rf_i = XD'    ;
        Rf_i = Rf_i' ;
        y_t  = Wc'*Rf_i ;
        e    = Dis(tt)-y_t ;
        Er(tt) = e    ;
        Wc   = Wc + muw*e*Rf_i ;
    end
end