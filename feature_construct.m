clc;
close all;
clear;
%% 
feature = zeros(14484,144);
label = zeros(14484,1);
for cnum = 1:14484
    k=1;
    Cells.cell(cnum).CDVL = length(Cells.cell(cnum).CCD.V);
    Cells.cell(cnum).CCVL = length(Cells.cell(cnum).CCV.V);
    Cells.cell(cnum).Q = Cells.cell(cnum).CCD.Q(Cells.cell(cnum).CDVL);
    % 对恒流和恒压阶段进行识别
    cc_end = find(Cells.cell(cnum).CCV.V>=3649,1)-1;
    if isempty(cc_end)
        continue
    end
    cv_st = find(Cells.cell(cnum).CCV.V>=3648,1);
    cv_end = Cells.cell(cnum).CCVL;
    if isempty(cv_st)
        continue
    end
    if cv_st ==1
        continue
    end
    % 特征提取节
    %电压相关特征
    V_CC = Cells.cell(cnum).CCV.V(1:cc_end);
    V_CCCV = Cells.cell(cnum).CCV.V;
    %F1 V_CC_MAX
    V_CC_MAX = max(V_CC);
    % F2 V_CC_MIN
    V_CC_MIN = min(V_CC);
    % F3 V_CC_quan_1_3
    V_CC_quan_1_3 = quantile(V_CC,1/3);
    % F4 V_CC_quan_2_3
    V_CC_quan_2_3 = quantile(V_CC,2/3);
    % F5 V_CC_quan_1_2
    V_CC_quan_1_2 = quantile(V_CC,0.5);
    % F6 V_CC_quan_1_5
    V_CC_quan_1_5 = quantile(V_CC,0.2);
    % F7 V_CC_quan_2_5
    V_CC_quan_2_5 = quantile(V_CC,0.4);
    % F8 V_CC_quan_3_5
    V_CC_quan_3_5 = quantile(V_CC,0.6);
    % F9 V_CC_quan_4_5
    V_CC_quan_4_5 = quantile(V_CC,0.8);
    % F10 V_CC_MEAN
    V_CC_MEAN = mean(V_CC);
    % F11 V_CC_VAR
    V_CC_VAR = var(V_CC);
    % F12 V_CC_skew
    V_CC_skew = skewness(V_CC);
    % F13 V_CC_kurt
    V_CC_kurt = kurtosis(V_CC);
    % F14 V_CC_slope
    V_CC_slope = (V_CC(cc_end)-V_CC(1))./ cc_end;
    % F15 V_CCCV_MAX
    V_CCCV_MAX = max(V_CCCV);
    % F16 V_CCCV_MAX
    V_CCCV_MIN = min(V_CCCV);
    % F17 V_CCCV_quan_1_3
    V_CCCV_quan_1_3 = quantile(V_CCCV,1/3);
    % F18 V_CCCV_quan_2_3
    V_CCCV_quan_2_3 = quantile(V_CCCV,2/3);
    % F19 V_CCCV_quan_1_2
    V_CCCV_quan_1_2 = quantile(V_CCCV,0.5);
    % F20 V_CCCV_quan_1_5
    V_CCCV_quan_1_5 = quantile(V_CCCV,0.2);
    % F21 V_CCCV_quan_2_5
    V_CCCV_quan_2_5 = quantile(V_CCCV,0.4);
    % F22 V_CCCV_quan_3_5
    V_CCCV_quan_3_5 = quantile(V_CCCV,0.6);
    % F23 V_CCCV_quan_4_5
    V_CCCV_quan_4_5 = quantile(V_CCCV,0.8);
    % F24 V_CCCV_MEAN
    V_CCCV_MEAN = mean(V_CCCV);
    % F25 V_CCCV_VAR
    V_CCCV_VAR = var(V_CCCV);
    % F26 V_CCCV_skew
    V_CCCV_skew = skewness(V_CCCV);
    % F27 V_CCCV_kurt
    V_CCCV_kurt = kurtosis(V_CCCV);
    % F28 V_CCCV_slope
    V_CCCV_slope = (V_CCCV(Cells.cell(cnum).CCVL)-V_CCCV(1))./Cells.cell(cnum).CCVL;
    
    %电流相关特征
    I_CV = Cells.cell(cnum).CCV.I(cv_st:Cells.cell(cnum).CCVL);
    I_CCCV = Cells.cell(cnum).CCV.I;
    % F29 I_CV_MAX
    I_CV_MAX = max(I_CV);
    % F30 I_CV_slope_st
    I_CV_slope_st = (I_CV(1)-I_CV(2))./1;
    % F31 I_CV_quan_1_3
    I_CV_quan_1_3 = quantile(I_CV,1/3);
    % F32 I_CV_quan_2_3
    I_CV_quan_2_3 = quantile(I_CV,2/3);
    % F33 I_CV_quan_1_2
    I_CV_quan_1_2 = quantile(I_CV,0.5);
    % F34 I_CV_quan_1_5
    I_CV_quan_1_5 = quantile(I_CV,0.2);
    % F35 I_CV_quan_2_5
    I_CV_quan_2_5 = quantile(I_CV,0.4);
    % F35 I_CV_quan_3_5
    I_CV_quan_3_5 = quantile(I_CV,0.6);
    % F36 I_CV_quan_4_5
    I_CV_quan_4_5 = quantile(I_CV,0.8);
    % F37 I_CV_MEAN
    I_CV_MEAN = mean(I_CV);
    % F38 I_CV_VAR
    I_CV_VAR = var(I_CV);
    % F39 I_CV_skew
    I_CV_skew = skewness(I_CV);
    % F40 I_CV_kurt
    I_CV_kurt = kurtosis(I_CV);
    % F41 I_CV_slope
    I_CV_slope = (Cells.cell(cnum).CCV.I(Cells.cell(cnum).CCVL)-Cells.cell(cnum).CCV.I(cv_st))./(Cells.cell(cnum).CCVL-cv_st);
    % F42 I_CCCV_MAX
    I_CCCV_MAX = max(I_CCCV);
    % F43 I_CCCV_MIN
    I_CCCV_MIN = min(I_CCCV);% 全是零值没有学习意义，不予纳入特征集
    % F44 I_CCCV_quan_1_3
    I_CCCV_quan_1_3 = quantile(I_CCCV,1/3);
    % F45 I_CCCV_quan_2_3
    I_CCCV_quan_2_3 = quantile(I_CCCV,2/3);
    % F46 I_CCCV_quan_1_2
    I_CCCV_quan_1_2 = quantile(I_CCCV,0.5);
    % F47 I_CCCV_quan_1_5
    I_CCCV_quan_1_5 = quantile(I_CCCV,0.2);
    % F48 I_CCCV_quan_2_5
    I_CCCV_quan_2_5 = quantile(I_CCCV,0.4);
    % F49 I_CCCV_quan_3_5
    I_CCCV_quan_3_5 = quantile(I_CCCV,0.6);
    % F50 I_CCCV_quan_4_5
    I_CCCV_quan_4_5 = quantile(I_CCCV,0.8);
    % F51 I_CCCV_MEAN
    I_CCCV_MEAN = mean(I_CCCV);
    % F52 I_CCCV_VAR 
    I_CCCV_VAR = var(I_CCCV);
    % F53 I_CCCV_skew
    I_CCCV_skew = skewness(I_CCCV);
    % F54 I_CCCV_kurt
    I_CCCV_kurt = kurtosis(I_CCCV);
    % F55 I_CCCV_slope
    I_CCCV_slope = (I_CCCV(Cells.cell(cnum).CCVL)-I_CCCV(1))./Cells.cell(cnum).CCVL;
    
    % 容量相关特征
    Q_CC = Cells.cell(cnum).CCV.Q(1:cc_end);
    Q_CCCV = Cells.cell(cnum).CCV.Q;
    Q_CV = Q_CCCV(cv_st:cv_end);
    % F56 Q_CC_MAX
    Q_CC_MAX = max(Q_CC);
    % F57 Q_CC_quan_1_3
    Q_CC_quan_1_3 = quantile(Q_CC,1/3);
    % F58 Q_CC_quan_2_3
    Q_CC_quan_2_3 = quantile(Q_CC,2/3);
    % F59 Q_CC_quan_1_2
    Q_CC_quan_1_2 = quantile(Q_CC,0.5);
    % F60 Q_CC_quan_1_5
    Q_CC_quan_1_5 = quantile(Q_CC,0.2);
    % F64 Q_CC_quan_2_5
    Q_CC_quan_2_5 = quantile(Q_CC,0.4);
    % F65 Q_CC_quan_3_5
    Q_CC_quan_3_5 = quantile(Q_CC,0.6);
    % F66 Q_CC_quan_4_5`
    Q_CC_quan_4_5 = quantile(Q_CC,0.8);
    % F67 Q_CC_MEAN
    Q_CC_MEAN = mean(Q_CC);
    % F68 Q_CC_VAR
    Q_CC_VAR = var(Q_CC);
    % F69 Q_CC_skew
    Q_CC_skew = skewness(Q_CC);
    % F70 Q_CC_kurt
    Q_CC_kurt = kurtosis(Q_CC);
    % F71 Q_CC_slope
    Q_CC_slope = (Q_CC(cc_end)-Q_CC(1))./cc_end;
    % F72 Q_CV_quan_1_3
    Q_CV_quan_1_3 = quantile(Q_CV,1/3);
    % F73 Q_CV_quan_2_3
    Q_CV_quan_2_3 = quantile(Q_CV,2/3);
    % F74 Q_CV_quan_1_2
    Q_CV_quan_1_2 = quantile(Q_CV,0.5);
    % F75 Q_CV_quan_1_5
    Q_CV_quan_1_5 = quantile(Q_CV,0.2);
    % F76 Q_CV_quan_2_5
    Q_CV_quan_2_5 = quantile(Q_CV,0.4);
    % F77 Q_CV_quan_3_5
    Q_CV_quan_3_5 = quantile(Q_CV,0.6);
    % F78 Q_CV_quan_4_5
    Q_CV_quan_4_5 = quantile(Q_CV,0.8);
    % F79 Q_CV_MEAN
    Q_CV_MEAN = mean(Q_CV);
    % F80 Q_CV_VAR 
    Q_CV_VAR = var(Q_CV);
    % F81 Q_CV_skew
    Q_CV_skew = skewness(Q_CV);
    % F82 Q_CV_kurt
    Q_CV_kurt = kurtosis(Q_CV);
    % F83 Q_CV_slope
    Q_CV_slope = (Q_CV(length(Q_CV))-Q_CV(1))./(length(Q_CV));
    % F84 Q_CCCV_quan_1_3
    Q_CCCV_quan_1_3 = quantile(Q_CCCV,1/3);
    % F85 Q_CCCV_quan_2_3
    Q_CCCV_quan_2_3 = quantile(Q_CCCV,2/3);
    % F86 Q_CCCV_quan_1_2
    Q_CCCV_quan_1_2 = quantile(Q_CCCV,0.5);
    % F87 Q_CCCV_quan_1_5
    Q_CCCV_quan_1_5 = quantile(Q_CCCV,0.2);
    % F88 Q_CCCV_quan_2_5
    Q_CCCV_quan_2_5 = quantile(Q_CCCV,0.4);
    % F89 Q_CCCV_quan_3_5
    Q_CCCV_quan_3_5 = quantile(Q_CCCV,0.6);
    % F90 Q_CCCV_quan_4_5
    Q_CCCV_quan_4_5 = quantile(Q_CCCV,0.8);
    % F91 Q_CCCV_MEAN
    Q_CCCV_MEAN = mean(Q_CCCV);
    % F92 Q_CCCV_VAR
    Q_CCCV_VAR = var(Q_CCCV);
    % F93 Q_CCCV_skew
    Q_CCCV_skew = skewness(Q_CCCV);
    % F94 Q_CCCV_kurt
    Q_CCCV_kurt = kurtosis(Q_CCCV);
    % F95 Q_CCCV_slope
    Q_CCCV_slope = (Q_CCCV(cv_end)-Q_CCCV(1))./cv_end;
    
    % 能量相关特征
    E_CC = Cells.cell(cnum).CCV.E(1:cc_end);
    E_CV = Cells.cell(cnum).CCV.E(cv_st:cv_end);
    E_CCCV = Cells.cell(cnum).CCV.E;
    % F96 E_CC_MAX
    E_CC_MAX = max(E_CC);
    % F97 E_CC_quan_1_3
    E_CC_quan_1_3 = quantile(E_CC,1/3);
    % F98 E_CC_quan_2_3
    E_CC_quan_2_3 = quantile(E_CC,2/3);
    % F99 E_CC_quan_1_2
    E_CC_quan_1_2 = quantile(E_CC,0.5);
    % F100 E_CC_quan_1_5
    E_CC_quan_1_5 = quantile(E_CC,0.2);
    % F101 E_CC_quan_2_5
    E_CC_quan_2_5 = quantile(E_CC,0.4);
    % F102 E_CC_quan_3_5
    E_CC_quan_3_5 = quantile(E_CC,0.6);
    % F103 E_CC_quan_4_5
    E_CC_quan_4_5 = quantile(E_CC,0.8);
    % F104 E_CC_MEAN
    E_CC_MEAN = mean(E_CC);
    % F105 E_CC_VAR
    E_CC_VAR = var(E_CC);
    % F106 E_CC_skew
    E_CC_skew = skewness(E_CC);
    % F107 E_CC_kurt
    E_CC_kurt = kurtosis(E_CC);
    % F108 E_CV_MAX
    E_CV_MAX = max(E_CV);
    % F109 E_CV_quan_1_3
    E_CV_quan_1_3 = quantile(E_CV,1/3);
    % F110 E_CV_quan_2_3
    E_CV_quan_2_3 = quantile(E_CV,2/3);
    % F111 E_CV_quan_1_2
    E_CV_quan_1_2 = quantile(E_CV,0.5);
    % F112 E_CV_quan_1_5
    E_CV_quan_1_5 = quantile(E_CV,0.2);
    % F113 E_CV_quan_2_5
    E_CV_quan_2_5 = quantile(E_CV,0.4);
    % F114 E_CV_quan_3_5
    E_CV_quan_3_5 = quantile(E_CV,0.6);
    % F115 E_CV_quan_4_5
    E_CV_quan_4_5 = quantile(E_CV,0.8);
    % F116 E_CV_MEAN
    E_CV_MEAN = mean(E_CV);
    % F117 E_CV_VAR
    E_CV_VAR = var(E_CV);
    % F118 E_CV_skew
    E_CV_skew = skewness(E_CV);
    % F119 E_CV_kurt
    E_CV_kurt = kurtosis(E_CV);
    % F120 E_CCCV_quan_1_3
    E_CCCV_quan_1_3 = quantile(E_CCCV,1/3);
    % F121 E_CCCV_quan_2_3
    E_CCCV_quan_2_3 = quantile(E_CCCV,2/3);
    % F122 E_CCCV_quan_1_2
    E_CCCV_quan_1_2 = quantile(E_CCCV,0.5);
    % F133 E_CCCV_quan_1_5
    E_CCCV_quan_1_5 = quantile(E_CCCV,0.2);
    % F134 E_CCCV_quan_2_5
    E_CCCV_quan_2_5 = quantile(E_CCCV,0.4);
    % F135 E_CCCV_quan_3_5
    E_CCCV_quan_3_5 = quantile(E_CCCV,0.6);
    % F136 E_CCCV_quan_4_5
    E_CCCV_quan_4_5 = quantile(E_CCCV,0.8);
    % F137 E_CCCV_MEAN
    E_CCCV_MEAN = mean(E_CCCV);
    % F138 E_CCCV_skew
    E_CCCV_skew = skewness(E_CCCV);
    % F139 E_CCCV_kurt
    E_CCCV_kurt = kurtosis(E_CCCV);
    
    % IC曲线特征
    % IC曲线提取
    if Cells.cell(cnum).CCVL<100
        continue
    end
    % 对恒流和恒压阶段进行识别
    % 通过定义函数进行IC曲线提取
    n=1;
    dv=4;%电压变化步长
    CDV=Cells.cell(cnum).CCD.V;
    CDQ=Cells.cell(cnum).CCD.Q;
    t=1;%时间序列起始时刻
        ICV=[];%清空上一次循环留下来的值
        IC=[];%清空上一次循环留下来的值
        nnn=[];%清空上一次循环留下来的值
    while(t<length(CDV))
        dd=0;
        dt=0;
        while(dd<dv && t+dt<length(CDV))          
               dt=dt+1;%变化dv对应的时间变化
               dd=abs(CDV(t+dt,1)-CDV(t,1));
        end
        if t+dt>=length(CDV)
                break;
        end
            ICV(n)=CDV(t,1);
            dV(n)=CDV(t+dt,1)-CDV(t,1);
            dQ(n)=CDQ(t+dt,1)-CDQ(t,1);
            IC(n)=abs(dQ(n)/dV(n));
            IC_filtered = gaussfilt(IC,2);
            t=t+dt;
            n=n+1;
    end
    threshold = 0.05;
    [peaks,peaklocations] = extract_peak(IC_filtered,threshold);
    [~, maxIndex] = max(peaks);
    maxpeaklocation = peaklocations(maxIndex);
    nnn=zeros(length(IC),1);
    nnn(:,1)=cnum;
    % F140 IC_MAX
    IC_MAX = max(IC_filtered);
    % F141 IC_MIN
    IC_MIN = min(IC_filtered);
    % F142 IC_quan_1_3
    IC_quan_1_3 = quantile(IC_filtered,1/3);
    % F143 IC_quan_2_3
    IC_quan_2_3 = quantile(IC_filtered,2/3);
    % F144 IC_quan_1_2
    IC_quan_1_2 = quantile(IC_filtered,0.5);
    % F145 IC_quan_1_5
    IC_quan_1_5 = quantile(IC_filtered,0.2);
    % F146 IC_quan_2_5
    IC_quan_2_5 = quantile(IC_filtered,0.4);
    % F147 IC_quan_3_5
    IC_quan_3_5 = quantile(IC_filtered,0.6);
    % F148 IC_quan_4_5
    IC_quan_4_5 = quantile(IC_filtered,0.8);
    % F150 IC_MEAN
    IC_MEAN = mean(IC_filtered);
    % F151 IC_VAR
    IC_VAR = var(IC_filtered);
    % F152 IC_skew
    IC_skew = skewness(IC_filtered);
    % F153 IC_kurt
    IC_kurt = kurtosis(IC_filtered);
    % F154 IC_peak
    IC_peak = max(peaks);
    if isempty(IC_peak)
        continue
    end
    % F155 IC_peaklocation
    IC_peaklocation = ICV(maxpeaklocation);
    % F156 IC_lslope
    IC_lslope = (IC(maxpeaklocation)-IC(maxpeaklocation-1))./dv;
    % F157 IC_rslope
    IC_rslope = (IC(maxpeaklocation+1)-IC(maxpeaklocation))./dv;
    % F158 IC_peakarea
    IC_peakarea = trapz(IC(maxpeaklocation-1:maxpeaklocation+1));
    
    % 标签提取节
    % CDV阶段放电容量提取
    Q_disc = Cells.cell(cnum).CCD.Q;
    Q_label = Cells.cell(cnum).Q;
    
    % 特征构建节
    feature(cnum,1) = V_CC_MAX;
    feature(cnum,2) = V_CC_MIN;
    feature(cnum,3) = V_CC_quan_1_3;
    feature(cnum,4) = V_CC_quan_2_3;
    feature(cnum,5) = V_CC_quan_1_2;
    feature(cnum,6) = V_CC_quan_1_5;
    feature(cnum,7) = V_CC_quan_2_5;
    feature(cnum,8) = V_CC_quan_3_5;
    feature(cnum,9) = V_CC_quan_4_5;
    feature(cnum,10) = V_CC_MEAN;
    feature(cnum,11) = V_CC_VAR;
    feature(cnum,12) = V_CC_skew;
    feature(cnum,13) = V_CC_kurt;
    feature(cnum,14) = V_CC_slope;
    feature(cnum,15) = V_CCCV_MAX;
    feature(cnum,16) = V_CCCV_MIN;
    feature(cnum,17) = V_CCCV_quan_1_3;
    feature(cnum,18) = V_CCCV_quan_2_3;
    feature(cnum,19) = V_CCCV_quan_1_2;
    feature(cnum,20) = V_CCCV_quan_1_5;
    feature(cnum,21) = V_CCCV_quan_2_5;
    feature(cnum,22) = V_CCCV_quan_3_5;
    feature(cnum,23) = V_CCCV_quan_4_5;
    feature(cnum,24) = V_CCCV_MEAN;
    feature(cnum,25) = V_CCCV_VAR;
    feature(cnum,26) = V_CCCV_skew;
    feature(cnum,27) = V_CCCV_kurt;
    feature(cnum,28) = V_CCCV_slope;
    feature(cnum,29) = I_CV_MAX;
    feature(cnum,30) = I_CV_slope_st;
    feature(cnum,31) = I_CV_quan_1_3;
    feature(cnum,32) = I_CV_quan_2_3;
    feature(cnum,33) = I_CV_quan_1_2;
    feature(cnum,34) = I_CV_quan_1_5;
    feature(cnum,35) = I_CV_quan_2_5;
    feature(cnum,36) = I_CV_quan_3_5;
    feature(cnum,37) = I_CV_quan_4_5;
    feature(cnum,38) = I_CV_MEAN;
    feature(cnum,39) = I_CV_VAR;
    feature(cnum,40) = I_CV_skew;
    feature(cnum,41) = I_CV_kurt;
    feature(cnum,42) = I_CV_slope;
    feature(cnum,43) = I_CCCV_MAX;
    feature(cnum,44) = I_CCCV_quan_1_3;
    feature(cnum,45) = I_CCCV_quan_2_3;
    feature(cnum,46) = I_CCCV_quan_1_2;
    feature(cnum,47) = I_CCCV_quan_1_5;
    feature(cnum,48) = I_CCCV_quan_2_5;
    feature(cnum,49) = I_CCCV_quan_3_5;
    feature(cnum,50) = I_CCCV_quan_4_5;
    feature(cnum,51) = I_CCCV_MEAN;
    feature(cnum,52) = I_CCCV_VAR;
    feature(cnum,53) = I_CCCV_skew;
    feature(cnum,54) = I_CCCV_kurt;
    feature(cnum,55) = I_CCCV_slope;
    feature(cnum,56) = Q_CC_MAX;
    feature(cnum,57) = Q_CC_quan_1_3;
    feature(cnum,58) = Q_CC_quan_2_3;
    feature(cnum,59) = Q_CC_quan_1_2;
    feature(cnum,60) = Q_CC_quan_1_5;
    feature(cnum,61) = Q_CC_quan_2_5;
    feature(cnum,62) = Q_CC_quan_3_5;
    feature(cnum,63) = Q_CC_quan_4_5;
    feature(cnum,64) = Q_CC_MEAN;
    feature(cnum,65) = Q_CC_VAR;
    feature(cnum,66) = Q_CC_skew;
    feature(cnum,67) = Q_CC_kurt;
    feature(cnum,68) = Q_CC_slope;
    feature(cnum,69) = Q_CV_quan_1_3;
    feature(cnum,70) = Q_CV_quan_2_3;
    feature(cnum,71) = Q_CV_quan_1_2;
    feature(cnum,72) = Q_CV_quan_1_5;
    feature(cnum,73) = Q_CV_quan_2_5;
    feature(cnum,74) = Q_CV_quan_3_5;
    feature(cnum,75) = Q_CV_quan_4_5;
    feature(cnum,76) = Q_CV_MEAN;
    feature(cnum,77) = Q_CV_VAR;
    feature(cnum,78) = Q_CV_skew;
    feature(cnum,79) = Q_CV_kurt;
    feature(cnum,80) = Q_CV_slope;
    feature(cnum,81) = Q_CCCV_quan_1_3;
    feature(cnum,82) = Q_CCCV_quan_2_3;
    feature(cnum,83) = Q_CCCV_quan_1_2;
    feature(cnum,84) = Q_CCCV_quan_1_5;
    feature(cnum,85) = Q_CCCV_quan_2_5;
    feature(cnum,86) = Q_CCCV_quan_3_5;
    feature(cnum,87) = Q_CCCV_quan_4_5;
    feature(cnum,88) = Q_CCCV_MEAN;
    feature(cnum,89) = Q_CCCV_VAR;
    feature(cnum,90) = Q_CCCV_skew;
    feature(cnum,91) = Q_CCCV_kurt;
    feature(cnum,92) = Q_CCCV_slope;
    feature(cnum,93) = E_CC_MAX;
    feature(cnum,94) = E_CC_quan_1_3;
    feature(cnum,95) = E_CC_quan_2_3;
    feature(cnum,96) = E_CC_quan_1_2;
    feature(cnum,97) = E_CC_quan_1_5;
    feature(cnum,98) = E_CC_quan_2_5;
    feature(cnum,99) = E_CC_quan_3_5;
    feature(cnum,100) = E_CC_quan_4_5;
    feature(cnum,101) = E_CC_MEAN;
    feature(cnum,102) = E_CC_VAR;
    feature(cnum,103) = E_CC_skew;
    feature(cnum,104) = E_CC_kurt;
    feature(cnum,105) = E_CV_MAX;
    feature(cnum,106) = E_CV_quan_1_3;
    feature(cnum,107) = E_CV_quan_2_3;
    feature(cnum,108) = E_CV_quan_1_2;
    feature(cnum,109) = E_CV_quan_1_5;
    feature(cnum,110) = E_CV_quan_2_5;
    feature(cnum,111) = E_CV_quan_3_5;
    feature(cnum,112) = E_CV_quan_4_5;
    feature(cnum,113) = E_CV_MEAN;
    feature(cnum,114) = E_CV_VAR;
    feature(cnum,115) = E_CV_skew;
    feature(cnum,116) = E_CV_kurt;
    feature(cnum,117) = E_CCCV_quan_1_3;
    feature(cnum,118) = E_CCCV_quan_2_3;
    feature(cnum,119) = E_CCCV_quan_1_2;
    feature(cnum,120) = E_CCCV_quan_1_5;
    feature(cnum,121) = E_CCCV_quan_2_5;
    feature(cnum,122) = E_CCCV_quan_3_5;
    feature(cnum,123) = E_CCCV_quan_4_5;
    feature(cnum,124) = E_CCCV_MEAN;
    feature(cnum,125) = E_CCCV_skew;
    feature(cnum,126) = E_CCCV_kurt;
    feature(cnum,127) = IC_MAX;
    feature(cnum,128) = IC_MIN;
    feature(cnum,129) = IC_quan_1_3;
    feature(cnum,130) = IC_quan_2_3;
    feature(cnum,131) = IC_quan_1_2;
    feature(cnum,132) = IC_quan_1_5;
    feature(cnum,133) = IC_quan_2_5;
    feature(cnum,134) = IC_quan_3_5;
    feature(cnum,135) = IC_quan_4_5;
    feature(cnum,136) = IC_MEAN;
    feature(cnum,137) = IC_VAR;
    feature(cnum,138) = IC_skew;
    feature(cnum,139) = IC_kurt;
    feature(cnum,140) = IC_peak;
    feature(cnum,141) = IC_peaklocation;
    feature(cnum,142) = IC_lslope;
    feature(cnum,143) = IC_rslope;
    feature(cnum,144) = IC_peakarea;
    label(cnum,1) = Q_label;
end