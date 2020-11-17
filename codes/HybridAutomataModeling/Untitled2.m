% FSR_CV = [FSR_CV_2; FSR_CV];
% FSR_HBase = [FSR_HBase_2; FSR_HBase];
% FSR_MHP = [FSR_MHP_2; FSR_MHP];
% 
% LL_CV = [LL_CV_2; LL_CV];
% LL_HBase = [LL_HBase_2; LL_HBase];
% LL_MHP = [LL_MHP_2; LL_MHP];
% 
% PP_CV = [PP_CV_2; PP_CV];
% PP_HBase = [PP_HBase_2; PP_HBase];
% PP_MHP = [PP_MHP_2; PP_MHP];



for ii=1:30
    
    temp = PP_MHP_unique(:,ii);
    temp(temp==0 | temp==-1) = [];
    MeanPP_MHP(1,ii) = mean(temp);
    
%     
%     temp = LL_CV_unique(:,ii);
%     temp(temp==-1) = [];
%     MeanLL_CV(1,ii) = mean(temp);
%     
    
    
    
end