function [ temp ] = TriggerConditions(TrigerXsave,x,gamai,mui,flag)
%% ���㴥������
ei = x - TrigerXsave(2:1:end,flag-1);
NormE = norm(ei);
NormX = gamai*norm(x)+mui;
%������㴥����������temp��Ϊ1
if NormE > NormX
    temp = true;
%��������㣬��temp��Ϊ0
else
    temp = false;
end

end

