function [ temp ] = TriggerConditions(TrigerXsave,x,gamai,mui,flag)
%% 计算触发条件
ei = x - TrigerXsave(2:1:end,flag-1);
NormE = norm(ei);
NormX = gamai*norm(x)+mui;
%如果满足触发条件，则将temp置为1
if NormE > NormX
    temp = true;
%如果不满足，则将temp置为0
else
    temp = false;
end

end

