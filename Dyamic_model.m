function [ dx,Wi ] = Dyamic_model( G, H, J, branches, x, u)
% Dyamic_model ：搭建控制系统动力学模型
%  输入：G，H，J, branches, x, u
%  输出：k+1时刻下各个子系统的状态x(k+1)

%% 初始化参数
XN = 5;       %状态量xi的维度


%% 引入电网负荷频率控制系统一般模型(不包含xj(k)部分)
Wi = rand(size(x))-0.5;
dx = G*x + H*u + J*Wi;

for i=1:size(branches,2)
    k=branches(i,1);
    l=branches(i,2);
    dx((k-1)*XN+1)= dx((k-1)*XN+1)+branches(i,4)*sin(x((k-1)*XN+5)-x((l-1)*XN+5));
    dx((l-1)*XN+1)= dx((l-1)*XN+1)+branches(i,4)*sin(x((l-1)*XN+5)-x((k-1)*XN+5));
end
end

