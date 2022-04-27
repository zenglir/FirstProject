function  ui  = Control_input_calculation(H,Gi,Gji,x,XN,v,uL,Pi,SN)
%% 函数功能：计算控制输出ui




%% 系统控制量
%总控制量
ui  = zeros(SN,1);
%切换模型控制(低增益控制uir和扰动补偿控制uin)
uir = zeros(SN,1);    
uin = zeros(SN,1);
%滑模控制补偿
uis = zeros(SN,1);


%% 计算ui

for i = 1:SN
    uin(i) = -pinv(H(1:XN,1))*Gji(XN*(i-1)+1:XN*i,:)*x;
    uir(i) = -inv(H(1:XN,1)'*Pi*H(1:XN,1)+ Eye)*H(1:XN,1)'*Pi*Gi(XN*(i-1) +...
             1: XN*i,XN*(i-1) + 1: XN*i)*x(XN*(i-1) + 1: XN*i);
    if abs(uir(i)) <= uL
        ui(i) = v*uir(i) + uin(i);    % a = v*ui(i)+uin(i)
    else
        ui(i) = tanh(v*uir(i))*uL + uin(i); % a = sign(a)uL
    end
end

end

