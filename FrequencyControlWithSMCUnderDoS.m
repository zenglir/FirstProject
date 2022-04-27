clc;
close all;
clear;

%% 系统参数初始化 xi = [ fi ?Pmi ?Pvi ?Ptie-i ∫ACEi ]'
T=0.5;D=0.26;M=0.2;Td=5;Te=0.2;
Th=4.5;Tg=0.2;R=0.5;K=0.1;ag=0.5;
ae=0.25;ah=0.25;Ti=1;

% 子系统参数矩阵（向量）
Ai=[-D/M 1/M 0  0 0;
    0   -1/Td 1/Td 0 0;
    1/(Tg*R) 0 -1/Tg K/Tg 0;
    -Ti 0 0 0 0;
    1 0 0 0 0];
Bi=[ 2/M;0;0;0; 0];
Ei = zeros(size(Ai));
Ei(1,1) = -1/M/5;

% Matpower信息
CA = case14;
branches = CA.branch; 
SN = size(CA.bus,1);

% 总系统表达式参数A、B、E
A = zeros(size(Ai,1)*SN,size(Ai,2)*SN);
B = zeros(size(Bi,1)*SN,size(Bi,2)*SN);
E = zeros(size(Ei,1)*SN,size(Ei,2)*SN);

% 利用Ai、Bi、Ei计算A、B、E
for i=1:size(branches,1)
    k=branches(i,1);
    l=branches(i,2);
    A((k-1)*size(Ai,1)+2,(l-1)*size(Ai,2)+1)=branches(i,4);
    A((l-1)*size(Ai,1)+2,(k-1)*size(Ai,2)+1)=branches(i,4);
end

for i=1:SN
    A((i-1)*size(Ai,1)+1:i*size(Ai,1),(i-1)*size(Ai,2)+1:i*size(Ai,2)) = Ai;
    A((i-1)*size(Ai,1)+1,(i-1)*size(Ai,2)+2) = -sum( A((i-1)*size(Ai,1)+2,(i-1)*size(Ai,2)+6:end))...
                                             -sum(A((i-1)*size(Ai,1)+2,1:(i-1)*size(Ai,2)));
    if( A((i-1)*size(Ai,1)+1,(i-1)*size(Ai,2)+2)~=0)
             A((i-1)*size(Ai,1)+2,(i-1)*size(Ai,2)+6:end) = -A((i-1)*size(Ai,1)+2,(i-1)*size(Ai,2)+6:end)/ ...
                                                           A((i-1)*size(Ai,1)+1,(i-1)*size(Ai,2)+2)/M;
             A((i-1)*size(Ai,1)+2,1:(i-1)*size(Ai,2)) = -A((i-1)*size(Ai,1)+2,1:(i-1)*size(Ai,2))/ ...
                                                       A((i-1)*size(Ai,1)+1,(i-1)*size(Ai,2)+2)/M;
    end
    B((i-1)*size(Bi,1)+1:i*size(Bi,1),(i-1)*size(Bi,2)+1:i*size(Bi,2)) = Bi;
    E((i-1)*size(Ei,1)+1:i*size(Ei,1),(i-1)*size(Ei,2)+1:i*size(Ei,2)) = Ei;
end

% 设置起始控制时间、采样时间、仿真停止时间、DoS攻击时间
StartTime = 0;
EndTime = 12.5;
FaultTime = 0;
FaultClearTime = 0.8;
SampingTime = 0.05;
DoSTime = 5;
XN = size(Bi,1); 

% 离散化状态方程参数
[G,H] = c2d(A,B,SampingTime);
[~,J] = c2d(A,E,SampingTime);

% 从G中将Gji分离出来
Gji = G;
Temp = zeros(size(Ai));
for i=1:SN
    Gji((i-1)*size(Ai,1)+1:i*size(Ai,1),(i-1)*size(Ai,2)+1:i*size(Ai,2)) = Temp;
end

%% 初始化控制参数
% 低增益控制参数初始化
uL = 0.25;
v = 4;

% 李雅普诺夫函数参数初始化
I  = eye(size(Ai));
Ep = 0.43;  %0.45                                                      %Ep代表公式中的ε，这里选择自己给定一个（当不满足条件时，
                                                                 %重新设定一个新的数值）
Pi = care(A(1:XN,1:XN), B(1:XN,1), Ep*I,1);     %利用care()函数计算Pi
% 验证计算出来的Pi是否有效
TEST = A(1:XN,1:XN)'*Pi + Pi*B(1:XN,1:XN) - Pi*B(1:XN,1)*B(1:XN,1)'*Pi + Ep*I;
det(TEST)
% 事件触发机制参数初始化
gamai = 0.05;   %触发增益
mui = 0.01;     %最小触发阈值
ei = zeros(size(A,1),1);
TrigerXsave = zeros(size(A,1)+1,EndTime/SampingTime);

%% 初始化滑模控制的参数
Si = zeros(SN,EndTime/SampingTime);
bi = zeros(SN,EndTime/SampingTime);
Li = -pinv(H(1:XN,1));

%% 初始化系统状态，用于保存各个时刻状态x
x = zeros(size(Ai,1)*SN,1);
xSave = zeros(size(Ai,1)*SN, EndTime/SampingTime);
flag = 1;

%%各项控制量初始化
Eye = eye(size(H(1:XN,1),2),size(H(1:XN,1),2));
ui  = zeros(SN,1); %控制量
uir = zeros(SN,1); %切换模型控制(低增益控制uir和扰动补偿控制uin)
uin = zeros(SN,1);
uis = zeros(SN,1); %滑模控制补偿
u = zeros(SN,1);   %控制向量u
uSave = zeros(SN, EndTime/SampingTime);  %保存所有时刻下的控制向量
Uir_Uin = zeros(SN,EndTime/SampingTime); %保存所有时刻下的(低增益控制uir和扰动补偿控制uin)
K = 0.12;
UinSave = zeros(size(uin,1),EndTime/SampingTime);

%% 开始仿真
for k = 1: EndTime/SampingTime-1
    
    % 假设这是事件触发的第一个时刻
    if k == FaultClearTime/SampingTime
        TrigerXsave(1,flag) = k;
        TrigerXsave(2:1:end,flag) = x;
        flag = flag +1;
    
    % 将故障清除的下一时刻利用所设计的控制策略对系统状态进行控制
    elseif k == FaultClearTime/SampingTime + 1
        
        % 计算控制器输出分量
        for i =1:SN
            
            % 计算滑模控制 uis
            bi(i,k) = bi(i,k-1) - Li*(G(1:XN,1:XN) - eye(XN,XN))*xSave(XN*(i-1)+1:XN*i,k-1)-...
                Li*H(1:XN,1)*Uir_Uin(k-1);
            Si(i,k) = Li*xSave(XN*(i-1)+1:XN*i,k) - bi(i,k);
            uis(i)  = -K*tanh(H(1:XN,1)'*Li'*Si(i,k));
            
            % 计算uin和uir
            uin(i) = -2*pinv(H(1:XN,1))*Gji(XN*(i-1)+1:XN*i,:)*x;
            uir(i) = -inv(H(1:XN,1)'*Pi*H(1:XN,1)+ Eye)*H(1:XN,1)'*Pi*G(XN*(i-1) +...
                     1: XN*i,XN*(i-1) + 1: XN*i)*x(XN*(i-1) + 1: XN*i);
            if abs(uir(i)) <= uL
                ui(i) = v*uir(i) + uin(i);    % a = v*ui(i)+uin(i)
            else
                ui(i) = tanh(v*uir(i))*uL + uin(i); % a = sign(a)uL
            end
        end  
        u = ui + uis; %计算控制向量u
    
    % 在将来的时刻利用事件触发控制对系统状态进行控制
    elseif k>FaultClearTime/SampingTime+1&&k<DoSTime/SampingTime
        Temp = TriggerConditions(TrigerXsave,x,gamai,mui,flag);
        if Temp
            TrigerXsave(1,flag) = k;
            TrigerXsave(2:1:end,flag) = x;
            
            % 计算控制器输出分量
            for i =1:SN
                %计算滑模控制uis
                bi(i,k) = bi(i,k-1) - Li*(G(1:XN,1:XN) - eye(XN,XN))*xSave(XN*(i-1)+1:XN*i,k-1)-...
                    Li*H(1:XN,1)*Uir_Uin(k-1);
                Si(i,k) = Li*xSave(XN*(i-1)+1:XN*i,k) - bi(i,k);
                uis(i)  = -K*tanh(H(1:XN,1)'*Li'*Si(i,k));
                
                % 计算uin和uir
                uin(i) = -2*pinv(H(1:XN,1))*Gji(XN*(i-1)+1:XN*i,:)*x;
                uir(i) = -inv(H(1:XN,1)'*Pi*H(1:XN,1)+ Eye)*H(1:XN,1)'*Pi*G(XN*(i-1) +...
                         1: XN*i,XN*(i-1) + 1: XN*i)*x(XN*(i-1) + 1: XN*i);
                if abs(uir(i)) <= uL
                    ui(i) = v*uir(i) + uin(i);    % a = v*ui(i)+uin(i)
                else
                    ui(i) = tanh(v*uir(i))*uL + uin(i); % a = sign(a)uL
                end
            end
            u = ui + uis; %计算控制向量u
            flag = flag + 1;
        end

    % 此时子系统2将受到DoS攻击，与系统1,3,4,5之间的通信链路断开
    elseif k== DoSTime
        [G,H,J,Gji] = DenialofServiceAttack( A,Ai,branches,B,E,SampingTime);
        Pi = care(A(1:XN,1:XN), B(1:XN,1), Ep*I,1);
    elseif k>DoSTime
        for i =1:SN
            uin(i) = pinv(H(1:XN,1))*Gji(XN*(i-1)+1:XN*i,:)*x;
            ui(i) = uin(i);
        end
    end
    
    uSave(:,k) = u;   %保存当前时刻的控制向量u
    UinSave(:,flag) = uin;     %保存 sigma(v*uir)+uin
    next_x = Dyamic_model(G, H, J, branches, x, u);
    x = next_x;
    xSave(:,k+1) = x;
end

%t=0:SampingTime:EndTime-SampingTime;
plot(xSave(1:5:end,:)');title('state');
figure()
plot(uSave(2,:)');