clc;
close all;
clear;

%% ϵͳ������ʼ�� xi = [ fi ?Pmi ?Pvi ?Ptie-i ��ACEi ]'
T=0.5;D=0.26;M=0.2;Td=5;Te=0.2;
Th=4.5;Tg=0.2;R=0.5;K=0.1;ag=0.5;
ae=0.25;ah=0.25;Ti=1;

% ��ϵͳ��������������
Ai=[-D/M 1/M 0  0 0;
    0   -1/Td 1/Td 0 0;
    1/(Tg*R) 0 -1/Tg K/Tg 0;
    -Ti 0 0 0 0;
    1 0 0 0 0];
Bi=[ 2/M;0;0;0; 0];
Ei = zeros(size(Ai));
Ei(1,1) = -1/M/5;

% Matpower��Ϣ
CA = case14;
branches = CA.branch; 
SN = size(CA.bus,1);

% ��ϵͳ���ʽ����A��B��E
A = zeros(size(Ai,1)*SN,size(Ai,2)*SN);
B = zeros(size(Bi,1)*SN,size(Bi,2)*SN);
E = zeros(size(Ei,1)*SN,size(Ei,2)*SN);

% ����Ai��Bi��Ei����A��B��E
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

% ������ʼ����ʱ�䡢����ʱ�䡢����ֹͣʱ�䡢DoS����ʱ��
StartTime = 0;
EndTime = 12.5;
FaultTime = 0;
FaultClearTime = 0.8;
SampingTime = 0.05;
DoSTime = 5;
XN = size(Bi,1); 

% ��ɢ��״̬���̲���
[G,H] = c2d(A,B,SampingTime);
[~,J] = c2d(A,E,SampingTime);

% ��G�н�Gji�������
Gji = G;
Temp = zeros(size(Ai));
for i=1:SN
    Gji((i-1)*size(Ai,1)+1:i*size(Ai,1),(i-1)*size(Ai,2)+1:i*size(Ai,2)) = Temp;
end

%% ��ʼ�����Ʋ���
% ��������Ʋ�����ʼ��
uL = 0.25;
v = 4;

% ������ŵ����������ʼ��
I  = eye(size(Ai));
Ep = 0.43;  %0.45                                                      %Ep����ʽ�еĦţ�����ѡ���Լ�����һ����������������ʱ��
                                                                 %�����趨һ���µ���ֵ��
Pi = care(A(1:XN,1:XN), B(1:XN,1), Ep*I,1);     %����care()��������Pi
% ��֤���������Pi�Ƿ���Ч
TEST = A(1:XN,1:XN)'*Pi + Pi*B(1:XN,1:XN) - Pi*B(1:XN,1)*B(1:XN,1)'*Pi + Ep*I;
det(TEST)
% �¼��������Ʋ�����ʼ��
gamai = 0.05;   %��������
mui = 0.01;     %��С������ֵ
ei = zeros(size(A,1),1);
TrigerXsave = zeros(size(A,1)+1,EndTime/SampingTime);

%% ��ʼ����ģ���ƵĲ���
Si = zeros(SN,EndTime/SampingTime);
bi = zeros(SN,EndTime/SampingTime);
Li = -pinv(H(1:XN,1));

%% ��ʼ��ϵͳ״̬�����ڱ������ʱ��״̬x
x = zeros(size(Ai,1)*SN,1);
xSave = zeros(size(Ai,1)*SN, EndTime/SampingTime);
flag = 1;

%%�����������ʼ��
Eye = eye(size(H(1:XN,1),2),size(H(1:XN,1),2));
ui  = zeros(SN,1); %������
uir = zeros(SN,1); %�л�ģ�Ϳ���(���������uir���Ŷ���������uin)
uin = zeros(SN,1);
uis = zeros(SN,1); %��ģ���Ʋ���
u = zeros(SN,1);   %��������u
uSave = zeros(SN, EndTime/SampingTime);  %��������ʱ���µĿ�������
Uir_Uin = zeros(SN,EndTime/SampingTime); %��������ʱ���µ�(���������uir���Ŷ���������uin)
K = 0.12;
UinSave = zeros(size(uin,1),EndTime/SampingTime);

%% ��ʼ����
for k = 1: EndTime/SampingTime-1
    
    % ���������¼������ĵ�һ��ʱ��
    if k == FaultClearTime/SampingTime
        TrigerXsave(1,flag) = k;
        TrigerXsave(2:1:end,flag) = x;
        flag = flag +1;
    
    % �������������һʱ����������ƵĿ��Ʋ��Զ�ϵͳ״̬���п���
    elseif k == FaultClearTime/SampingTime + 1
        
        % ����������������
        for i =1:SN
            
            % ���㻬ģ���� uis
            bi(i,k) = bi(i,k-1) - Li*(G(1:XN,1:XN) - eye(XN,XN))*xSave(XN*(i-1)+1:XN*i,k-1)-...
                Li*H(1:XN,1)*Uir_Uin(k-1);
            Si(i,k) = Li*xSave(XN*(i-1)+1:XN*i,k) - bi(i,k);
            uis(i)  = -K*tanh(H(1:XN,1)'*Li'*Si(i,k));
            
            % ����uin��uir
            uin(i) = -2*pinv(H(1:XN,1))*Gji(XN*(i-1)+1:XN*i,:)*x;
            uir(i) = -inv(H(1:XN,1)'*Pi*H(1:XN,1)+ Eye)*H(1:XN,1)'*Pi*G(XN*(i-1) +...
                     1: XN*i,XN*(i-1) + 1: XN*i)*x(XN*(i-1) + 1: XN*i);
            if abs(uir(i)) <= uL
                ui(i) = v*uir(i) + uin(i);    % a = v*ui(i)+uin(i)
            else
                ui(i) = tanh(v*uir(i))*uL + uin(i); % a = sign(a)uL
            end
        end  
        u = ui + uis; %�����������u
    
    % �ڽ�����ʱ�������¼��������ƶ�ϵͳ״̬���п���
    elseif k>FaultClearTime/SampingTime+1&&k<DoSTime/SampingTime
        Temp = TriggerConditions(TrigerXsave,x,gamai,mui,flag);
        if Temp
            TrigerXsave(1,flag) = k;
            TrigerXsave(2:1:end,flag) = x;
            
            % ����������������
            for i =1:SN
                %���㻬ģ����uis
                bi(i,k) = bi(i,k-1) - Li*(G(1:XN,1:XN) - eye(XN,XN))*xSave(XN*(i-1)+1:XN*i,k-1)-...
                    Li*H(1:XN,1)*Uir_Uin(k-1);
                Si(i,k) = Li*xSave(XN*(i-1)+1:XN*i,k) - bi(i,k);
                uis(i)  = -K*tanh(H(1:XN,1)'*Li'*Si(i,k));
                
                % ����uin��uir
                uin(i) = -2*pinv(H(1:XN,1))*Gji(XN*(i-1)+1:XN*i,:)*x;
                uir(i) = -inv(H(1:XN,1)'*Pi*H(1:XN,1)+ Eye)*H(1:XN,1)'*Pi*G(XN*(i-1) +...
                         1: XN*i,XN*(i-1) + 1: XN*i)*x(XN*(i-1) + 1: XN*i);
                if abs(uir(i)) <= uL
                    ui(i) = v*uir(i) + uin(i);    % a = v*ui(i)+uin(i)
                else
                    ui(i) = tanh(v*uir(i))*uL + uin(i); % a = sign(a)uL
                end
            end
            u = ui + uis; %�����������u
            flag = flag + 1;
        end

    % ��ʱ��ϵͳ2���ܵ�DoS��������ϵͳ1,3,4,5֮���ͨ����·�Ͽ�
    elseif k== DoSTime
        [G,H,J,Gji] = DenialofServiceAttack( A,Ai,branches,B,E,SampingTime);
        Pi = care(A(1:XN,1:XN), B(1:XN,1), Ep*I,1);
    elseif k>DoSTime
        for i =1:SN
            uin(i) = pinv(H(1:XN,1))*Gji(XN*(i-1)+1:XN*i,:)*x;
            ui(i) = uin(i);
        end
    end
    
    uSave(:,k) = u;   %���浱ǰʱ�̵Ŀ�������u
    UinSave(:,flag) = uin;     %���� sigma(v*uir)+uin
    next_x = Dyamic_model(G, H, J, branches, x, u);
    x = next_x;
    xSave(:,k+1) = x;
end

%t=0:SampingTime:EndTime-SampingTime;
plot(xSave(1:5:end,:)');title('state');
figure()
plot(uSave(2,:)');