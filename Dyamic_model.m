function [ dx,Wi ] = Dyamic_model( G, H, J, branches, x, u)
% Dyamic_model �������ϵͳ����ѧģ��
%  ���룺G��H��J, branches, x, u
%  �����k+1ʱ���¸�����ϵͳ��״̬x(k+1)

%% ��ʼ������
XN = 5;       %״̬��xi��ά��


%% �����������Ƶ�ʿ���ϵͳһ��ģ��(������xj(k)����)
Wi = rand(size(x))-0.5;
dx = G*x + H*u + J*Wi;

for i=1:size(branches,2)
    k=branches(i,1);
    l=branches(i,2);
    dx((k-1)*XN+1)= dx((k-1)*XN+1)+branches(i,4)*sin(x((k-1)*XN+5)-x((l-1)*XN+5));
    dx((l-1)*XN+1)= dx((l-1)*XN+1)+branches(i,4)*sin(x((l-1)*XN+5)-x((k-1)*XN+5));
end
end

