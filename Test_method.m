%% 系统参数初始化 xi = [ fi ?Pmi ?Pvi ?Ptie-i ∫ACEi ]'
T=0.5;D=0.26;M=0.2;Td=5;Te=0.2;
Th=4.5;Tg=0.2;R=0.5;K=0.1;ag=0.5;
ae=0.25;ah=0.25;Bi=1;

% 子系统参数矩阵（向量）
A=[-D/M 1/M 0  0 0;
    0   -1/Td 1/Td 0 0;
    1/(Tg*R) 0 -1/Tg K/Tg 0;
    -Bi 0 0 0 0;
    1 0 0 0 0];
B=[ 2/M;0;0;0; 0];

Q = 0.6*eye(5,5);
R = 1;
[P,l,g] = care(A, B, Q, R);
TEST = A'*P + P*A - P*B*R*B'*P + Q;