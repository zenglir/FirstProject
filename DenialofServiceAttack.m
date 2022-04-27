function [ G,H,J,Gji ] = DenialofServiceAttack( A,Ai,branches,B,E,SampingTime)
%DENIAL_OF_SERVIE 此处显示有关此函数的摘要
%   此处显示详细说明
num = 2;
% Temp = find(branches(:,1)==num);
% for j =1:size(Temp,1)
%     branches(Temp(j,1),2)=0;
% end
% 
% Temp1 = find(branches(:,2)==num);
% for j =1:size(Temp1,1)
%     branches(Temp1(j,1),1)=0;
% end

for i=1:size(branches,1)
    k=branches(i,1);
    l=branches(i,2);
    if k == num||l== num
        A((k-1)*size(Ai,1)+2,(l-1)*size(Ai,2)+1)=0;
        A((l-1)*size(Ai,1)+2,(k-1)*size(Ai,2)+1)=0;
    end
end
[G,H] = c2d(A,B,SampingTime);
[~,J] = c2d(A,E,SampingTime);

% 从G中将Gji分离出来
Gji = G;
Temp = zeros(size(Ai));
for i=1:14
    Gji((i-1)*size(Ai,1)+1:i*size(Ai,1),(i-1)*size(Ai,2)+1:i*size(Ai,2)) = Temp;
end

end

