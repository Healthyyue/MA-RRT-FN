a = ones(50);
% a(3,3:7)=0;
% a(3:10,7)=0;
% a(10,3:7)=0;
% a(17,13:17)=0;
% a(10:17,13)=0;
% a(10,13:17)=0;
% a(14,15)=0;


a(5,15:40)=0;
a(12,10:40)=0;
a(20,10:40)=0;
a(25,10:40)=0;
a(30,10:40)=0;
a(35,10:40)=0;
a(40,10:40)=0;
a(45,10:40)=0;
a(10:45,5)=0;
a(10:45,45)=0;

 
b = a;
%disp(a(end,end));
b(end+1,end+1) = 0;
%disp(b);

colormap([0 0 0;1 1 1]);  % ������ɫ
%disp(size(a));
pcolor(0.5:size(a,2)+0.5,0.5:size(a,1)+0.5,b); % ����դ����ɫ
set(gca,'XTick',1:size(a,1),'YTick',1:size(a,2));  % ��������
axis image xy;  % ��ÿ��������ʹ����ͬ�����ݵ�λ������һ��