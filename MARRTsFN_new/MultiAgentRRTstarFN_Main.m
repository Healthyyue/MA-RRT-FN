
% ��������������ͼ��ʼ��
% input_map = zeros(50,50);                                               % ��ͼ��ʼ��
% agent_count = agent;                                                    % ���������������ʼ��
% [rows,cols] = size(input_map);                                          % ��ͼ������������
% 
% input_map(5,15:40)=1;
% input_map(12,10:40)=1;
% input_map(20,10:40)=1;
% input_map(25,10:40)=1;
% input_map(30,10:40)=1;
% input_map(35,10:40)=1;
% input_map(40,10:40)=1;
% input_map(45,10:40)=1;
% input_map(10:45,5)=1;
% input_map(10:45,45)=1;
agent = 3;
map_index = 3;
% ��������������ͼ��ʼ��
choose_map = {zeros(10,10);zeros(30,30);zeros(50,50);zeros(70,70);zeros(90,90)}; % ��ͼѡ������ʼ��
input_map = choose_map{map_index,1};                                    % ��ͼ��ʼ��
agent_count = agent;                                                    % ���������������ʼ��
[rows,cols] = size(input_map);                                          % ��ͼ������������
% ��ʼ���ϰ���λ��
for i = 1 : (rows*cols/10)                                              % ȡ����������1/10��Ϊ�ϰ������
    input_map(ceil((rows-1)*rand),ceil((cols-1)*rand)) = 1;             % ���ѡ��1��������Ϊ�ϰ������
end
% ��ʼ��������λ�ú�Ŀ��
valid = false;                                                          % ��ʼ����������Ч�̶�Ϊ��
A = [];                                                                 % ��������������ʼ��Ϊ��
for i = 1 : agent_count
    while valid == false
        rand_position = [ceil(5*rand),ceil(5*rand)];                             % ���ѡ��1��������Ϊ�������ʼλ�ø���
        rand_goal = [ceil((rows-1)*rand),ceil((cols-1)*rand)];                  % ���ѡ��1��������Ϊ������Ŀ��λ�ø���
        if (input_map(rand_position(1),rand_position(2)) ~= 1) && ...
                (input_map(rand_position(1),rand_position(2)) ~= -1) && ...
                (input_map(rand_goal(1),rand_goal(2)) ~= 1) && ...
                (input_map(rand_goal(1),rand_goal(2)) ~= -1)                     % ������ĳ�ʼλ�ò��غϣ�Ŀ��λ�ò��غϣ����ϰ��ﲻ����
            input_map(rand_position(1),rand_position(2)) = -1;                   % ��ʼλ����-1
            input_map(rand_goal(1),rand_goal(2)) = -1;                           % Ŀ��λ����-1
            backup_A = Agent(rand_position,rand_goal,input_map);                % ��ʼ����������,��ÿ��������ĵ�ͼ�������Լ��ĳ�ʼλ�ú�Ŀ��λ�á�
            A = [A;backup_A];                                                   % �Ѹ������������δ�������
            valid = true;                                                       % ��Ч�̶�Ϊ��
        end
    end
    valid = false;                                                              % ��Ч�̶�����
end
position = A(1).position;                                                       % �洢�������λ�ã���Ϊ��ʼ״̬�ڵ㣬��������
goal = A(1).goal;                                                               % �洢�������Ŀ�꣬��ΪĿ��״̬�ڵ㣬��������
if agent_count > 1
    for i = 2 : agent_count
        position = [position,A(i).position];
        goal = [goal,A(i).goal];
    end
end
% ���е���rrt�㷨
        single_solution = [];                                                           % ����������Ľ��ʼ��Ϊ��
        for i = 1 : agent_count                                                         % ��ÿһ�������壬������һ�ν�
            fprintf('С��%d:', i);
            rrts = RRTs(input_map,A(i).position,A(i).goal,1);                           % ��ʼ��RRT�㷨���������
            rrts = runRRTs(rrts);                                                       % ����rrt�㷨
            rrts
            single_solution = [single_solution;rrts];                                   % ��õĽ�洢��single_solution��
        end
        % ����isMA-RRTs�㷨
        is_marrts = MARRTs(input_map,position,goal,agent_count);                         % ��ʼ��MARRTs�㷨���������
        fprintf('�೵�㷨����~\n');
        is_marrts = runMARRTs(is_marrts,single_solution);                                % ����MARRTs�㷨
        celldisp(is_marrts.path);
        
        A = cell2mat(is_marrts.path);
        for i = 1 : rrt.agent_count
            for j = 1 : size(A,1)
                d_map(A(j,i),A(j,i+1))=4;
            end
        end
        pcolor(0.5:size(rrt.map,2)+0.5,0.5:size(rrt.map,1)+0.5,d_map);
        hold on

