%
%
% ��ʼ����ͼ�� 1�����ϰ��� -1�����������ʼλ�ú�Ŀ��λ��    
% input_map = zeros(10,10);
% input_map = zeros(30,30);
% input_map = zeros(50,50);
% input_map = zeros(70,70);
% input_map = zeros(90,90);




is_solutions_marrtsfn = cell(12,10);                                              % ��������ʼ��
is_solutions_marrts = cell(12,10);
sol_ismarrts = cell(12,10);
sol_ismarrtsfn = cell(12,10);
parfor repeat = 1 : 1
    for rand_instance = 1 : 1
        agent = 8;
        % ��������������ͼ��ʼ��
        input_map = zeros(50,50);                                               % ��ͼ��ʼ��        
        agent_count = agent;                                                    % ���������������ʼ��     
        [rows,cols] = size(input_map);                                          % ��ͼ������������
        
        input_map(5,15:40)=1;
        input_map(12,10:40)=1;
        input_map(20,10:40)=1;
        input_map(25,10:40)=1;
        input_map(30,10:40)=1;
        input_map(35,10:40)=1;
        input_map(40,10:40)=1;
        input_map(45,10:40)=1;
        input_map(10:45,5)=1;
        input_map(10:45,45)=1;
        
        % ��ʼ��������λ�ú�Ŀ��
        valid = false;                                                          % ��ʼ����������Ч�̶�Ϊ��
        A = [];                                                                 % ��������������ʼ��Ϊ��
        for i = 1 : agent_count
            while valid == false
                rand_position = [ceil(25*rand),ceil(25*rand)];              % ���ѡ��1��������Ϊ�������ʼλ�ø���
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
            rrts = RRTs(input_map,A(i).position,A(i).goal,1);                           % ��ʼ��RRT�㷨���������
            rrts = runRRTs(rrts);                                                       % ����rrt�㷨
            single_solution = [single_solution;rrts];                                   % ��õĽ�洢��single_solution��
        end
%         % ����isMA-RRTsFN�㷨
        is_marrtsfn = MARRTsFN(input_map,position,goal,agent_count);                         % ��ʼ��MARRTs�㷨���������
        is_marrtsfn = runRRTfn(is_marrtsfn,single_solution);                                % ����MARRTs�㷨
        sol_ismarrtsfn{rand_instance,repeat} = is_marrtsfn;                                          % �����������
        is_marrtsfn
        celldisp(is_marrtsfn.path)
    end
end

is_solutions_marrts(:,:) = sol_ismarrts(:,:);
is_solutions_marrtsfn(:,:) = sol_ismarrtsfn(:,:);
save('all_repeat_all_marrts.mat','is_solutions_marrts','is_solutions_marrtsfn','-append');% ��������ļ�

