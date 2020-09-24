%
%
% 初始化地图： 1——障碍物 -1——智能体初始位置和目标位置    
% input_map = zeros(10,10);
% input_map = zeros(30,30);
% input_map = zeros(50,50);
% input_map = zeros(70,70);
% input_map = zeros(90,90);




is_solutions_marrtsfn = cell(12,10);                                              % 解容器初始化
is_solutions_marrts = cell(12,10);
sol_ismarrts = cell(12,10);
sol_ismarrtsfn = cell(12,10);
parfor repeat = 1 : 1
    for rand_instance = 1 : 1
        agent = 8;
        % 智能体数量，地图初始化
        input_map = zeros(50,50);                                               % 地图初始化        
        agent_count = agent;                                                    % 智能体的数量，初始化     
        [rows,cols] = size(input_map);                                          % 地图的行数和列数
        
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
        
        % 初始化智能体位置和目标
        valid = false;                                                          % 初始化采样点有效程度为否
        A = [];                                                                 % 智能体对象数组初始化为空
        for i = 1 : agent_count
            while valid == false
                rand_position = [ceil(25*rand),ceil(25*rand)];              % 随机选择1个网格作为智能体初始位置格子
                rand_goal = [ceil((rows-1)*rand),ceil((cols-1)*rand)];                  % 随机选择1个网格作为智能体目标位置格子
                if (input_map(rand_position(1),rand_position(2)) ~= 1) && ...
                        (input_map(rand_position(1),rand_position(2)) ~= -1) && ...
                        (input_map(rand_goal(1),rand_goal(2)) ~= 1) && ...
                        (input_map(rand_goal(1),rand_goal(2)) ~= -1)                     % 智能体的初始位置不重合，目标位置不重合，和障碍物不干涉
                    input_map(rand_position(1),rand_position(2)) = -1;                   % 初始位置置-1
                    input_map(rand_goal(1),rand_goal(2)) = -1;                           % 目标位置置-1
                    backup_A = Agent(rand_position,rand_goal,input_map);                % 初始化智能体类,【每个智能体的地图仅包含自己的初始位置和目标位置】
                    A = [A;backup_A];                                                   % 把各个智能体依次存入数组
                    valid = true;                                                       % 有效程度为真
                end
            end
            valid = false;                                                              % 有效程度重置
        end
        position = A(1).position;                                                       % 存储智能体的位置，作为初始状态节点，是行向量
        goal = A(1).goal;                                                               % 存储智能体的目标，作为目标状态节点，是行向量
        if agent_count > 1
            for i = 2 : agent_count
                position = [position,A(i).position];
                goal = [goal,A(i).goal];
            end
        end                        
        % 运行单个rrt算法
        single_solution = [];                                                           % 单个智能体的解初始化为空
        for i = 1 : agent_count                                                         % 对每一个智能体，单独求一次解
            rrts = RRTs(input_map,A(i).position,A(i).goal,1);                           % 初始化RRT算法，分配参数
            rrts = runRRTs(rrts);                                                       % 运行rrt算法
            single_solution = [single_solution;rrts];                                   % 求得的解存储在single_solution中
        end
%         % 运行isMA-RRTsFN算法
        is_marrtsfn = MARRTsFN(input_map,position,goal,agent_count);                         % 初始化MARRTs算法，分配参数
        is_marrtsfn = runRRTfn(is_marrtsfn,single_solution);                                % 运行MARRTs算法
        sol_ismarrtsfn{rand_instance,repeat} = is_marrtsfn;                                          % 结果存入数组
        is_marrtsfn
        celldisp(is_marrtsfn.path)
    end
end

is_solutions_marrts(:,:) = sol_ismarrts(:,:);
is_solutions_marrtsfn(:,:) = sol_ismarrtsfn(:,:);
save('all_repeat_all_marrts.mat','is_solutions_marrts','is_solutions_marrtsfn','-append');% 结果存入文件

