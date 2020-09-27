
% 智能体数量，地图初始化
% input_map = zeros(50,50);                                               % 地图初始化
% agent_count = agent;                                                    % 智能体的数量，初始化
% [rows,cols] = size(input_map);                                          % 地图的行数和列数
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
% 智能体数量，地图初始化
choose_map = {zeros(10,10);zeros(30,30);zeros(50,50);zeros(70,70);zeros(90,90)}; % 地图选择器初始化
input_map = choose_map{map_index,1};                                    % 地图初始化
agent_count = agent;                                                    % 智能体的数量，初始化
[rows,cols] = size(input_map);                                          % 地图的行数和列数
% 初始化障碍物位置
for i = 1 : (rows*cols/10)                                              % 取网格数量的1/10作为障碍物格子
    input_map(ceil((rows-1)*rand),ceil((cols-1)*rand)) = 1;             % 随机选择1个网格作为障碍物格子
end
% 初始化智能体位置和目标
valid = false;                                                          % 初始化采样点有效程度为否
A = [];                                                                 % 智能体对象数组初始化为空
for i = 1 : agent_count
    while valid == false
        rand_position = [ceil(5*rand),ceil(5*rand)];                             % 随机选择1个网格作为智能体初始位置格子
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
            fprintf('小车%d:', i);
            rrts = RRTs(input_map,A(i).position,A(i).goal,1);                           % 初始化RRT算法，分配参数
            rrts = runRRTs(rrts);                                                       % 运行rrt算法
            rrts
            single_solution = [single_solution;rrts];                                   % 求得的解存储在single_solution中
        end
        % 运行isMA-RRTs算法
        is_marrts = MARRTs(input_map,position,goal,agent_count);                         % 初始化MARRTs算法，分配参数
        fprintf('多车算法启动~\n');
        is_marrts = runMARRTs(is_marrts,single_solution);                                % 运行MARRTs算法
        celldisp(is_marrts.path);
        
        A = cell2mat(is_marrts.path);
        for i = 1 : rrt.agent_count
            for j = 1 : size(A,1)
                d_map(A(j,i),A(j,i+1))=4;
            end
        end
        pcolor(0.5:size(rrt.map,2)+0.5,0.5:size(rrt.map,1)+0.5,d_map);
        hold on

