%matlab��ʼ��

clc;             %�������ڵ�����
clear all;       %��������ռ�����б�������������MEX�ļ�
close all;       %�ر����е�figure����

[y,Fs] = audioread('001.wav'); sound(y,Fs); % ������Ϊ001�����֣�ע����ļ���Ҫ��matlab�ļ�λ��ͬһ�ļ�����



%���ܲ������趨����
n = 100;   % ����һ��n x n�ķ����޸Ĵ�ֵ�����޸�����ͼƬ�ķ�����
wallpercent = 0.4;  % ��������������ɵ��ϰ���ռ�ܷ������ı��� ����0.5 ��ʾ�ϰ���ռ�ܸ�����50%
Weights=1;       %��̬��������ʽA���㷨�е�h��n��Ȩ��ϵ��
Corner_amend=1;  %ѡ���Ƿ���йսǵ��������ñ�����Ϊ0�򲻽��йս���������Ϊ1����йս�����
Environmental_Set=0; %�����������ѡ���Ƿ���������ϰ�����趨Ϊ0����ʹ����һ�δ����Ļ�����Ϣ�����趨Ϊ1����������������ϰ���
Reset_GS=1;   %�����������ѡ���Ƿ������趨��ʼ�����ֹ�㣬���趨Ϊ1����ʼ�����趨��ʼ�����ֹ�㣬ͬʱ��Ҫ������New_goalposind��New_startposind��ֵ�޸�Ϊ����ѡ�����ʼ�����ֹ�������ֵ����Ϊ0��ر�
New_startposind=4100;   New_goalposind=8200;  %����Reset_GS�趨Ϊ1������Ҫ������New_goalposind��New_startposind��ֵ�޸�Ϊ����ѡ�����ʼ�����ֹ�������ֵ��Ҫȷ��������������㴦û���ϰ���


%�����Լ��ϰ���Ĵ���
if(Environmental_Set)
[field, startposind, goalposind, costchart, fieldpointers] =initializeField(n,wallpercent); %������ɰ����ϰ����ʼ�㣬��ֹ�����Ϣ�ľ���
save('Environmental','field','startposind','goalposind','costchart','fieldpointers' )
else
load('Environmental')
end

%�����趨��ʼ�����ֹ��
if(Reset_GS)
[field, startposind, goalposind, costchart, fieldpointers] = Reset_G_S(field, startposind, goalposind, costchart, fieldpointers,New_startposind,New_goalposind);
end

% ·���滮���õ���һЩ����ĳ�ʼ��
setOpen = [startposind]; setOpenCosts = [0]; setOpenHeuristics = [Inf];
setClosed = []; setClosedCosts = [];
movementdirections = {'R','L','D','U'};  %�ƶ�����

%��ʼ��һЩ����·����������Ҫ�õ��ı���
Parent_node=0; %Parent_node��ʼ��������ᱨ��
Expected_note=0;%Expected_note��ʼ��������ᱨ��
untext_ii=0;  %δ����������µ�ii
amend_count=0;% ��¼�����Ĵ���


% �����������������ɻ������ϰ����㣬�յ�
axishandle = createFigure(field,costchart,startposind,goalposind);    %��������ɵķ����ϰ������������ͼ��

%%

% ���whileѭ���Ǳ�����ĺ��ģ�����ѭ�����е�����Ѱ����ֹ��
while ~max(ismember(setOpen,goalposind)) && ~isempty(setOpen)
   
    [temp, ii] = min(setOpenCosts +Weights*setOpenHeuristics);     %Ѱ����չ��������Сֵ 
    
    if ((setOpen(ii)~=startposind) && (Corner_amend==1))
    [new_ii,amend_count_1]=Path_optimization(temp, ii,fieldpointers,setOpen,setOpenCosts,startposind,Weights,setOpenHeuristics,Parent_node,Expected_note,untext_ii,amend_count); %����·�����������ڱ�֤�����Ӿ���Ļ����ϣ�ʹ�����ת��Ĵ���
    ii=new_ii;
    amend_count=amend_count_1;
    end
    
    %������������þ��ǰ�����ĵ���Ϊ���ڵ㣬Ȼ�������չ�ҵ��ӽڵ㣬�����ҵ��ӽڵ�Ĵ��ۣ����Ұ��ӽڵ�����յ�Ĵ����ҵ�
    [costs,heuristics,posinds] = findFValue(setOpen(ii),setOpenCosts(ii), field,goalposind,'euclidean');
 
  setClosed = [setClosed; setOpen(ii)];     % ���ҳ�������չ�����ĵ��д�����С���Ǹ��㴮������setClosed �� 
  setClosedCosts = [setClosedCosts; setOpenCosts(ii)];    % ����չ�����ĵ��д�����С���Ǹ���Ĵ��۴�������setClosedCosts ��
  
  % ��setOpen��ɾ���ղŷŵ�����setClosed�е��Ǹ���
  %��������λ�ھ�����ڲ�
  if (ii > 1 && ii < length(setOpen))
    setOpen = [setOpen(1:ii-1); setOpen(ii+1:end)];
    setOpenCosts = [setOpenCosts(1:ii-1); setOpenCosts(ii+1:end)];
    setOpenHeuristics = [setOpenHeuristics(1:ii-1); setOpenHeuristics(ii+1:end)];
    
  %��������λ�ھ����һ��
  elseif (ii == 1)
    setOpen = setOpen(2:end);
    setOpenCosts = setOpenCosts(2:end);
    setOpenHeuristics = setOpenHeuristics(2:end);
    
  %��������λ�ھ�������һ��
  else
    setOpen = setOpen(1:end-1);
    setOpenCosts = setOpenCosts(1:end-1);
    setOpenHeuristics = setOpenHeuristics(1:end-1);
  end
  
 %%  
  % ����չ�����ĵ��з���Ҫ��ĵ�ŵ�setOpen �����У���Ϊ��ѡ��
  for jj=1:length(posinds)
  
    if ~isinf(costs(jj))   % �жϸõ㣨���񣩴�û���ϰ���
        
      % �ж�һ�¸õ��Ƿ� �Ѿ�������setOpen �������setClosed ������
      % �������Ҫ�������չ��Ȳ���setOpen ����Ҳ����setClosed ������
      if ~max([setClosed; setOpen] == posinds(jj))
        fieldpointers(posinds(jj)) = movementdirections(jj);
        costchart(posinds(jj)) = costs(jj);
        setOpen = [setOpen; posinds(jj)];
        setOpenCosts = [setOpenCosts; costs(jj)];
        setOpenHeuristics = [setOpenHeuristics; heuristics(jj)];
        
      % �������Ҫ�������չ���Ѿ���setOpen ������
      elseif max(setOpen == posinds(jj))
        I = find(setOpen == posinds(jj));
        % ���ͨ��Ŀǰ�ķ����ҵ�������㣬��֮ǰ�ķ����ã�����С���͸��������
        if setOpenCosts(I) > costs(jj)
          costchart(setOpen(I)) = costs(jj);
          setOpenCosts(I) = costs(jj);
          setOpenHeuristics(I) = heuristics(jj);
          fieldpointers(setOpen(I)) = movementdirections(jj);
        end
        
        % �������Ҫ�������չ���Ѿ���setClosed ������
      else
        I = find(setClosed == posinds(jj));
        % ���ͨ��Ŀǰ�ķ����ҵ�������㣬��֮ǰ�ķ����ã�����С���͸��������
        if setClosedCosts(I) > costs(jj)
          costchart(setClosed(I)) = costs(jj);
          setClosedCosts(I) = costs(jj);
          fieldpointers(setClosed(I)) = movementdirections(jj);
        end
      end
    end
  end
  
 %% 
  
  if isempty(setOpen) break; end
  set(axishandle,'CData',[costchart costchart(:,end); costchart(end,:) costchart(end,end)]);
  set(gca,'CLim',[0 1.1*max(costchart(find(costchart < Inf)))]);
  drawnow; 
end

%%

%����findWayBack��������·�����ݣ������Ƴ�·������
if max(ismember(setOpen,goalposind))
  disp('Solution found!');
  p = findWayBack(goalposind,fieldpointers); % ����findWayBack��������·�����ݣ������ݽ�����ھ���P��
  plot(p(:,2)+0.5,p(:,1)+0.5,'Color',0.2*ones(3,1),'LineWidth',4);  %�� plot��������·������
  drawnow;
  drawnow;
  clear sound
  [y,Fs] = audioread('002.wav'); sound(y,Fs); % ������Ϊ000�����֣�ע����ļ���Ҫ��matlab�ļ�λ��ͬһ�ļ�����
elseif isempty(setOpen)
  disp('No Solution!'); 
  clear sound
  [y,Fs] = audioread('000.wav'); 
  sound(y,Fs);
end

%% 

%findWayBack������������·�����ݣ���������������������ֹ��goalposind�;���fieldpointers�����������P
function p = findWayBack(goalposind,fieldpointers)

    n = length(fieldpointers);  % ��ȡ�����ĳ���Ҳ����n
    posind = goalposind;
    [py,px] = ind2sub([n,n],posind); % ������ֵposindת��Ϊ����ֵ [py,px]
    p = [py px];
    
    %����whileѭ�����л��ݣ������ǻ��ݵ���ʼ���ʱ��ֹͣ��Ҳ�����ھ���fieldpointers���ҵ�Sʱֹͣ
    while ~strcmp(fieldpointers{posind},'S')
      switch fieldpointers{posind}
          
        case 'L' % ��L�� ��ʾ��ǰ�ĵ�������ߵĵ���չ������
          px = px - 1;
        case 'R' % ��R�� ��ʾ��ǰ�ĵ������ұߵĵ���չ������
          px = px + 1;
        case 'U' % ��U�� ��ʾ��ǰ�ĵ���������ĵ���չ������
          py = py - 1;
        case 'D' % ��D�� ��ʾ��ǰ�ĵ������±ߵĵ���չ������
          py = py + 1;
      end
      p = [p; py px];
      posind = sub2ind([n n],py,px);% ������ֵת��Ϊ����ֵ
    end
end

%% 
%������������þ��ǰ�����ĵ���Ϊ���ڵ㣬Ȼ�������չ�ҵ��ӽڵ㣬�����ҵ��ӽڵ�Ĵ��ۣ����Ұ��ӽڵ�����յ�Ĵ����ҵ���
%�������������costs��ʾ��չ���ӽڵ㵽��ʼ��Ĵ��ۣ�heuristics��ʾ��չ�����ĵ㵽��ֹ��ľ����Լ�Ƕ��٣�posinds��ʾ��չ�������ӽڵ�
function [cost,heuristic,posinds] = findFValue(posind,costsofar,field,goalind,heuristicmethod)
    n = length(field);  % ��ȡ����ĳ���
    [currentpos(1) currentpos(2)] = ind2sub([n n],posind);   %��Ҫ������չ�ĵ㣨Ҳ���Ǹ��ڵ㣩������ֵ��չ������ֵ
    [goalpos(1) goalpos(2)] = ind2sub([n n],goalind);        %����ֹ�������ֵ��չ������ֵ
    cost = Inf*ones(4,1); heuristic = Inf*ones(4,1); pos = ones(4,2); %������cost��heuristic��ʼ��Ϊ4x1�������ֵ�ľ���pos��ʼ��Ϊ4x2��ֵΪ1�ľ���
    
    % ��չ����һ
    newx = currentpos(2) - 1; newy = currentpos(1);
    if newx > 0
      pos(1,:) = [newy newx];
      switch lower(heuristicmethod)
        case 'euclidean'
          heuristic(1) = 10*abs(goalpos(2)-newx) + 10*abs(goalpos(1)-newy);
        case 'taxicab'
          heuristic(1) = 10*abs(goalpos(2)-newx) +10*abs(goalpos(1)-newy);
      end
      cost(1) = costsofar + field(newy,newx);
    end

    % ��չ�����
    newx = currentpos(2) + 1; newy = currentpos(1);
    if newx <= n
      pos(2,:) = [newy newx];
      switch lower(heuristicmethod)
        case 'euclidean'
          heuristic(2) = 10*abs(goalpos(2)-newx) +10*abs(goalpos(1)-newy);
        case 'taxicab'
          heuristic(2) = 10*abs(goalpos(2)-newx) + 10*abs(goalpos(1)-newy);
      end
      cost(2) = costsofar + field(newy,newx);
    end

    % ��չ������
    newx = currentpos(2); newy = currentpos(1)-1;
    if newy > 0
      pos(3,:) = [newy newx];
      switch lower(heuristicmethod)
        case 'euclidean'
          heuristic(3) = 10*abs(goalpos(2)-newx) + 10*abs(goalpos(1)-newy);
        case 'taxicab'
          heuristic(3) = 10*abs(goalpos(2)-newx) + 10*abs(goalpos(1)-newy);
      end
      cost(3) = costsofar + field(newy,newx);
    end

    % ��չ������
    newx = currentpos(2); newy = currentpos(1)+1;
    if newy <= n
      pos(4,:) = [newy newx];
      switch lower(heuristicmethod)
        case 'euclidean'
          heuristic(4) = 10*abs(goalpos(2)-newx) + 10*abs(goalpos(1)-newy);
        case 'taxicab'
          heuristic(4) = 10*abs(goalpos(2)-newx) + 10*abs(goalpos(1)-newy);
      end
      cost(4) = costsofar + field(newy,newx);
    end
     posinds = sub2ind([n n],pos(:,1),pos(:,2)); % ����չ�������ӽڵ������ֵת��Ϊ����ֵ
end

%% 
%�����������þ���������ɻ������ϰ����ʼ�㣬��ֹ���
function [field, startposind, goalposind, costchart, fieldpointers] = initializeField(n,wallpercent)
    field = 10*ones(n,n);%���������������ľ���Ϊ10
    field(ind2sub([n n],ceil(n^2.*rand(n*n*wallpercent,1)))) = Inf;%����ȡ��
    % ���������ʼ�����ֹ��
    startposind = sub2ind([n,n],ceil(n.*rand),ceil(n.*rand));  %���������ʼ�������ֵ
    goalposind = sub2ind([n,n],ceil(n.*rand),ceil(n.*rand));   %���������ֹ�������ֵ
    field(startposind) = 0; field(goalposind) = 0;  %�Ѿ�������ʼ�����ֹ�㴦��ֵ��Ϊ0
    
    costchart = NaN*ones(n,n);%����һ��nxn�ľ���costchart��ÿ��Ԫ�ض���ΪNaN�����Ǿ����ʼNaN��Ч����
    costchart(startposind) = 0;%�ھ���costchart�н���ʼ��λ�ô���ֵ��Ϊ0
    
    % ����Ԫ������
    fieldpointers = cell(n,n);%����Ԫ������n*n
    fieldpointers(:)= {'1'};
    fieldpointers{startposind} = 'S'; fieldpointers{goalposind} = 'G'; %��Ԫ���������ʼ���λ�ô���Ϊ 'S'����ֹ�㴦��Ϊ'G'
    fieldpointers(field==inf)={'0'};
    
   
end
% end of this function

%%
%����������ɵĻ������������л����Ļ���
function axishandle = createFigure(field,costchart,startposind,goalposind)

      % ���if..else�ṹ���������ж����û�д򿪵�figureͼ������������ô���һ��figureͼ
      if isempty(gcbf)                                       %gcbf�ǵ�ǰ����ͼ��ľ����isempty(gcbf)����gcbfΪ�յĻ������ص�ֵ��1������gcbfΪ�ǿյĻ������ص�ֵ��0
      figure('Position',[560 70 700 700], 'MenuBar','none');  %�Դ�����figureͼ��������ã������������Ļ���ľ���Ϊ450��������Ļ�·��ľ���Ϊ50�����ȺͿ�ȶ�Ϊ700�����ҹر�ͼ��Ĳ˵���
      axes('position', [0.01 0.01 0.99 0.99]);               %�����������λ�ã����½ǵ�������Ϊ0.01,0.01   ���Ͻǵ�������Ϊ0.99 0.99  ��������Ϊfigureͼ�����½�����Ϊ0 0   �����Ͻ�����Ϊ1 1 ��
      else
      gcf; cla;   %gcf ���ص�ǰ Figure ����ľ��ֵ��Ȼ������cla����������
      end
      
      n = length(field);  %��ȡ����ĳ��ȣ�����ֵ������n
      field(field < Inf) = 0; %��fieid�����е��������Ҳ����û���ϰ����λ�ô�����Ϊ0
      pcolor(1:n+1,1:n+1,[field field(:,end); field(end,:) field(end,end)]);%�����һ���ظ��ģ���n X n��Ϊ n+1 X n+1 ��
 
      cmap = flipud(colormap('jet'));  %���ɵ�cmap��һ��256X3�ľ���ÿһ�е�3��ֵ��Ϊ0-1֮�������ֱ������ɫ��ɵ�rgbֵ
      cmap(1,:) = zeros(3,1); cmap(end,:) = ones(3,1); %������cmap�ĵ�һ����Ϊ0 �����һ����Ϊ1
      colormap(flipud(cmap)); %������ɫ�ĵ�ת 
      hold on;
      axishandle = pcolor([1:n+1],[1:n+1],[costchart costchart(:,end); costchart(end,:) costchart(end,end)]);  %������costchart������չ����ֵ��ɫ�󸳸�axishandle
      [goalposy,goalposx] = ind2sub([n,n],goalposind);
      [startposy,startposx] = ind2sub([n,n],startposind);
       plot(goalposx+0.5,goalposy+0.5,'ys','MarkerSize',10,'LineWidth',6);
       plot(startposx+0.5,startposy+0.5,'go','MarkerSize',10,'LineWidth',6);
       %uicontrol('Style','pushbutton','String','RE-DO', 'FontSize',12, 'Position', [1 1 60 40], 'Callback','astardemo');
end
%%
function [new_ii,amend_count_1] = Path_optimization(temp, ii,fieldpointers,setOpen,setOpenCosts,startposind,Weights,setOpenHeuristics,Parent_node,Expected_note,untext_ii,amend_count)
   n = length(fieldpointers);  %��ȡ����ĳ��ȣ�����ֵ������n
   
 %��ȡ�丸�ڵ������ֵ
  switch fieldpointers {setOpen(ii)}
        case 'L' % ��L�� ��ʾ��ǰ�ĵ�������ߵĵ���չ������
          Parent_node = setOpen(ii) - n;
        case 'R' % ��R�� ��ʾ��ǰ�ĵ������ұߵĵ���չ������
           Parent_node = setOpen(ii) + n;
        case 'U' % ��U�� ��ʾ��ǰ�ĵ���������ĵ���չ������
           Parent_node = setOpen(ii) -1;
        case 'D' % ��D�� ��ʾ��ǰ�ĵ������±ߵĵ���չ������
           Parent_node = setOpen(ii) + 1;          
  end
  
  if Parent_node==startposind  %��������ĸ��ڵ�����ʼ��Ļ�����������
     new_ii=ii;
     amend_count_1=amend_count;
  else 
  
       %��ȡ������һ��Ҫ�ߵĵ������ֵ
     switch fieldpointers{Parent_node}
          
        case 'L' % ��L�� ��ʾ��ǰ�ĵ�������ߵĵ���չ������,��ֱ�ߵĻ�����������Ҫ�ߵ���һ����Ϊ�˵��ұߵĵ�
          Expected_note = Parent_node + n;
        case 'R' % ��R�� ��ʾ��ǰ�ĵ������ұߵĵ���չ�����ģ���ֱ�ߵĻ�����������Ҫ�ߵ���һ����Ϊ�˵���ߵĵ�
           Expected_note =  Parent_node - n;
        case 'U' % ��U�� ��ʾ��ǰ�ĵ���������ĵ���չ�����ģ���ֱ�ߵĻ�����������Ҫ�ߵ���һ����Ϊ�˵�����ĵ�
           Expected_note = Parent_node +1;
        case 'D' % ��D�� ��ʾ��ǰ�ĵ������±ߵĵ���չ�����ģ���ֱ�ߵĻ�����������Ҫ�ߵ���һ����Ϊ�˵�����ĵ�
           Expected_note = Parent_node - 1;
     end
   
     if ((Expected_note<=0)||(Expected_note>n*n))   %������������ĵ㲻�ڴ�ѡ�����setOpen�У����߳����߽磬��������
          new_ii=ii;
          amend_count_1=amend_count;
     else
           
           %�����µ�Ҫ������չ�ĵ���setOPen�е�����ֵ 
         if fieldpointers{setOpen(ii)}==fieldpointers{Parent_node}   %�������֮ǰҪ�ߵĵ�������������Ĺ���ֱ�ߵĵ㣬��������
                new_ii=ii;
                amend_count_1=amend_count;
         elseif  find(setOpen == Expected_note)  %�����������Ҫ�ߵĵ��ڴ�ѡ�����setOpen��
             
                 untext_ii=find(setOpen == Expected_note);
                 now_cost=setOpenCosts(untext_ii) +Weights*setOpenHeuristics(untext_ii);   %����������Ҫ���ѵĴ���
                 
                if  temp==now_cost       %������������ĵ�Ҫ���ѵĴ��۵�������֮ǰҪ�ߵĵ㻨�ѵĴ��ۣ��ͽ�����������Ϊ֮ǰҪ�ߵĵ㣬�Ǵ�ѡ�����setOPen�д�����С��һ����֮һ�����������ĵ�Ĵ��۲�����С�ڸõ㣩
                     new_ii=untext_ii;   %���µ�setOPen���������ֵ��ֵ��new_ii���
                     amend_count=amend_count+1;
                     amend_count_1=amend_count; %amend_count_1�м�¼�����ǽ��������Ĵ�����Ϊ�˲鿴��������Ƿ��з�������
                else
                     new_ii=ii;          %������������ĵ�Ҫ���ѵĴ��۴�������֮ǰҪ�ߵĵ㻨�ѵĴ��ۣ�������������A���㷨Ҫ��֤������չ�ĵ��Ǵ�ѡ���д�����С�ģ���Ҳ�ǵ���Զ����ֹ�����һ��ս��޷��õ�������ԭ��
                     amend_count_1=amend_count;
                end
         else
                 new_ii=ii;    %������������ĵ㲻�ڴ�ѡ�����setOpen�У�Ҳ������������ϰ�����߳����߽��ˣ�������������
                 amend_count_1=amend_count;
         end
     end   
  end     
     
end



function [field, startposind, goalposind, costchart, fieldpointers] = Reset_G_S(field, startposind, goalposind, costchart, fieldpointers,New_startposind,New_goalposind)
    
   %��initializeField������������ʱ����field������û���ϰ���ĵ㴦�趨Ϊ10�����ϰ���ĵ㴦�趨Ϊinf����ʼ�����ֹ����Ϊ0
   %������Ҫ���µ���ʼ�����ֹ�㴦��Ϊ0��ԭ����ʼ�����ֹ�㴦��Ϊ10
    field(startposind) = 10; field(goalposind) = 10;  
    field(New_startposind) = 0; field(New_goalposind) = 0;  
   
    %��initializeField������������ʱ����costchart�����е���ʼ���趨Ϊ0���������趨ΪNAN
    %������Ҫ���µ���ʼ���趨Ϊ0��ԭ������ʼ���趨Ϊ1
    costchart(startposind) = NaN;
    costchart(New_startposind) =0;
    
    % ��initializeField������������ʱ����fieldpointersԪ����������ʼ���λ�ô���Ϊ'S'����ֹ�㴦��Ϊ'G'�����ϰ���ĵ���Ϊ'0'��û���ϰ���ĵ���Ϊ'1'
    % ������Ҫ���µ���ʼ�㴦��Ϊ'S'���µ���ֹ�㴦��Ϊ'G'��ԭ������ʼ�����ֹ�㴦��Ϊ'1'
    fieldpointers{startposind} = '1'; fieldpointers{goalposind} = '1'; 
    fieldpointers{New_startposind} = 'S'; fieldpointers{New_goalposind} = 'G'; 
  
    %������ǽ��µ���ʼ����µ���ֹ�������ֵ�ֱ�ֵ��startposind��goalposind�������
    startposind=New_startposind;
    goalposind=New_goalposind;
end
