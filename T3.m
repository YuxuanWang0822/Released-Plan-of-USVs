%/**************************************
%;function: caculate angle of motion
%   inputs:   x_z_all:深度图对应的坐标值地图 
%             R_window：移动窗口的半径大小
%
%   outputs: val
%   the value of the resopond angle of the position to centre
%****************************************/

function [val,B_Pos,B_d,Free_Pos]=get_beta2(x_z_all,R_window)

pi =3.14159;
[K1,K2] = size(x_z_all);%取出数组的长度
val_rsc()=0;
val(360/5)=0;

bedal()=0.0;%保存为一维的数组
k=1;
t(360/5)=0;%平均次数
for i=1:K1
    for j=1:K2
        if (((i-K1/2-0.5)^2+(j-K2/2-0.5)^2)<=(R_window)^2)%保证是圆形移动窗口 R_window*R_window
            if((j-K2/2-0.5)>0)%属于一 二 象限
                bedal(k)=atan((j-K2/2-0.5)/(i-K1/2-0.5));  %k对应是列扫描过去的
                  if (bedal(k)<0)
                        bedal(k)=bedal(k)+pi;
                  end  
                  for n=1:360/5
                      if ((5*pi*(n-1)/180<=bedal(k))&&(bedal(k)<5*pi*n/180));%n是极坐标中5度 一间隔的 个数 即角度的不同
                          if (0~=x_z_all(i,j))%是障碍物
                              val(n) = val(n)+sqrt((j-K2/2-0.5)^2+(i-K1/2-0.5)^2);
                              t(n)=t(n)+1;%累加的次数   
                          end
                      end
                  end
                k=k+1;
            else%3 4 象限
                bedal(k)=atan((j-K2/2-0.5)/(i-K1/2-0.5));  %k对应是列扫描过去的
                  if (bedal(k)>0)
                        bedal(k)=bedal(k)+pi; % 3 
                  else
                        bedal(k)=bedal(k)+2*pi;% 4
                  end    
                  
                 for n=1:360/5
                      if ((5*pi*(n-1)/180<=bedal(k))&&(bedal(k)<5*pi*n/180));%n是极坐标中5度 一间隔的 个数 即角度的不同
                          if (0~=x_z_all(i,j))%是障碍物
                              val(n) = val(n)+sqrt((j-K2/2-0.5)^2+(i-K1/2-0.5)^2);
                              t(n)=t(n)+1;%累加的次数   
                          end
                      end
                 end
               k=k+1;
            end
        end
    end
end
%对进行调整
for n=1:360/5
    if 0~=t(n)
            val(n)=val(n)/t(n);%把n个栅格距离信息的均值作为扇区的矢量值
    else
            val(n)=val(n);
    end
end

%test the bedal
F=figure(2);
subplot(211); 
temp = 1:1:length(bedal);
PT=bar(temp,bedal(temp));
%set(PT,'LineWidth',[2]);
%axis([-Tp 2*Tp -1.2*A 1.2*A]);
AX=gca;
set(AX,'FontSize',12);
T=title('栅格对应角度');
set(T,'FontSize',14);
X=xlabel('按列扫描的方式 ');
set(X,'FontSize',14);
Y=ylabel('角度 弧度制 [ rad ]');
set(Y,'FontSize',14);


% test the val
%向量距离直方图
subplot(212); 
temp = 1:length(val);
bar(temp*5,val(temp));
axis([0 72*5 0 max(val)]);
AX1=gca;
set(AX1,'FontSize',12);
T=title('距离向量直方图');
set(T,'FontSize',14);
X=xlabel('方向角度(per 5)[。] ');
set(X,'FontSize',14);
Y=ylabel('距离 [ cm ]');
set(Y,'FontSize',14);
% detect the barrier
sta_val=val;

%假定机器人的尺寸为Rober_Size=10*10，认为障碍物之间相差 d_disg_Max=10 进行分类
%
cout_B=1;%initiation
d_disg_Max=10;
cout_Free=0;%
flag_new_B=1;
flag_new_F=0;
for i=1:length(sta_val)-1
    if (0~=sta_val(i))
        flag_new_F=0;%清除标志说明自由行走区间结束了
        if (1==flag_new_B)
            B_Pos(cout_B).angle_begin=i*5;
            flag_new_B=0;%清除标志
        end
        if (abs(sta_val(i+1)-sta_val(i))>d_disg_Max)
            cout_B=cout_B+1;
            flag_new_B=1;
        else
            B_Pos(cout_B).angle_end=i*5;
            B_d(cout_B)=min(sta_val(i+1),sta_val(i));
        end
    else
        if (flag_new_F==0)
            cout_Free=cout_Free+1;
            Free_Pos(cout_Free).angle_begin=i*5;
            flag_new_F=1;%说明有一个可行走区域
        else
            Free_Pos(cout_Free).angle_end=i*5;             
        end
    end
end
