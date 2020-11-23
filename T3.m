%/**************************************
%;function: caculate angle of motion
%   inputs:   x_z_all:���ͼ��Ӧ������ֵ��ͼ 
%             R_window���ƶ����ڵİ뾶��С
%
%   outputs: val
%   the value of the resopond angle of the position to centre
%****************************************/

function [val,B_Pos,B_d,Free_Pos]=get_beta2(x_z_all,R_window)

pi =3.14159;
[K1,K2] = size(x_z_all);%ȡ������ĳ���
val_rsc()=0;
val(360/5)=0;

bedal()=0.0;%����Ϊһά������
k=1;
t(360/5)=0;%ƽ������
for i=1:K1
    for j=1:K2
        if (((i-K1/2-0.5)^2+(j-K2/2-0.5)^2)<=(R_window)^2)%��֤��Բ���ƶ����� R_window*R_window
            if((j-K2/2-0.5)>0)%����һ �� ����
                bedal(k)=atan((j-K2/2-0.5)/(i-K1/2-0.5));  %k��Ӧ����ɨ���ȥ��
                  if (bedal(k)<0)
                        bedal(k)=bedal(k)+pi;
                  end  
                  for n=1:360/5
                      if ((5*pi*(n-1)/180<=bedal(k))&&(bedal(k)<5*pi*n/180));%n�Ǽ�������5�� һ����� ���� ���ǶȵĲ�ͬ
                          if (0~=x_z_all(i,j))%���ϰ���
                              val(n) = val(n)+sqrt((j-K2/2-0.5)^2+(i-K1/2-0.5)^2);
                              t(n)=t(n)+1;%�ۼӵĴ���   
                          end
                      end
                  end
                k=k+1;
            else%3 4 ����
                bedal(k)=atan((j-K2/2-0.5)/(i-K1/2-0.5));  %k��Ӧ����ɨ���ȥ��
                  if (bedal(k)>0)
                        bedal(k)=bedal(k)+pi; % 3 
                  else
                        bedal(k)=bedal(k)+2*pi;% 4
                  end    
                  
                 for n=1:360/5
                      if ((5*pi*(n-1)/180<=bedal(k))&&(bedal(k)<5*pi*n/180));%n�Ǽ�������5�� һ����� ���� ���ǶȵĲ�ͬ
                          if (0~=x_z_all(i,j))%���ϰ���
                              val(n) = val(n)+sqrt((j-K2/2-0.5)^2+(i-K1/2-0.5)^2);
                              t(n)=t(n)+1;%�ۼӵĴ���   
                          end
                      end
                 end
               k=k+1;
            end
        end
    end
end
%�Խ��е���
for n=1:360/5
    if 0~=t(n)
            val(n)=val(n)/t(n);%��n��դ�������Ϣ�ľ�ֵ��Ϊ������ʸ��ֵ
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
T=title('դ���Ӧ�Ƕ�');
set(T,'FontSize',14);
X=xlabel('����ɨ��ķ�ʽ ');
set(X,'FontSize',14);
Y=ylabel('�Ƕ� ������ [ rad ]');
set(Y,'FontSize',14);


% test the val
%��������ֱ��ͼ
subplot(212); 
temp = 1:length(val);
bar(temp*5,val(temp));
axis([0 72*5 0 max(val)]);
AX1=gca;
set(AX1,'FontSize',12);
T=title('��������ֱ��ͼ');
set(T,'FontSize',14);
X=xlabel('����Ƕ�(per 5)[��] ');
set(X,'FontSize',14);
Y=ylabel('���� [ cm ]');
set(Y,'FontSize',14);
% detect the barrier
sta_val=val;

%�ٶ������˵ĳߴ�ΪRober_Size=10*10����Ϊ�ϰ���֮����� d_disg_Max=10 ���з���
%
cout_B=1;%initiation
d_disg_Max=10;
cout_Free=0;%
flag_new_B=1;
flag_new_F=0;
for i=1:length(sta_val)-1
    if (0~=sta_val(i))
        flag_new_F=0;%�����־˵�������������������
        if (1==flag_new_B)
            B_Pos(cout_B).angle_begin=i*5;
            flag_new_B=0;%�����־
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
            flag_new_F=1;%˵����һ������������
        else
            Free_Pos(cout_Free).angle_end=i*5;             
        end
    end
end
