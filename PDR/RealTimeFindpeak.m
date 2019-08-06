close all,
clc
addpath('/home/lab5254/Desktop/PDR')
best=[];
 for pp = 1:length(slope)
     pp
    b = slope(1,2)*0.65;
    a = slope(1,1);
% a=0.0275;%p(1,1);
%  b=0.325;%p(1,2);


yg=[];
PosX = 0; PosY = 0;
valley = false; peak = false;
trajectory = zeros(1,4);
peakdata = zeros(1,4);
steps = 0;
lag         = 5;       % lag for the smoothing
threshold   = 3.5;     % number of st.dev. away from the mean to signal
influence   = 0;

rpy1 = rpy(3202:4482,:);




 ERROR=0;
 k=[];total_d=[];
 
 
for i = 1:length(rpy1)
    
    yg = [yg rpy1(i,1)];
    
    if i > lag + 5
        
        turn_init = mean(rpy1(1:10,2))-0.0758;
        RotMat  = [ cos(turn_init)  -sin(turn_init);...
            sin(turn_init)   cos(turn_init)];
        
        % Peak signal detection
        signals = ThresholdingAlgo(yg,lag,threshold,influence);
        if signals(i) == -1
            if yg(i) < yg(i-1)
                iv = yg(i);
                iv_Location = i;
                valley = true;
                signals(i-1) = 0;
            end
        elseif signals(i) == 1 && valley == true
            if yg(i) > yg(i-1)
                ip = yg(i);
                ip_Location = i;
                signals(i-1) = 0;
            elseif yg(i) < yg(i-1)
                peak = true;
            end
        end
        
        if (valley && peak) == true
            % walking freq (between 0.1sec ~ 2sec(50Hz))
            if (ip_Location - iv_Location) >= 5 && (ip_Location - iv_Location) < 100 && (ip-iv) > 0.2
                steps = steps + 1;
                peakdata = [peakdata;iv_Location,iv,ip_Location,ip];
                [StepLength,deltaH,PosX,PosY] = ...
                    PosUpdata(iv_Location,iv,ip_Location,ip,rpy1,PosX,PosY,a,b,0);
                
                PosRot = [PosX,PosY].*RotMat;
                X =  PosRot(1);
                Y =  PosRot(2);
                
                trajectory = [trajectory;StepLength,deltaH,X,Y];
                %fprintf(['steps = ',num2str(steps),' ,sneding goal...\r\n'])
            end
            valley = false;
            peak = false;
        end
    end
    
end

n=1;
k = datab(n:end,1);

p=trajectory(n+1:end,1);

total_d=sum(k)-sum(p);
ERROR = (abs(k-p));
avgerror = mean(abs(k-p));
MAX_ERROR = max(ERROR(1:end));
MIN_ERROR = min(ERROR(1:end));
stderror=std(abs(k-p));
best =[best;total_d,sum(ERROR),avgerror,MAX_ERROR,MIN_ERROR,stderror];
end


q=figure,hold on,grid minor
ref=0;
dif=abs(best(:,1)-ref);
stem(dif)
idx=find(dif == min(dif));
ymin=abs(best(idx,1));
pi=best(:,1);
a=ones(1,length(slope)+1)';
for i=1:length(a)
    
    a(i,1)=i;
end

plot(idx,ymin,'ro')
text(idx,ymin,['m=' num2str(slope(idx,1)) char(10) 'b=' num2str(slope(idx,2))])

legend('error','best_{parameter}')
% saveas(q,'error','fig')
% saveas(q,'error','jpg')
%%
h=figure;
hold on
plot(yg)
plot(peakdata(2:end,3),yg(peakdata(2:end,3)'),'rv','MarkerFaceColor','r');
plot(peakdata(2:end,1),yg(peakdata(2:end,1)'),'rs','MarkerFaceColor','b');
% 
% % plot(rpy1(:,2))
% legend('pitch','theta_m_a_x','theta_m_i_n'), xlabel('Time[sec]'), ylabel('Pitch[rad]')
% % saveas(h,'findpeak','fig')
% % saveas(h,'findpeak','jpg')
% 
% % saveas(h,'findpeak','fig')
u= figure;
plot(-trajectory(:,4),trajectory(:,3),'o-'),
axis equal,grid minor
legend('trajectory'), xlabel('position Y[m]'), ylabel('position X[m]')
 title({[num2str(a),'*dH+',num2str(b),'=',num2str(sum(trajectory(1:end,1)))];['total_k=',num2str(ktoatal)]});
% 
% deltaH = rad2deg(peakdata(2:end,4)-peakdata(2:end,2));
% % saveas(u,'path','fig')
% % saveas(u,'path','jpg')
% % 
% f=figure,
% subplot(211)
% hold on,grid minor
% n=1;
% k = datab(n:end,1);
% plot(k)
% 
% p=trajectory(n+1:end,1);
% plot(p);
% 
% legend('kinect','PDR')
% 
% 
% ERROR = abs(k-p);
% avgerror = mean((ERROR));
% 
% subplot(212)
% stem(ERROR,'g')
% 
% legend('error'),xlabel('count'), ylabel('distance[m]'),grid minor
% %  title({['SL=',num2str(m),'*dH+',num2str(b),'std=',num2str(std(ERROR))];...
% %      ['total_K=',num2str(sum(k)),', total_P=',num2str(sum(p)),',mean(error)=',num2str(avgerror)]})
% 
% subplot(211)
% MAX_ERROR = max(ERROR(1:end));
% MIN_ERROR = min(ERROR(1:end));
% title({['SL=',num2str(a),'*dH+',num2str(b)];...
%     [num2str(sum(k)),'-',num2str(sum(p)),'=',num2str(sum(k)-sum(p))];...
%     ['MAE=',num2str(avgerror),',MAX=',num2str(MAX_ERROR),',MIN=',num2str(MIN_ERROR)]})
% legend('kinect','PDR','error'), xlabel('count'), ylabel('distance[m]')

%  saveas(f,'7-15-11-30-12(7)-fit7','jpg')
%  saveas(f,'7-15-11-30-12(7)-fit7','fig')