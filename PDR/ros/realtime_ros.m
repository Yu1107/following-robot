clear,close all
rosshutdown
rosinit
clc


filename = datetime('now'); 

lag         = 5;       % lag for the smoothing
threshold   = 3.5;     % number of st.dev. away from the mean to signal
influence   = 0;


rpy = zeros(1,2);
xg = []; yg = [];
ip = []; iv = [];
ip_Location = []; iv_Location = [];
PosX = 0; PosY = 0;
valley = false; peak = false;
trajectory = zeros(1,3);
peakdata = zeros(1,4);
steps = 0;

%% ros
desiredRate = 50;
looptime = 15;
n = desiredRate * looptime;

rate = rosrate(desiredRate);
rate.OverrunAction = 'slip';
% boolsub = rossubscriber('/goal_achieved','std_msgs/Bool');
imusub = rossubscriber('/imu/rpy/complementary_filtered','geometry_msgs/Vector3Stamped');
[pospub,posmsg] = rospublisher('/PDR_position','geometry_msgs/Point');

% node = robotics.ros.Node('/matlab_PDR');
% rate = robotics.ros.Rate(node,desiredRate);
% pospub = robotics.ros.Publisher(node,'/position','geometry_msgs/Point');
% posmsg = rosmessage('geometry_msgs/Point');
% imusub = robotics.ros.Subscriber(node,'/imu/rpy/complementary_filtered','geometry_msgs/Vector3Stamped');
%%

  a=.023721;
b = 0.383;

reset(rate);
fprintf('start recording...\r\n')
for i = 1:n
    imudata = receive(imusub);
    rpy(i,:) = [imudata.Vector.X,imudata.Vector.Z];
    
    yg = [yg imudata.Vector.X];
    
    if i > lag +5
        
        turn_init=mean(rpy(1:10,2));
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
            if (ip_Location - iv_Location) >= 10 && (ip_Location - iv_Location) < 100
                steps = steps + 1
                peakdata = [peakdata;iv_Location,iv,ip_Location,ip];
                [StepLength,PosX,PosY] = PosUpdata(iv_Location,iv,ip_Location,ip,rpy,PosX,PosY,a,b,0);
                trajectory = [trajectory;StepLength,PosX,PosY];
                
                PosRot = [PosX,PosY]*RotMat;
                posmsg.X = PosRot(1);
                posmsg.Y = PosRot(2);
                posmsg.Z = 0;
                send(pospub,posmsg);
                fprintf('sneding goal...\r\n')
            end
            valley = false;
            peak = false;
        end
    end
    waitfor(rate);
end
%sum(trajectory(:,1))

save(char(filename))
%%

% figure('Position',[200 100 1000 500])
% subplot(2,1,1); hold on;plot(xg,yg,'r.');
% title(sprintf(['Draw data points (%.0f max)      [settings: lag = %.0f, '...
%     'threshold = %.2f, influence = %.2f]'],n,lag,threshold,influence));
% plot(xg(lag+1:end),avg(lag+1:end),'LineWidth',1,'Color','cyan');
% plot(xg(lag+1:end),avg(lag+1:end)+threshold*dev(lag+1:end),...
%     'LineWidth',1,'Color','green');
% plot(xg(lag+1:end),avg(lag+1:end)-threshold*dev(lag+1:end),...
%     'LineWidth',1,'Color','green');
%
% subplot(2,1,2);
% hold on; title('Signal output');
% stairs(xg(lag+1:end),signals(lag+1:end),'LineWidth',2,'Color','blue');
% ylim([-2 2]); xlim([0 50]); hold off;

%%
% figure,
% plot(trajectory(:,3),trajectory(:,2),'o-'),grid on
% title(['Total Distance = ',num2str(sum(trajectory(:,1)))])
% ylabel('PositionY'),xlabel('PositionX'),legend('trajectory')


