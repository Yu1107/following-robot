clear,close all;
rosshutdown;
rosinit;
clc;

 filename = datetime('now');

lag         = 5;       % lag for the smoothing
threshold   = 3.5;     % number of st.dev. away from the mean to signal
influence   = 0;

rpy = zeros(1,2);
yg = [];
ip = []; iv = [];
ip_Location = []; iv_Location = [];
PosX = 0; PosY = 0;
valley = false; peak = false;
trajectory = zeros(1,4);
peakdata = zeros(1,4);
steps = 0;

%% ros
desiredRate = 50;
looptime = 20;
n = desiredRate * looptime;
rate = rosrate(desiredRate);
rate.OverrunAction = 'slip';

%imu topic
imusub = rossubscriber('/imu/rpy/complementary_filtered','geometry_msgs/Vector3Stamped');
[pospub,posmsg] = rospublisher('/PDR_position','geometry_msgs/Point');
goalsub = rossubscriber('/wp','geometry_msgs/Point');

%kinect topic
% startsub = rossubscriber('/short_kinect','std_msgs/Bool');
boolsub = rossubscriber('/initial_position','std_msgs/Bool');
asub = rossubscriber('/parameter','geometry_msgs/Point');

%% main
fprintf('start recording...\r\n')
while 1
    %      startdata = receive(startsub);
    booldata = receive(boolsub);
    
    if booldata.Data == 1
        %         if startdata.Data == 1
        fprintf('Get the initial point\r\n')
        a = asub.LatestMessage.X;
        b = asub.LatestMessage.Y*0.65;
        %
        %get kinect point(x,y)
        goaldata = receive(goalsub);
        posmsg.X = goaldata.X;
        posmsg.Y = goaldata.Y;
        posmsg.Z = 0;
        send(pospub,posmsg);
        
        reset(rate);
        for i = 1:n
            
            imudata = receive(imusub);
            rpy(i,:) = [imudata.Vector.X,imudata.Vector.Z];
            yg = [yg imudata.Vector.X];
            
            if i > lag + 5
                
                turn_init = mean(rpy(1:10,2)) - 0.1358;
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
                    if (ip_Location - iv_Location) >= 10 && (ip_Location - iv_Location) < 100 && (ip-iv) > 0.2
                        steps = steps + 1;
                        [StepLength,deltaH,PosX,PosY] = PosUpdata...
                            (iv_Location,iv,ip_Location,ip,rpy,PosX,PosY,a,b,0);
                        trajectory = [trajectory;StepLength,deltaH,PosX,PosY];
                        PosRot = [PosX,PosY]*RotMat;
                        %                         posmsg.X = PosRot(1);
                        %                         posmsg.Y = PosRot(2);
                        posmsg.X = goaldata.X + PosRot(1);
                        posmsg.Y = goaldata.Y + PosRot(2);
                        posmsg.Z = StepLength;
                        send(pospub,posmsg);
                        
                        fprintf(['steps = ',num2str(steps),' ,sneding goal...\r\n'])
                    end
                    valley = false;
                    peak = false;
                end
            end
            waitfor(rate);
        end

    end
    
end
%%
 save(char(filename))
% figure
% plot(-ans(:,2),ans(:,1),'o-'),grid minor;xlim([-3 3])
% ylabel('PositionY'),xlabel('PositionX'),legend('trajectory')
% title(['Total Distance = ',num2str(sum(trajectory(:,1)))])
