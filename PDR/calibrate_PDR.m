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
peakdata = zeros(1,4);
steps = 0;

%% ros
desiredRate = 50;
looptime = 25;
n = desiredRate * looptime;
rate = rosrate(desiredRate);
rate.OverrunAction = 'slip';

%imu topic
% imusub = rossubscriber('/imu/rpy/madgwick_filtered','geometry_msgs/Vector3Stamped');
imusub = rossubscriber('/imu/rpy/complementary_filtered','geometry_msgs/Vector3Stamped');
[pospub,posmsg] = rospublisher('/PDR_position','geometry_msgs/Point');
 [pdrpub,pdrmsg] = rospublisher('/PDR_SL','geometry_msgs/Point');
 
%kinect topic
boolsub = rossubscriber('/initial_position','std_msgs/Bool');
goalsub = rossubscriber('/wp','geometry_msgs/Point');
%
% startsub = rossubscriber('/short_kinect','std_msgs/Bool');

% step fomula parameter
asub = rossubscriber('/parameter','geometry_msgs/Point');
bvalue_sub = rossubscriber('/calibration_b','geometry_msgs/Point');
%% main
fprintf('start recording...\r\n')
booldata = receive(boolsub);
b = asub.LatestMessage.Y;
a = asub.LatestMessage.X;
error=0;
if booldata.Data == 1
    
    %get kinect point(x,y)
    goaldata = receive(goalsub);
    posmsg.X = goaldata.X;
    posmsg.Y = goaldata.Y;
    posmsg.Z = 0;
    send(pospub,posmsg);
    fprintf('Get the initial point\r\n')
    
    reset(rate);
    for i = 1:n
        %b = asub.LatestMessage.Y;
        %a = asub.LatestMessage.X;
        
        imudata = receive(imusub);
        rpy(i,:) = [imudata.Vector.X,imudata.Vector.Z];
        yg = [yg imudata.Vector.X];
        
        if i > lag + 5
            
            turn_init = mean(rpy(1:10,2)) ;%- 0.1358 ;
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
                    peakdata = [peakdata;iv_Location,iv,ip_Location,ip];
                    [StepLength,deltaH,PosX,PosY] = PosUpdata...
                        (iv_Location,iv,ip_Location,ip,rpy,PosX,PosY,a,b,error);
                    
                    PosRot = [PosX,PosY]*RotMat;
                    posmsg.X = goaldata.X + PosRot(1);
                    posmsg.Y = goaldata.Y + PosRot(2);
                    %                     posmsg.X = PosRot(1);
                    %                     posmsg.Y = PosRot(2);
                    posmsg.Z = StepLength;
                    send(pospub,posmsg);
                    
                    pdrmsg.X = deltaH;
                    pdrmsg.Y = iv;
                    pdrmsg.Z = ip;
                    send(pdrpub,pdrmsg);
                    fprintf(['steps = ',num2str(steps),' ,sneding goal...\r\n'])
                end
                valley = false;
                peak = false;
            end
        end
        waitfor(rate);
    end
end

save(char(filename))
