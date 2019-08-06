clear all; close all;    clear m;

% smartphone
m = mobiledev;
% m.SampleRate = 'high';
m.AngularVelocitySensorEnabled = 1;
m.AccelerationSensorEnabled = 1;
pause(0.01);
m.Logging = 1;
starttime = clock;
disp('start');
k = waitforbuttonpress;

if k == 0
    m.Logging = 0;
    stoptime = clock;
    m.AngularVelocitySensorEnabled = 0;
    m.AccelerationSensorEnabled = 0;
    disp('stop');
else
    disp('Key press')
end
costtime = etime(stoptime, starttime)

% get sensor data of gyro
[av, tav] = angvellog(m);

for k = 2:length(tav)
    p = av(k,1); q = av(k,2); r = av(k,3);
end

% get sensor data of accel
[acc, tacc] = accellog(m);

for k = 2:length(tacc)
    ax = acc(k,1); ay = acc(k,2); az = acc(k,3);
end
% get sensor data of magnetic field

[log, timestamp] = magfieldlog(m);
%% initialization
nSamples = min([length(tav), length(tacc)]);
ekfEulerSaved = zeros(nSamples, 3);
tkalman = zeros(nSamples, 1);
% kalman filter
rot='yxz';

for k =1:nSamples
    % get gyro value
    p = av(k,1); q = av(k,2); r = av(k,3);
    if k == 1
        dt = tav(1);
    else
        dt = tav(k) - tav(k-1);
    end
    
    A = eye(4) + dt * 1/2 * [0 -p -q -r;
        p 0 r -q;
        q -r 0 p;
        r q -p 0]
    % get accel value
    % find the nearest time stamp in tacc
    [minValue,accIdx] = min( abs(tav(k) - tacc));
    if minValue > 10
        accIdx = k;
    end
    tkalman(k) = tacc(accIdx);
    
    ax = acc(accIdx,1); ay = acc(accIdx,2); az = acc(accIdx,3);
    [phi_a, theta_a] = convertAccel2Euler(ax, ay, az);
    [phi, theta, psi] = ekf_euler([phi_a theta_a]', [p q r], dt);
    quatEKF = euler2quaternion(phi, theta, psi, rot);
    ekfEulerSaved(k, :) = [phi theta psi];
end

% visualize results
ekfPhiSaved = ekfEulerSaved(:,1)*180/pi;
ekfThetaSaved = ekfEulerSaved(:,2)*180/pi;
ekfPsiSaved = ekfEulerSaved(:,3)*180/pi;

% figure('Name','EKF Euler angle','NumberTitle','off'),
% plot(tkalman, ekfPhiSaved, tkalman, ekfThetaSaved, tkalman,ekfPsiSaved);
% legend( 'pitch(x)','roll(z)', 'yaw(y)'), title('ekf'), xlabel('smaple'), ylabel('angle')

%% Find signal peaks - peaks under a threshold value are considered as noise.
clc,close all
data=rpy(953:1300,:);
[Max_PkValue, Max_PeakLocation] = findpeaks(data(:,1), 'MINPEAKHEIGHT', 1.7,'minpeakdistance',50);
[Min_PkValue, Min_PeakLocation] = findpeaks(-data(:,1), 'MINPEAKHEIGHT',-1.5,'minpeakdistance',50);
if Min_PeakLocation(1) >  Max_PeakLocation(1)
    Max_PeakLocation = Max_PeakLocation(2:end);
    Max_PkValue = Max_PkValue(2:end);
end
if Min_PeakLocation(end) >  Max_PeakLocation(end)
    Min_PeakLocation(end) = [];
    Min_PkValue(end) = [];
end

figure('Name','Find signal peaks','NumberTitle','off'),hold on
plot(data(:,1));      %pitch angle
plot(Max_PeakLocation,data(Max_PeakLocation),'rv','MarkerFaceColor','r');
plot(Min_PeakLocation,data(Min_PeakLocation),'rs','MarkerFaceColor','b');
legend('pitch','theta_m_a_x','theta_m_i_n'), xlabel('smaple'), ylabel('pitch[rad]')

deltaH = rad2deg(Max_PkValue-abs(Min_PkValue));
StepCount = length(Max_PeakLocation); % step number
% position update
PositionX = zeros(StepCount, 1);
PositionY = zeros(StepCount, 1);
distance = 0;

turn_init = mean(data(1:10,2)); -0.1358;
% turn_init = -2.1243;


RotMat  = [ cos( turn_init )   -sin( turn_init ) ;
    sin( turn_init )   cos( turn_init )   ]  ;

%3/28-16-2346
%     b= [.0357;.2357;.4757];
%   YawSin=[2.3918;2.842;2.427];YawCos=YawSin;

for k = 1:StepCount
    pos_start = Min_PeakLocation(k);
    pos_end = Max_PeakLocation(k);
    
    % orientation (yaw)(unit:rad)
    YawSin(k, 1) = mean( data(pos_start:pos_end,2)) ;% *(turn_init);
    YawCos(k, 1) = mean( data(pos_start:pos_end,2)) ;% *(turn_init);
    
    %for i = 1:100
        %sclar(k,i) = i*0.001+0.2;
    % step length estimation
    StepLength(k,1) = ( deltaH(k,1))*0.02137 + .4557;%b(k);
    %     StepLength(k,1) = deltaH(k,1)*0.027111 + .383;
    
    w=0.5;
    % position update
    PositionX(k+1,1) = PositionX(k) + StepLength(k,1) * cos(YawCos(k,1)) ;%(unit:rad)
    PositionY(k+1,1) = PositionY(k) + StepLength(k,1) * sin(YawSin(k,1)) ;
    %MLINSQRT(k,i)=(kkk(2,1)+PositionX(k,i))+(kk(2,1)+PositionY(k,i));
    %end
end
distance = sum(StepLength);

fprintf('RotMat = %f,distance = %f\r\n',rad2deg(turn_init),distance)
[PositionX,PositionY]*RotMat;
 %%
%  figure
%  plot(-PositionY,PositionX,'o-'),grid on;
%  xlabel('PositionY'); ylabel('PositionX');%xlim([-3 1]),ylim([0 4])


 figure
%  plot(-ans(:,2)-goaldata.Y,ans(:,1)+goaldata.X,'bo-'),grid on;xlabel('PositionY'); ylabel('PositionX');%xlim([-3 1]),ylim([0 4])
plot(-PositionY,PositionX,'o-'),grid on;
 
% figure
% plot(-PositionX,PositionY,'o-'),grid on;
% xlabel('PositionX'); ylabel('PositionY');
%    title([' estimation=',num2str(distance)]);legend('trajectory'); 

%  line=length(PositionY)/3;
% plot(PositionY(1:line),PositionX(1:line),'ro-');hold on
%     plot(PositionY(line+1:line*2),PositionX(line+1:line*2),'go-');
%          plot(PositionY(line*2+1:end),PositionX(line*2+1:end),'bo-');
%          legend('loop1','loop2','loop3');xlabel('PositionY'); ylabel('PositionX');