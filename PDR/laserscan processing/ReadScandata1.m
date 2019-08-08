%With PointCloud2 objects you should be able to get the intensity values using 'readRGB':
%https://www.mathworks.com/help/robotics/ref/readrgb.html


%  clear,
%  close all,
clc
addpath('/home/lab5254/Desktop/report/108-6-30(static  velodyne/static');
filename='2019-06-30-21-54-04.bag';
bag = rosbag(filename);

p=[];

% f = figure('visible','off');
f = figure(1);
hold on;
%% read SICK laserscan
% scanSel = select(bag,'Topic','/scan');
% scan = readMessages(scanSel);
% scants = timeseries(scanSel);

%  c=120;
%  a =round( 97/c*length(scan));
%  b = round( 107/c*length(scan));

%  for i = a:2:b
%     ranges = scan{i,1}.Ranges;
%     angles = scan{i,1}.readScanAngles;
%
%     for k=1:length(scan{i, 1}.Intensities)
%         if scan{i, 1}.Intensities(k,1) < 200
%
%              scan{i,1}.Ranges(k,1)=single(Inf);
%         end
%     end
%      plot(scan{i,1}),hold on,axis equal
% %      i
% end

%% read velodyne laserscan
% scanSel = select(bag,'Topic','/velodyne_points');
% 
% scan=readMessages(scanSel);



for kk = 1:length(Untitled)
    kk
    starttime = Untitled(kk,1)-0.1;
    endtime = starttime+0.1;
    %choose timestamp
    bagselect2 = select(bag,'Time',...
        [starttime endtime],'Topic','/velodyne_points');
    scan=readMessages(bagselect2);
    
    
    %% torso
    for i=1:1:length(scan)
        
        a = roundn(readXYZ(scan{i}),-2); %roundn—任意位位置四舍五入
        for g=1:length(a)
            
            
            if a(g,3) > 0.5% && a(g,3) < 0.5  %high
                if a(g,1) >-10 && a(g,1) <10 && a(g,2) >-10 && a(g,2) <10 %limit xaxis yaxix
                    %                  plot(a(g,1),a(g,2),'b.');
                    p=[p;a(g,1),a(g,2)];
                end
            end
        end
    end
    %%
    
    %
    % c = 59; %54 %59
    % start =round( 31/c*length(scan)); %28 %31
    % b = round( 33/c*length(scan)); %40 %49
    % final = round( 49/c*length(scan)); %40 %49
    %%
    
    if kk > 2
        %%leg
        for i=1:1:length(scan)
            
            a = roundn(readXYZ(scan{i}),-2); %roundn—任意位位置四舍五入
            for g=1:length(a)
                if a(g,3) > 0 && a(g,3) < 0.5  %high
                    if a(g,1) >-10 && a(g,1) <10 && a(g,2) >-10 && a(g,2) <10 %limit xaxis yaxix
                        %                  plot(a(g,1),a(g,2),'b.');
                        p=[p;a(g,1),a(g,2)];
                        
                    end
                end
            end
            
        end
    end
    
end


% plot(p(:,1),p(:,2),'b.')
%%
turn_init = 3.5;
RotMat  = [ cos(turn_init)  -sin(turn_init);...
    sin(turn_init)   cos(turn_init)];
p=[p(:,1),p(:,2)]*RotMat;

%   p=[p(:,1)+1.76,p(:,2)+6.25];
p=[p(:,1)+0.5209,p(:,2)+6.351];
plot(p(:,1),p(:,2),'b.')

axis equal,grid minor
xlabel('position X[m]'), ylabel('position Y[m]')
title('Movement trajectory')
% saveas(f,'path','jpg')
% saveas(f,'path','fig')

fprintf('ok\r\n')
