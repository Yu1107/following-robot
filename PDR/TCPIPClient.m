%%%%%%%%%%%%%%%%%%%%%%
%253-264���T�w���H�����H����Ѽ�
%Serve_Client_Goal �쬰 0 0 0
%Goal_number=1;���ü{  ���H�����H�{���������� [bodiesdx]�����Y
%%%%%%%%%%%%%%%%%%%%%
clc
clear
%�e��
height = 424;width = 512;
Im.figure = figure(1);
Im.ax = axes;
Im.show = imshow(zeros(height,width,3));
title('Correct Image')
encoder.figure = figure(2);
encoder.show = plot(0,0);
xlabel('Robot_X(m)')
ylabel('Robot_Y(m)')
grid on
%  https://blogs.mathworks.com/loren/2011/05/27/transferring-data-between-two-computers-using-matlab/
t = tcpip('192.168.0.202',55000,'NetworkRole','Client');
set(t,'InputBufferSize',24);   %�᭱�Ʀr�i�� whos�h�d  �����W������
fopen(t);
%�Pvs2013�s��
robot_t = tcpip('localhost', 1234, 'NetworkRole', 'client');
fopen(robot_t);
% Create Kinect 2 object and initialize it
addpath('Mex');
k2 = Kin2('color','depth','body','body_index');
%��l���
oldbodies=[];
catchbody=0;
%�w�q�H���m�����C���T
colorbody=[0.5 0 0;
    0 0.5 0;
    0 0 0.5;
    0.3 0.5 0.2;
    0.5 0.5 0;
    0.5 0 0.5];
colorbod=colorbody*255;
%��l��T��s
Serve_Client_Goal=[1 1 1 ];%'false'     %%%%%%%%%%%%%%%%%%%%%%%[1 1 1]
Serve_bodygoal=[0 0 0];
Detection_first_head=1;%'true'
Missing_Target=0;%'false'
Omega_time_start=0;
follow_Omega_time_start=0;
robot_i=1;
preLeftWheelPos=0;
preRightWheelPos=0;
robot_X=0.65;
robot_Y=0.2;
robot_Dir=0/57.3;
rotation= 25/57.3;%�۾��ե�����
WheelRadius=0.085;
while true
    validData = k2.updateData;
    %�p�GClient�PServe�ؼФ��@�P�A�hServe���򴣨ѥؼи�T
    if all(Serve_Client_Goal)==0
        Serve_bodygoal = fread(t,3,'double')  ;   %�᭱�Ʀr�i�� whos�h�d  �����W������
        Serve_bodygoal = Serve_bodygoal';
    end
    %�p�GServe�S���ݨ�ؼСA�BClient�PServe�ؼФ��@�P �h���򵥫�Serve�ǰe��T
    if all(Serve_bodygoal==0) && all(Serve_Client_Goal)==0
        str_send = [num2str(0) ';' num2str(0)];
        fwrite(robot_t, str_send);
        %%%%%%%-----------------------Ū��encoder��T-----------------------------%%%%%%%
        pause(0.03) %�C��data���w�Įɶ�  (���t����)
        if robot_t.BytesAvailable
            [robot_X , robot_Y, robot_Dir,preLeftWheelPos,preRightWheelPos]=...
                encoderonline(robot_t,robot_X,robot_Y,robot_Dir,preLeftWheelPos,preRightWheelPos,robot_i);
            robotPosition(robot_i,:)=[ robot_X  robot_Y robot_Dir*180/pi ] ;
            set(encoder.show(1),...
                'XData',robotPosition(:,1),...
                'YData',robotPosition(:,2),...
                'Marker','o'   ,'MarkerSize', 5     ,'LineStyle','none')
            robot_i=robot_i+1;
        end
        continue
    end
    %Serve_kinect������
    if all(Serve_Client_Goal)==0
        Xg=(Serve_bodygoal(1)-robot_X);
        Zg=(Serve_bodygoal(2)-robot_Y);
        theta=atan(Xg/Zg);%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        Goal=[    Xg  Zg  (-theta+robot_Dir)    ];
        Goal_distance=sqrt(   (Xg)^2  +   (Zg)^2     );
        %����Ѽ�
        min_distance=1.3;
        max_distance=1.5;
        if Omega_time_start==0
            Omega_time=tic;
        end
        Omega_time_start=1;
        time_toc=toc(Omega_time);
        if time_toc<0.5
            Omega_gain=1;
            foward_gain=0.5;
        elseif time_toc>=0.5&&time_toc<1
            Omega_gain=1.8;
            foward_gain=0.8;
        elseif time_toc>=1.5&&time_toc<2
            Omega_gain=2.5;
            foward_gain=1.2;
        elseif time_toc>=2
            Omega_gain=2.5;
            foward_gain=1.5;
        end
        %Robot��h
        if Goal_distance<=min_distance
            minv=Goal_distance*(-2);
            wl=Goal(3)*(Omega_gain);
        end
        %Robot����
        if Goal_distance>min_distance && Goal_distance<max_distance
            minv=0;
            wl=Goal(3)*Omega_gain;
        end
        %Robot�e�i
        if Goal_distance>=max_distance
            minv=Goal_distance*foward_gain;
            wl=Goal(3)*Omega_gain;
        end
        vr=(minv+wl)/(WheelRadius*2); %�����l���|�A�k���t��
        vl=(minv-wl)/(WheelRadius*2);  %�����l���|�A�����t��
        str_send = [num2str(vl) ';' num2str(vr)];
        fwrite(robot_t, str_send);
        %%%%%%%-----------------------Ū��encoder��T-----------------------------%%%%%%%
        pause(0.03) %�C��data���w�Įɶ�  (���t����)
        if robot_t.BytesAvailable
            [robot_X , robot_Y, robot_Dir,preLeftWheelPos,preRightWheelPos]=...
                encoderonline(robot_t,robot_X,robot_Y,robot_Dir,preLeftWheelPos,preRightWheelPos,robot_i);
            robotPosition(robot_i,:)=[ robot_X  robot_Y robot_Dir*180/pi ] ;
            set(encoder.show(1),...
                'XData',robotPosition(:,1),...
                'YData',robotPosition(:,2),...
                'Marker','o'   ,'MarkerSize', 5     ,'LineStyle','none')
            robot_i=robot_i+1;
        end
    end
    if validData
        %�p�G�����H���ݨ�F��A�PServe�ݽT�{
        [bodies] = k2.getBodies('Quat');
        if ~isempty(k2.getBodies('Quat')) && all(Serve_Client_Goal)==0
            for i=1:length(bodies)
                %xlabel�M�g
                bodies(i).Position(1,:)=-bodies(i).Position(1,:);
                %�۾��ե�����[z=cos*z+sin*y] [y=sin*z+cos*y]
                bodies(i).Position(3,:) =   cos(rotation)*bodies(i).Position(3,:) -sin(rotation)*bodies(i).Position(2,:);
                bodies(i).Position(2,:) =   sin(rotation)*bodies(i).Position(3,:) +cos(rotation)*bodies(i).Position(2,:);
                %�۾��ե�����[x=cos*x+sin*z] [z=sin*x+cos*z]
                bodies(i).Position(1,:) =   cos(robot_Dir)*bodies(i).Position(1,:) -sin(robot_Dir)*bodies(i).Position(3,:);
                bodies(i).Position(3,:) =   sin(robot_Dir)*bodies(i).Position(1,:) +cos(robot_Dir)*bodies(i).Position(3,:);
                % kinect [x,y,z] to  kinect [x,z,y]
                bodies(i).Position([2 3],:)=bodies(i).Position([3 2],:);
                %Client�ݨ줧�Z��
                candidate_x=bodies(i).Position(1,21);
                candidate_z=bodies(i).Position(2,21);
            end
            %�P�_Client�PServe�ݨ쪺�H�O�_�@��
            candidate_D=sqrt(   (Xg-candidate_x).^2  +   (Zg-candidate_z).^2     );
            candidate_dx=find(     candidate_D<=1        );
            
            if length(candidate_dx)==1&&Goal_distance<=3
                Goal_number=candidate_dx;
                Serve_Client_Goal=[1 1 1 ] ;
                fwrite(t,Serve_Client_Goal,'double');
                pause(0.03) %�g�Jdata����ɶ�  (���t����)
                continue%���ܸ��H�����H�{��
            end
        end
    end
    %%%%%%%%%%%%------------------- following robot  work start---------------------- %%%%%%%%%%%%
    %%%%%%%%%%%%------------------- following robot  work start---------------------- %%%%%%%%%%%%
    %%%%%%%%%%%%------------------- following robot  work start---------------------- %%%%%%%%%%%%
    %%%%%%%%%%%%------------------- following robot  work start---------------------- %%%%%%%%%%%%
    % Get frames from Kinect and save them on underlying buffer
    %Before processing the data, we need to make sure that a valid frame was acquired.
    %Client_Kinect������
    if validData&& all(Serve_Client_Goal)==1
        pc = k2.getPointCloud('output','pointCloud','color','true');
        Image=pc.Color;
        %�`�I����index�w�q�j�p
        datadx =zeros(   424*512,1  );
        %�h���勵���n �üе���-2 ----��kinect�^���{���Y�S�ե��n�������C��|�ܨ��ܥh���@�w�O�Ǧ�A�ҥH�T�w���զ�
        regulatedx=find(Image(:,1)==Image(217088,1)&Image(:,2)==Image(217088,2)&Image(:,3)==Image(217088,3));
        Image( regulatedx,: ) =  255 ;
        datadx(regulatedx,1)=-2;
        %�H�鰩�[�����j��
        [bodies] = k2.getBodies('Quat');
        %�H�鰩�[�S��������j��
        if isempty(k2.getBodies('Quat'))
            str_send = [num2str(0) ';' num2str(0)];
            fwrite(robot_t, str_send);
            %%%%%%%-----------------------Ū��encoder��T-----------------------------%%%%%%%
            pause(0.03) %�C��data���w�Įɶ�  (���t����)
            if robot_t.BytesAvailable
                [robot_X , robot_Y, robot_Dir,preLeftWheelPos,preRightWheelPos]=...
                    encoderonline(robot_t,robot_X,robot_Y,robot_Dir,preLeftWheelPos,preRightWheelPos,robot_i);
                robotPosition(robot_i,:)=[ robot_X  robot_Y robot_Dir*180/pi ] ;
                set(encoder.show(1),...
                    'XData',robotPosition(:,1),...
                    'YData',robotPosition(:,2),...
                    'Marker','o'   ,'MarkerSize', 5     ,'LineStyle','none')
                robot_i=robot_i+1;
            end
            if catchbody==1
                Missing_Target=1;
            end
            catchbody=0;
            follow_Omega_time_start=0;
            bodiesdx=zeros(6,1);
            oldbodiesdx=zeros(6,1);
        end
        %�Nx�b��g,y�Pz���
        if ~isempty(k2.getBodies('Quat'))
            Missing_Target=0;
            for i=1:length(bodies)
                %xlabel�M�g
                bodies(i).Position(1,:)=-bodies(i).Position(1,:);
                %�۾��ե�����[x=cos*x+sin*y] [y=sin*x+cos*y]
                bodies(i).Position(3,:) =   cos(rotation)*bodies(i).Position(3,:) -sin(rotation)*bodies(i).Position(2,:)  ;
                bodies(i).Position(2,:) =   sin(rotation)*bodies(i).Position(3,:) +cos(rotation)*bodies(i).Position(2,:);
                % kinect [x,y,z] to  kinect [x,z,y]
                bodies(i).Position([2 3],:)=bodies(i).Position([3 2],:);
            end
        end
        %�Ĥ@��������H�鰩�[�j��
        if catchbody==0 &&~isempty(k2.getBodies('Quat'))
            bodiesdx=zeros(6,1);
            for k=1:length(bodies)
                bodiesdx(k)=k;
            end
            oldbodies=bodies;
            catchbody=1;
        end
        %�ĤG�����᰻����H�鰩�[�j��
        if catchbody==1 &&~isempty(k2.getBodies('Quat'))
            Change_person =length(bodies)-  length(oldbodies) ; %�H���ܤƧP�_
            if  Change_person~=0
                [bodiesdx]=bodiesdx_func(Change_person,bodies,oldbodies,oldbodiesdx);
            end
            %%%%%%%-----------------------��J�����F-----------------------------%%%%%%%
            Goal_number=bodiesdx(1);
            Xg=bodies(Goal_number).Position(1,21);
            Zg=bodies(Goal_number).Position(2,21);
            theta=atan(Xg/Zg);
            Goal=[Xg Zg -theta];
            Goal_distance=sqrt(Xg^2+Zg^2);
            %����Ѽ�
            min_distance=1.3;
            max_distance=1.5;
            if follow_Omega_time_start==0
                follow_Omega_time=tic;
            end
            follow_Omega_time_start=1;
            follow_time_toc=toc(follow_Omega_time);
            % if follow_time_toc<0.5
            %     folow_Omega_gain=1;
            %     follow_foward_gain=0.8;
            % elseif follow_time_toc>=0.5&&follow_time_toc<1
            %     folow_Omega_gain=1.5;
            %     follow_foward_gain=1.4;
            % elseif follow_time_toc>=1&&follow_time_toc<1.5
            %     folow_Omega_gain=2;
            %     follow_foward_gain=2;
            % elseif follow_time_toc>=1.5
            %     folow_Omega_gain=2.5;
            %     follow_foward_gain=3;
            % end
            follow_foward_gain=0.5;
            folow_Omega_gain=1;
            %stop reset speed
            if Goal_distance<=max_distance
                follow_Omega_time_start=0;
            end
            %Robot��h
            if Goal_distance<=min_distance
                minv=Goal_distance*(-0.7);
                wl=Goal(3)*(folow_Omega_gain);
            end
            %Robot����
            if Goal_distance>min_distance && Goal_distance<max_distance
                minv=0;
                wl=Goal(3)*folow_Omega_gain;
            end
            %Robot�e�i
            if Goal_distance>=max_distance
                minv=Goal_distance*follow_foward_gain;
                wl=Goal(3)*folow_Omega_gain;
            end
            vr=(minv+wl)/(WheelRadius*2); %�����l���|�A�k���t��
            vl=(minv-wl)/(WheelRadius*2);  %�����l���|�A�����t��
            str_send = [num2str(vl) ';' num2str(vr)];
            fwrite(robot_t, str_send);
            %%%%%%%-----------------------Ū��encoder��T-----------------------------%%%%%%%
            pause(0.03) %�C��data���w�Įɶ�  (���t����)
            if robot_t.BytesAvailable
                [robot_X , robot_Y, robot_Dir,preLeftWheelPos,preRightWheelPos]=...
                    encoderonline(robot_t,robot_X,robot_Y,robot_Dir,preLeftWheelPos,preRightWheelPos,robot_i);
                robotPosition(robot_i,:)=[ robot_X  robot_Y robot_Dir*180/pi ] ;
                set(encoder.show(1),...
                    'XData',robotPosition(:,1),...
                    'YData',robotPosition(:,2),...
                    'Marker','o'   ,'MarkerSize', 5     ,'LineStyle','none')
                robot_i=robot_i+1;
            end
            %%%%%%%%%------------�H�骺�v��label������bodies�M��------------%%%%%%%%%
            bodyIndex = k2.getBodyIndex;
            bodyIndex=reshape(bodyIndex',[217088 1]) ;
            %�M��body��label�s��
            bodyIndexnumber_dx=unique(bodyIndex);
            %�h���s����255
            bodyIndexnumber_dx(length(bodyIndexnumber_dx))=[];
            %���Image�榡
            Image=double(Image);
            %�е��H���C��
            bodiescolorDx= find(bodiesdx~=0);
            for k=1:length(bodies)
                %�H��label�������v���M��
                bodiescolorIndex=find(bodiesdx==k);
                ImagebodyIndex=find(    bodyIndex==bodyIndexnumber_dx(k)     );
                Image(ImagebodyIndex,:)=Image(ImagebodyIndex,:)+repmat(colorbod(bodiescolorIndex,:),length( ImagebodyIndex),1)   ;
            end
            %���Image�榡
            Image=uint8(Image);
            %�����o����Ʀ����U�@����Ƥ��
            oldbodies=bodies;
            oldbodiesdx=bodiesdx;
            %�M�I�P�_�[�J�A�p�G�Y�������@�P�ܤ�
            Y_head=bodies(Goal_number).Position(3,4);
            if Detection_first_head==1
                oldY_head=Y_head;
            end
            if  (oldY_head-Y_head)>=0.07
                Warningmodel=[0 1 0 ]   ;
                fwrite(t,Warningmodel,'double');
                pause(0.03)
            end
            oldY_head=Y_head;
            Detection_first_head=0;
            %�M�I�P�_�[�J�A�p�G�ѤH�ݭn����
            shoulderYY=bodies(Goal_number).Position(2,5)-bodies(Goal_number).Position(2,9);
            shoulderXX=bodies(Goal_number).Position(1,5)-bodies(Goal_number).Position(1,9);
            shoulder_degree=abs(atan(shoulderYY/shoulderXX))*57.3;
            hand_distance=sqrt( (bodies(Goal_number).Position(1,24)-bodies(Goal_number).Position(1,22))^2+...
                (bodies(Goal_number).Position(2,24)-bodies(Goal_number).Position(2,22))^2   );
            if shoulder_degree<=10 && hand_distance<=0.05
                Warningmodel=[0 0 1 ]   ;
                fwrite(t,Warningmodel,'double');
                pause(0.03)
            end
        end
        %�M�I�P�_�[�J�A�p�G�򥢥ؼ�
        if Missing_Target==1
            Warningmodel=[1 0 0 ]  ;
            fwrite(t,Warningmodel,'double');
            pause(0.03)
        end
        % plot Image
        Image=reshape(Image,[width,height,3]);          %���s�զX�x�}�榡
        Image=permute(Image,[2 1 3]);                   %�N�x�}�� 123��m ����213��m
        Image=fliplr(Image);
        set(Im.show,'CData',Image );
        pause(0.001)%�e�ϩһݮɶ�(�ŧR)���M�|�«�
    end
end