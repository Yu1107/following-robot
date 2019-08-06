% BODYDEMO Illustrates how to use the Kin2 object to get and draw the
% Skeleton data
%
% Juan R. Terven, jrterven@hotmail.com
% Diana M. Cordova, diana_mce@hotmail.com
%
% Citation:
% Terven Juan. Cordova-Esparza Diana, "Kin2. A Kinect 2 Toolbox for MATLAB", Science of
% Computer Programming, 2016. DOI: http://dx.doi.org/10.1016/j.scico.2016.05.009
%
% https://github.com/jrterven/Kin2, 2016.

addpath('Mex');
clc
clear
% Create Kinect 2 object and initialize it
% Available sources: 'color', 'depth', 'infrared', 'body_index', 'body',
% 'face' and 'HDface'
k2 = Kin2('color','depth','body','body_index');
height = 424;width = 512;
Im.figure = figure(1);
Im.ax = axes;
Im.show = imshow(zeros(height,width,3));
title('Correct Image')
Body2D.figure = figure(2);
Body2D.show = plot(0,0,0,0,0,0,0,0,0,0,0,0);
xlabel('kinect_X(m)')
ylabel('kinect_Y(m)')
title('BodyDemo2D')
grid on
axis([-2 2 -1.5 1])
%�w�q��l��T
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
%�۾��ե�����
rotation= -28/57.3;
while true
    % Get frames from Kinect and save them on underlying buffer
    validData = k2.updateData;
    % Before processing the data, we need to make sure that a valid
    % frame was acquired.
    if validData
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
            for k=1:6
                set(Body2D.show(  k  ),   ...
                    'XData',[],...
                    'YData',[])
            end
            catchbody=0; %%%%%%%%%%%%%%??????????????????????
        end
        %�Nx�b��g,y�Pz���
        if ~isempty(k2.getBodies('Quat'))
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
                set(Body2D.show( bodiescolorDx(k)    ),   ...
                    'XData',bodies(bodiesdx(bodiescolorDx(k))).Position(1,:),...
                    'YData',bodies(bodiesdx(bodiescolorDx(k))).Position(3,:),...
                    'Marker','o'   ,'MarkerSize', 5     ,'LineStyle','none',...
                    'Color', colorbody(bodiescolorDx(k),:) )
                %�H��label�������v���M��
                bodiescolorIndex=find(bodiesdx==k);
                ImagebodyIndex=find(    bodyIndex==bodyIndexnumber_dx(k)     );
                Image(ImagebodyIndex,:)=Image(ImagebodyIndex,:)+repmat(colorbod(bodiescolorIndex,:),length( ImagebodyIndex),1)   ;
            end
            %���Image�榡
            Image=uint8(Image);
            %�S������H���m�h����
            bodiesZERODx= find(bodiesdx==0);
            for k=1:length(bodiesZERODx)
                set(Body2D.show(   bodiesZERODx(k)  ),   ...
                    'XData',[],...
                    'YData',[])
            end
            %�����o����Ʀ����U�@����Ƥ��
            oldbodies=bodies;
            oldbodiesdx=bodiesdx;
        end
        % plot Image
        Image=reshape(Image,[width,height,3]);          %���s�զX�x�}�榡
        Image=permute(Image,[2 1 3]);                   %�N�x�}�� 123��m ����213��m
        Image=fliplr(Image);
        set(Im.show,'CData',Image );
        % update figure  newdrawBodies���۩w�q���
        [bodies] = k2.getBodies('Quat');
        Im.show = imshow(Image, 'Parent', Im.ax);
        k2.newdrawBodies(Im.ax,bodies,'depth',4,3,15);
        pause(0.001)%�e�ϩһݮɶ�(�ŧR)���M�|�«�
    end
end

% Close kinect object
k2.delete;

close all;
