clear all;
close all;clc;
bag = rosbag('2018_11_21_st.bag');
geometry_message=select(bag,'MessageType','nav_msgs/Odometry');
k=geometry_message.NumMessages;
data=readMessages(geometry_message);
position=zeros(k,2);

for i=1:k
    position(i,1)=data{i,1}.Pose.Pose.Position.X;
    position(i,2)=data{i,1}.Pose.Pose.Position.Y;
end

for i=1:k
   % plot(position(i,1),position(i,2),'r.','markersize',5);
    daspect([1 1 1]);
    hold on
    
    t      = -4 * pi / 180 ;
    RotMat  = [ cos( t )   -sin( t ) ;
                sin( t )   cos( t )   ]  ;
    Pose = [position(i,1),position(i,2)] *  RotMat;  
    plot(Pose(1),Pose(2),'b.','markersize',5);
    hold on
            
            
    
    
end