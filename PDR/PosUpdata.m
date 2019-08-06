 function [StepLength,deltaH,PosX,PosY]=PosUpdata(Min_PeakLocation,Min_PkValue,Max_PeakLocation,Max_PkValue,rpy,PositionX,PositionY,a,b,error)
% function [StepLength,PosX,PosY]=PosUpdata...
%     (Min_PeakLocation,Min_PkValue,Max_PeakLocation,Max_PkValue,rpy,PositionX,PositionY,a,b,error)

pos_start = Min_PeakLocation;
pos_end = Max_PeakLocation;

% orientation (yaw)
% YawSin = mean(( rpy(pos_start:pos_end,2)));
% YawCos = mean(( rpy(pos_start:pos_end,2)));
YawSin = median(( rpy(pos_start:pos_end,2)));
YawCos = median(( rpy(pos_start:pos_end,2)));

if YawSin > pi
    YawSin = YawSin - (pi)
end
if YawSin <-pi
    YawSin = YawSin + (pi)
end
if YawCos > pi
    YawCos = YawCos - (pi)
end
if YawCos <-pi
    YawCos = YawCos + (pi)
end

deltaH = rad2deg(Max_PkValue - Min_PkValue);
% step length estimation
% StepLength = deltaH * 0.02137 + 0.4557;
%  StepLength = deltaH*0.023721 + b;
StepLength = deltaH * a + b;
%     StepLength = deltaH*0.027111 + .383;

% position update
PosX = PositionX + StepLength * cos(YawCos-deg2rad(error));%(unit:rad)
PosY = PositionY + StepLength * sin(YawSin-deg2rad(error));
end