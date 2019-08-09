count = 0;
for i = 1:N
    if abs(IMU.DATAf(4,i))> 0.01 || abs(IMU.DATAf(5,i))> 0.01 || abs(IMU.DATAf(6,i))> 0.01, count = count + 1;end
    if count>150,break;end
end
disp(i)

% i = 1;
% while abs(IMU.DATAf(4,i))< 0.01 || abs(IMU.DATAf(5,i))< 0.01 || abs(IMU.DATAf(6,i))< 0.01
%     i = i+1;
% end
% disp(i)