D2R=pi/180;
R2D=180/pi;


[q1 q2] = textread('Encoder.txt');

size = 1000000; % data size , 1001
sTime = 0.001; % sampling time , 1ms

des_q1 = 41;
des_q2 = 62;
des_q3 = 56;
des_q4 = -43;
des_q5 = 3;
des_q6 = 0;

for i=1:size-1
des_q1 = [des_q1 ; 10]; 
des_q2 = [des_q2 ; 10];
des_q3 = [des_q3 ; 10]; 
des_q4 = [des_q4 ; 10]; 
des_q5 = [des_q5 ; 10];
des_q6 = [des_q6 ; 10];
end
                          
figure(1)
tmTime=(size-1)*sTime;
subplot(6,1,1);hold on;grid on;
plot(0:sTime:tmTime,q1,'Color','k');
plot(0:sTime:tmTime,des_q1,'-.','Color','k'); 

legend('q1','desired')
ylabel('Angle[deg]');xlabel('time[s]');


subplot(6,1,2);hold on;grid on;
plot(0:sTime:tmTime,q2*D2R,'Color','k'); 
plot(0:sTime:tmTime,des_q2,'-.','Color','k'); 
legend('q2','desired')
ylabel('Angle[deg]');xlabel('time[s]');

subplot(6,1,3);hold on;grid on;
plot(0:sTime:tmTime,q3*D2R,'Color','k'); 
plot(0:sTime:tmTime,des_q3,'-.','Color','k'); 
legend('q3','desired')
ylabel('Angle[deg]');xlabel('time[s]');

subplot(6,1,4);hold on;grid on;
plot(0:sTime:tmTime,q4*D2R,'Color','k'); 
plot(0:sTime:tmTime,des_q4,'-.','Color','k'); 
legend('q4','desired')
ylabel('Angle[deg]');xlabel('time[s]');

subplot(6,1,5);hold on;grid on;
plot(0:sTime:tmTime,q5*D2R,'Color','k'); 
plot(0:sTime:tmTime,des_q5,'-.','Color','k'); 
legend('q5','desired')
ylabel('Angle[deg]');xlabel('time[s]');

subplot(6,1,6);hold on;grid on;
plot(0:sTime:tmTime,q6*D2R,'Color','k'); 
plot(0:sTime:tmTime,des_q6,'-.','Color','k'); 
legend('q6','desired')
ylabel('Angle[deg]');xlabel('time[s]');
% plot(0:sTime:tmTime,q4,'-.','Color','k'); 
% plot(0:sTime:tmTime,qd4,'Color','k'); 


%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%
%plot(0:0.2:time,endx,'-.','Color','k');  
% plot(0:0.2:time,x_err,':','Color','k'); 
% legend('Head motion','End-effector','Error')
% title('Position error in Head motion compensation');
% ylabel('X-axis[mm]');xlabel('time[s]');
%   
% 
% subplot(3,1,2);hold on;grid on;
% plot(0:0.2:time,hy,'Color','k');plot(0:0.2:time,endy,'-.','Color','k');  plot(0:0.2:time,y_err,':','Color','k');
% legend('Head motion','End-effector','Error')
% ylabel('Y-axis[mm]');xlabel('time[s]');
% 
% 
% subplot(3,1,3);hold on;grid on;
% plot(0:0.2:time,hz,'Color','k');plot(0:0.2:time,endz,'-.','Color','k');  plot(0:0.2:time,z_err,':','Color','k');
% legend('Head motion','End-effector','Error')
% ylabel('Z-axis[mm]');xlabel('time[s]');
% %-------------------------^^orientation^^----------------------
% 
% %\\\\\\\\\\\\\\\\\\\\orientation\\\\\\\\\\\\\\\\\\\\\\\\\
% figure(2)
% 
% subplot(3,1,1);hold on;grid on;
% plot(0:0.2:time,h_rot_z1,'Color','k'); plot(0:0.2:time,end_rot_z1,'-.','Color','k'); plot(0:0.2:time,rot_z1_err,':','Color','k');   
% title('Orientation error in Head motion compensation')
% legend('Head motion','End-effector','Error')
% ylabel('Z_1 angle[rad]');xlabel('time[s]');
%   
% 
% subplot(3,1,2);hold on;grid on;
% plot(0:0.2:time,h_rot_y,'Color','k');plot(0:0.2:time,end_rot_y,'-.','Color','k'); plot(0:0.2:time,rot_y_err,':','Color','k'); 
% legend('Head motion','End-effector','Error')
% ylabel('Y angle[rad]');xlabel('time[s]');
% 
% 
% subplot(3,1,3);hold on;grid on;
% plot(0:0.2:time,h_rot_z2,'Color','k');plot(0:0.2:time,end_rot_z2,'-.','Color','k'); plot(0:0.2:time,rot_z2_err,':','Color','k'); 
% legend('Head motion','End-effector','Error')
% ylabel('Z_2 angle[rad]');xlabel('time[s]');
% %-------------------------^^orientation^^----------------------

% 
% time=50
% sqrt((px_error(time)^2+py_error(time)^2+pz_error(time)^2)/3)
% 180/pi*sqrt((pitch_error(time)^2+roll_error(time)^2+yaw_error(time)^2)/3)