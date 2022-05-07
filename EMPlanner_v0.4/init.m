%%加载油门刹车标定表
load('calibration_table');

%%%%整车参数%%%%%
cf=-72653;
cr=-121449;
m=1820;
Iz=3746;
la=1.17;
lb=1.77;
%%%%%%%%%%%%%%%%%%%%
%%%%纵向双PID参数
KP_PID_distance=0.5;
KI_PID_distance=0.0;
KD_PID_distance=0.0;
KP_PID_speed=1.8;
KI_PID_speed=0;
KD_PID_speed=0;
%%%%%%%横向LQR参数
LQR_Q1=20;
LQR_Q2=3;
LQR_Q3=5;
LQR_Q4=1;
LQR_R=50;
%%%%生成离线LQR标定表LQR_OFFLINE
k=zeros(5000,4);
vx_break_point=zeros(1,5000);
for i=1:5000
    vx_break_point(i)=0.01*i;
    
    A=[0,1,0,0;
        0,(cf+cr)/(m*vx_break_point(i)),-(cf+cr)/m,(la*cf-lb*cr)/(m*vx_break_point(i));
        0,0,0,1;
        0,(la*cf-lb*cr)/(Iz*vx_break_point(i)),-(la*cf-lb*cr)/Iz,(la*la*cf+lb*lb*cr)/(Iz*vx_break_point(i))];
    B=[0;
        -cf/m;
        0;
        -la*cf/Iz];
    LQR_Q=1*[LQR_Q1,0,0,0;
        0,LQR_Q2,0,0;
        0,0,LQR_Q3,0;
        0,0,0,LQR_Q4];

    k(i,:)=lqr(A,B,LQR_Q,LQR_R);
end
LQR_K1=k(:,1)';
LQR_K2=k(:,2)';
LQR_K3=k(:,3)';
LQR_K4=k(:,4)';

%%%%车辆初始位置
host_x_init=0; 
host_y_init=0;