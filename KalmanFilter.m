clear all; clc;

dur = 300;
dt = 3;

%% 진짜 state 모델링

x = [10; 6]; %초기 진짜 위치
P = [1 1;    %초기 공분산인데 모르니까 임의로 넣음
     1 1];
A = [1 dt;   %진짜 state 변화
     0 1];
g = -0.6
u = g*[1/2*dt^2; dt]; %진짜 외부에서 가한 힘
w_coeff = [10; 10];   %진짜 state 변하는 것의 error
Q = [w_coeff(1)^2       0;          %공분산
           0      w_coeff(2)^2];
       
%% measurement 모델링

H = [1 0];      %측정1
v_coeff = 2000;
R = v_coeff^2;

H2 = [1 0];     %측정2
v2_coeff = 2000;
R2 = v2_coeff^2;

%% Kalman Filter 동작

x_est = [-2000; -100]; %초기 예측 값인데 아무 값이나 넣어도 됨
x_arr = []; %진짜 state 그래프
x_est_arr = []; %예측한 state 그래프, a priori와 aposteriori 같이 씀
x_mea_arr = []; %측정한 state 그래프
x_mea_arr2 = []; %측정한 state 그래프2

for t = 0: dt :dur
    x = A*x + u + w_coeff.*randn(2, 1); %진짜 상태 변화
    x_arr = [x_arr; x(1)]; %진짜 값 저장
    
    x_est = A*x_est + u; %상태 변화 예측
    
    %1번째 Kalman Filter
    y = H*x + v_coeff*randn(1); %변화한 상태 측정
    x_mea_arr = [x_mea_arr; y]; %변화한 상태 측정한 것 저장(위치만)
    
    P = A*P*transpose(A)+Q; %a priori 공분산
    K = P*transpose(H)/(H*P*transpose(H)+R); %Kalman Gain
    x_est = x_est + K*(y - H*x_est); %a posteriori 상태 (상태 업데이트)
    P = (eye(2)-K*H)*P; %a posteriori 공분산 (공분산 업데이트)
    
    
    %2번째 Kalman Filter
    y2 = H2*x + v2_coeff*randn(1); %변화한 상태 측정2
    x_mea_arr2 = [x_mea_arr2; y2]; %변화한 상태 측정한 것 저장2(위치만)

    K2 = P*transpose(H2)/(H2*P*transpose(H2)+R2); %Kalman Gain2
    x_est = x_est + K2*(y2 - H2*x_est); %a posteriori 상태2 (상태 업데이트)
    P = (eye(2)-K2*H2)*P; %a posteriori 공분산2 (공분산 업데이트)
    
    
    x_est_arr = [x_est_arr; x_est(1)]; %칼만 필터로 예측한 값 저장
    
    plot(0:dt:t, x_arr, '-r');      %진짜 실제 위치(빨간색)
    plot(0:dt:t, x_est_arr, '-b');  %칼만필터로 예측한 위치(파란색)
    plot(0:dt:t, x_mea_arr, '-g');  %측정한 위치(초록색)
    plot(0:dt:t, x_mea_arr2, '-c'); %측정한 위치2(하늘색)
    %칼만 필터가 잘 작동하면 초록색과 하늘색이 아무리 난리쳐도 파란색은 빨간색에 가까워져야 함
    
    axis([0 dur -20000 3000]);
    hold on
end
