%% Center of Head Test
close all; clear all; clc;
%% Initialize Model
Test = HeadDynamicsModel();
%% Plot Results
%% Angular Velocity Test
figure();
plot(Test.ts,Test.ws);
title('Angular Velocity vs. Time');
legend('\omega_x','\omega_y','\omega_z');

%% Quaternion Integration Test
figure();
plot(Test.ts,Test.qs);
title('Quaternion Components vs. Time');
legend('q_0','q_1','q_2','q_3');

%% Accelerometer Test
figure();
plot(Test.ts,Test.accel_meas);
title('Accelerometer Measurements vs. Time');
legend('a_x','a_y','a_z');

%% Gyroscope Test
figure();
plot(Test.ts,Test.gyro_meas);
title('Gyroscope Measurements vs. Time');
legend('g_x','g_y','g_z');

%% Gyro vs. Truth
figure();
plot(Test.ts,Test.ws(:,1),Test.ts,Test.gyro_meas(:,1));
title('Gyroscope Measurements vs. Truth');
legend('w_x','g_x');

%% Magnetometer Test
figure();
plot(Test.ts,Test.mag_meas);
title('Magnetometer Measurements vs. Time');
legend('m_x','m_y','m_z');

%% Visualize
color_strs = {'b','r','g','c','m'};
i_body = [1,0,0]';
j_body = [0,1,0]';
k_body = [0,0,1]';

grav_vec = Test.env_model.grav_vec;
field_vec = Test.env_model.field_vec;

%% Coordinate axes rotation
h1 = figure(); h2 = figure(); h3 = figure();
for i = 1:numel(color_strs)
    R = Quaternion.Rbw(Test.qs(i,:));
    i_world = R*i_body;
    j_world = R*j_body;
    k_world = R*k_body;
    
    figure(h1); hold on;
    quiver3(0,0,0,i_world(1),i_world(2),i_world(3),color_strs{i},'AutoScale','off');
    quiver3(0,0,0,j_world(1),j_world(2),j_world(3),color_strs{i},'AutoScale','off');
    quiver3(0,0,0,k_world(1),k_world(2),k_world(3),color_strs{i},'AutoScale','off');
end

%% Gravity vector
% Want to see all vectors pointing in similar direction
fprintf('Measured gravity vectors:\n');
for i = 1:numel(color_strs)
    figure(h2); hold on;
    g_world = Quaternion.vec2world(Test.accel_meas(i,:)',Test.qs(i,:)');
    fprintf('(x,y,z)=(%.2f,%.2f,%.2f)\n',g_world);
    quiver3(0,0,0,g_world(1),g_world(2),g_world(3),color_strs{i},'AutoScale','off');
end
quiver3(0,0,0,grav_vec(1),grav_vec(2),grav_vec(3),'k','AutoScale','off');
fprintf('Truth gravity vector: (%.2f,%.2f,%.2f)\n', grav_vec);
  
%% Magnetic field vector
% Want to see all vectors pointing in similar direction
fprintf('Measured magnetic field vectors:\n');
for i = 1:numel(color_strs)
    figure(h3); hold on;
    m_world = Quaternion.vec2world(Test.mag_meas(i,:)',Test.qs(i,:)');
    fprintf('(x,y,z)=(%.2f,%.2f,%.2f)\n',m_world);
    quiver3(0,0,0,m_world(1),m_world(2),m_world(3),color_strs{i},'AutoScale','off');
end
quiver3(0,0,0,field_vec(1),field_vec(2),field_vec(3),'k','AutoScale','off');
fprintf('Truth magnetic field vector: (%.2f,%.2f,%.2f)\n', field_vec);
