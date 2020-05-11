%evolution time
close all
sampling_time = 0.001;
tspan= 0:sampling_time:1;

print_act = 1;
test = '_secondcase';

%--------------------------TIME SIMULATION DEFINITION && PARAMETERS DEFINITION--------------------------------------

g = [0 0 -9.81];         %gravity vector fixed reference frame

p_omega_h = [0 0 0];     %angular momentum at vertex body frame at initial condition %[0 -10 -10]; [-200 0 -10]

p_omega_w = [0 0 0];     %angular momentum reaction wheels at initial condition

% attitude = [0 0 0 1];    %expressed in quaternions at initial condition is aligned

alpha_x = deg2rad(45); %0 | 45

R_x = [ 1           0                      0;
        0       cos(alpha_x)         -sin(alpha_x);
        0       sin(alpha_x)         cos(alpha_x);];

R_x_cube = [ 1           0                      0;
             0       cos(-alpha_x)         -sin(-alpha_x);
             0       sin(-alpha_x)         cos(-alpha_x);];
       
 beta_y = deg2rad(-30); %0 | -30
 
 R_y = [ cos(beta_y)           0            sin(beta_y);
            0                  1                0       ;
         -sin(beta_y)          0              cos(beta_y);];
     
 R_y_cube = [ cos(-beta_y)           0             sin(-beta_y)   ;
                  0                  1                0           ;
              -sin(-beta_y)           0              cos(-beta_y);];
          
 gamma_z = deg2rad(25);
 
 R_z = [ cos(gamma_z)         -sin(gamma_z)        0;
         sin(gamma_z)          cos(gamma_z)        0;
         0                              0            1];
     
 R_z_cube =[ cos(-gamma_z)         -sin(-gamma_z)        0;
         sin(-gamma_z)          cos(-gamma_z)        0;
         0                              0            1];
 

attitude = dcm2quat(R_z_cube*R_y_cube*R_x_cube);   

attitude = [attitude(2) attitude(3) attitude(4) attitude(1)];

g = g*R_x*R_y*R_z;

%total moment of inertia of whole system
teta_0_matrix = [3.34*10E-3           0            0;
                          0  3.34*10E-3           0;    
                          0           0   3.34*10E-3];
                
%moment of inertia of reaction wheel                
teta_w_matrix = [0.57*10E-3           0            0;
                          0  0.57*10E-3            0;
                          0           0   0.57*10E-3];

hat_teta_0_matrix = teta_0_matrix - teta_w_matrix;
                      
l=15;                    %cube edge length in centimeters

m_tot_real = 1.2;        %total mass in kg

global m_tot_nominal;

m_tot_nominal = 1.2*[(l/2) (l/2) (l/2)]';
                                
m_vector_real = m_tot_real*[(l/2),(l/2),l/2]; %center of gravity vector in body frame

m_vector_real = [m_vector_real(1), m_vector_real(2), m_vector_real(3)];

%-------------------------------PREPARING DATA FOR ODE INTEGRATION----------------------------------------------------

%Controller's Parameters
alfa = 15;
beta = 18;
gamma = 12;
delta = 10E-5;

paramCell = {hat_teta_0_matrix,teta_w_matrix,m_vector_real,alfa,beta,gamma,delta};

p0 = [g,p_omega_h,p_omega_w,attitude];                                                             %Initial condition

reltol = 1.0e-9;                                                                
abstol = 1.0e-9;

options = odeset('Reltol',reltol,'Abstol',abstol);

%---------------------------------------------ODE INTEGRATION----------------------------------------------------------- 

[t,p] = ode45 ( @(t,p) cubli_3D(t,p,paramCell),tspan,p0,options);

%-----------------------------------------------------------------------------------------------------------------------


%------------------------------------------RESULTS SHOWN WITH GRAPHS----------------------------------------------------

g_equilibrium = ones(1,length(p(:,1)))*-5.66;
figure; plot(t,[p(:,1) p(:,2) p(:,3) g_equilibrium']);grid;
title('Gravity Vector Body Frame');
legend('G_x','G_y','G_z','G_{equilibrium}');
if print_act==1
    print('G_vector'+string(test),'-dpng'); 
end


angles = quaternionState2Angle(p);
figure; plot(t,angles); grid;
title('Angles (deg)');
legend('Roll','Pitch','Yaw');
if print_act==1
    print('Angles'+string(test),'-dpng'); 
end



figure; plot(t,[p(:,7) p(:,8) p(:,9)]); grid;
title('Wheels Angular Momentum');
legend('W_x','W_y','W_x');
if print_act==1
    print('Wheels_AM'+string(test),'-dpng');
end

torque_x=zeros(1,length(tspan));
torque_y=zeros(1,length(tspan));
torque_z=zeros(1,length(tspan));

for i=1 : length(p(:,7))-1
    torque_x(i)=(p(i+1,7)-p(i,7))/sampling_time;
    torque_y(i)=(p(i+1,8)-p(i,8))/sampling_time;
    torque_z(i)=(p(i+1,9)-p(i,9))/sampling_time;
    
end

torque_x(1)=[];
torque_y(1)=[];
torque_z(1)=[];
td = t; td(1) = [];
figure; plot(td, [torque_x' torque_y' torque_z']); grid;
title('Torques [N/m]');
legend('T_x','T_y','T_z');
if print_act==1
print('Torques'+string(test),'-dpng');
end

figure; plot(t,[30*p(:,7)/(pi*0.57*10E-3),30*p(:,8)/(pi*0.57*10E-3),30*p(:,9)/(pi*0.57*10E-3)]); grid;
title('Velocities Reaction Wheels (RPM)');
legend('\omega_x','\omega_y','\omega_z');
if print_act==1
print('reactionVel'+string(test),'-dpng');
end

%---------------------------------------------------ANIMATION------------------------------------------------------------

%Animation()

%----------------------------------ODE cubli_3D--------------------------------

function dpdt = cubli_3D(t,p,paramCell)

global m_tot_nominal
dpdt = zeros(13,1);

%-----------------------------Parameters----------------------------------
teta_0_matrix = paramCell{1};
m_tot_vector = paramCell{3};

%Controller's Parameters
alfa = paramCell{4};
beta = paramCell{5};
gamma = paramCell{6};
delta = paramCell{7};


omega_h =(teta_0_matrix)\(p(4:6)-p(7:9));
omega_h_skew = skew3x3(omega_h);
m_skew  = skew3x3(m_tot_vector);
m_skew_nominal = skew3x3(m_tot_nominal);
g_skew = skew3x3(p(1:3));


%----------------------------NON LINEAR CONTROLLER -----------------------------



%Controller
p_omega_g = p(4:6)'*p(1:3)* p(1:3)/(norm(p(1:3))).^2;

p_omega_h = [p(4),p(5),p(6)];

p_perp_omega = p(4:6) - p_omega_g ;

k1 = (1 + beta * gamma + delta)*eye(3) + alfa*teta_0_matrix;

k2 = alfa*teta_0_matrix*skew3x3(p_perp_omega) + beta * m_skew_nominal * g_skew + skew3x3(p_omega_h);

k3 = gamma*(eye(3)+alfa*teta_0_matrix*(eye(3)-(p(1:3)*p(1:3)')/((norm(p(1:3))).^2)));

K4 = gamma * eye(3);

T = k1*m_skew_nominal*p(1:3) + k2*omega_h + k3*p(4:6) - K4*p(7:9);



%------------------------------System Dynamic------------------------------

dpdt(1:3) =  -omega_h_skew*p(1:3);                          % gravity vector

dpdt(4:6) =  -omega_h_skew*p(4:6) + m_skew*p(1:3);          % momentum vertex

dpdt(7:9) = T;                                              % momentum wheels

q = [p(10),p(11),p(12),p(13)]';                             % actual Quaternions  

Q = skew4x4(omega_h);

dpdt(10:13) = 1/2 * Q * q;                                  % Quaternion's Attitude


end

