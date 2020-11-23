% Get Jacobian
syms theta1 theta2 theta3 theta4
% theta1 = -0.5;
% theta2 = -0.5;
% theta3 = -0.5;
% theta4 = -0.5;

T1 = [sin(theta1) 0 cos(theta1) 0 ;
   -cos(theta1) 0 sin(theta1) 0 ;
   0 -1 0 2.5;
   0 0 0 1];
                

T2 = [sin(theta2) 0 -cos(theta2) 0;
   -cos(theta2) 0 -sin(theta2) 0;
   0 1 0 0;
   0 0 0 1];
                

T3 = [-cos(theta3) 0 -sin(theta3) 3.5*cos(theta3);
   -sin(theta3) 0 cos(theta3)  3.5*sin(theta3);
   0 1 0 0;
   0 0 0 1];
                

T4 = [cos(theta4) -sin(theta4) 0 -3*cos(theta4);
   sin(theta4) cos(theta4) 0 -3*sin(theta4);
   0 0 1 0;
   0 0 0 1];
                
%T1_T2 =T1 * T2
T_final = ((T1 * T2) * T3) * T4;

angles = [theta1; theta2; theta3; theta4];
end_position = T_final(1:3,end) 

J_k = jacobian (end_position, [theta1 theta2 theta3 theta4])

