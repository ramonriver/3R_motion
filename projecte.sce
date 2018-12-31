//-------------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------------//
//                          3R motion with Scilab                                      //
//-------------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------------//


//Scilab version: "scilab-6.0.1"



clc; //clear comand window
clear; //kill variables




//------------------------------------//
//       PARAMETERS MODULE            //
//------------------------------------//

//Functional parameters
P_start_endEffector=[0.9;-0.2]; //inicial position of end-effector (m)
P_goal_endEffector=[0.9;-0.7]; //final position of end-effector (m)
speed_endEffector=[0;-0.1]; //speed of end-effector (m/s)
deltaTime=0.2; //time between steps (s)
Number_products_second=1; //number of products per second that get the end-effector

//Robot statics
L1=0.62; //length of L1 (m)
L2=0.57; //length of L2 (m)
g1=0.1; //length of g1 of end-effector (m)
g2=0.2; //length of g2 of end-effector (m)
g3=0.3; //length of g3 of end-effector (m)
M1=4; //weight of L1 (kg)
M2=3; //weight of L2 (kg)
Mproduct=1 //weight of a product (kg)






//------------------------------------//
//       KINEMATICS MODULE            //
//------------------------------------//

//Jacobian
function [J]=jacobian(J1,J2,J3)
    //S=[Y;-X;1] ; 1 because they are revolute joint
    S1=[J1(2);-J1(1);1];
    S2=[J2(2);-J2(1);1];
    S3=[J3(2);-J3(1);1];
    J=[S1 S2 S3];
endfunction


//Inverse kinematics
//G: joint speeds
//T: Twist
//J: Jacobian
function [G]=inversekin(T,J)
    Jinverted=inv(J);
    G=Jinverted*T;
endfunction


//Forward kinematics
//T: Twist
//G: joint speeds
//J: Jacobian
function [T]=forwardkin(G,J)
    T=J*G;
endfunction


//Forward Pose
//P: (x,y) end-effector situation in reference frame (m)
//alpha: end-effector angle from x axis of reference frame (rad)
//a1: joint 1 angle from x axis of reference frame (rad)
//a2: joint 2 angle from x axis of previous link (rad)
//a3: joint 3 angle from x axis of previous link (rad)
function [P,alpha]=forwardPos(a1,a2,a3)
    //rotational matrix
    R1=[cos(a1) (-sin(a1));sin(a1) cos(a1)];
    R2=[cos(a1+a2) (-sin(a1+a2));sin(a1+a2) cos(a1+a2)];
    R3=[cos(a1+a2+a3) (-sin(a1+a2+a3));sin(a1+a2+a3) cos(a1+a2+a3)];
    
    //Pose end-effector
    P=R1*j1+R2*j2+R3*j3;
    alpha=a1+a2+a3;
endfunction


//Inverse Pose
//theta1: joint 1 angle from x axis of reference frame (rad)
//theta2: joint 2 angle from x axis of previous link (rad)
//theta3: joint 3 angle from x axis of previous link (rad) 
//P: (x,y) end-effector situation in reference frame (m)
//alpha: end-effector angle from x axis of reference frame (rad)
function [theta1,theta2,theta3]=inversePos(P,alpha)
    //length of the links
    modulej1=sqrt((j1(1)*j1(1))+(j1(2)*j1(2)));
    modulej2=sqrt((j2(1)*j2(1))+(j2(2)*j2(2)));
    modulej3=sqrt((j3(1)*j3(1))+(j3(2)*j3(2)));

    //x=modulej1*cos(theta1)+modulej2*cos(theta1+theta2)+modulej3*cos(theta1+theta2+theta3)
    //y=modulej1*sin(theta1)+modulej2*sin(theta1+theta2)+modulej3*sin(theta1+theta2+theta3)
    //alpha=theta1+theta2+theta3
    //theta2=arccos(fu/d)
    //d=2*modulej1*modulej2
    //fu=(x-modulej3*cos(alpha))²+(y-modulej3*sin(alpha))²-modulej1²-modulej2²
    d=2*modulej1*modulej2;
    fu=(P(1)-(modulej3*cos(alpha)))*(P(1)-(modulej3*cos(alpha)))+(P(2)-modulej3*sin(alpha))*(P(2)-modulej3*sin(alpha))-(modulej1*modulej1)-(modulej2*modulej2);
    theta2=acos(fu / d);

    //To solve "theta1":
    //A*cos(theta1)-B*sin(theta1)=E
    //B*cos(theta1)+A*sin(theta1)=F
    //x=[cos(theta1);sin(theta1)]
    //m*x=v
    A=modulej1+(modulej2*cos(theta2));
    B=modulej2*sin(theta2);
    E=P(1)-modulej3*cos(alpha);
    F=P(2)-modulej3*sin(alpha);
    m=[A (-B);B A];
    m_inverted=inv(m);
    v=[E;F];
    solution=m_inverted*v;

    theta1=-acos(solution(1)); //(-) for "elbow" down, below x axis

    //alpha=theta1+theta2+theta3
    theta3=alpha-theta1-theta2;
endfunction







//------------------------------------//
//           MAIN MODULE              //
//------------------------------------//


//Inicialization
j1=[L1;0];
j2=[L2;0];
j3=[g1;0];
alpha_j3=0;
//Position of last link
P_start=[P_start_endEffector(1)-g3;P_start_endEffector(2)+g2];
P_goal=[P_goal_endEffector(1)-g3;P_goal_endEffector(2)+g2];
//define end-effector goal position and time
P_next=P_start; //firstly, the position is the start position
Time_current=0; //current time in the process


plot_speed=scf(2); //creates figure with id==2 for joints speed and make it the current one


//joints positions lists
t=list(); //list for x values; position joint 1
x=list(); //list for y values; position joint 1
u=list(); //list for x values; position joint 2
y=list(); //list for y values; position joint 2
v=list(); //list for x values; position joint 3
z=list(); //list for y values; position joint 3
//end-effector position list
g1_x=list();
g1_y=list();
g2_x=list();
g2_y=list();
g3_x=list();
g3_y=list();


//main loop
//it keeps inside the loop while we do not reach the goal position
while (P_next(2)>P_goal(2))
    //determine joints angles
    [theta1,theta2,theta3]=inversePos(P_next,alpha_j3);
    
    //position of the joints
    Joint1=[0;0]; //first joint in reference frame
    Joint2=[cos(theta1)*L1;sin(theta1)*L1]; //second joint position
    Joint3=[Joint2(1)+cos(theta1+theta2)*L2;Joint2(2)+sin(theta1+theta2)*L2]; //third joint position
    Jac=jacobian(Joint1,Joint2,Joint3);
    
    //list of data for robot motion animation
    t($+1)=Joint1(1);
    x($+1)=Joint1(2);
    u($+1)=Joint2(1);
    y($+1)=Joint2(2);
    v($+1)=Joint3(1);
    z($+1)=Joint3(2);
    g1_x($+1)=Joint3(1)+g1;
    g1_y($+1)=Joint3(2);
    g2_x($+1)=Joint3(1)+g1;
    g2_y($+1)=Joint3(2)-g2;
    g3_x($+1)=Joint3(1)+g1+g3;
    g3_y($+1)=Joint3(2)-g2;
    
    //joints speeds matrix
    T=[speed_endEffector(1);speed_endEffector(2);0]; //Twist for goal movement
    G=inversekin(T,Jac);
    
    //show joints speeds graphically
    scf(plot_speed); //set joints speeds plot as the current one
    plot2d(Time_current,[G(1) G(2) G(3)],[-1,-12,-3],leg="Joint1@Joint2@Joint3");
    title('JOINTS SPEEDS','color','red','fontsize', 4);
    xlabel("Time (s)", "fontsize", 2,"color", "blue");
    ylabel("Speed (rad/s)", "fontsize", 2, "color", "blue");
    
    //determine new position and time
    P_next=[P_next(1);P_next(2)+speed_endEffector(2)*deltaTime];
    Time_current=Time_current+deltaTime;
    
end




//------------------------------------//
//     ROBOT MOTION PLOT MODULE       //
//------------------------------------//

plot_animation=scf(3); //creates figure with id==3 for robot motion and make it the current one

// Draw initial figure
//figure(3);

//plot joint 1 position
plot2d(t(1),x(1),-3);
h_compound = gce();
h_compound.children.mark_size = 5;
h_compound.children.mark_background = 2;
//plot joint 2 position
plot2d(u(1),y(1),-3);
h_compound = gce();
h_compound.children.mark_size = 5;
h_compound.children.mark_background = 2;
//plot joint 3 position
plot2d(v(1),z(1),-3);
h_compound = gce();
h_compound.children.mark_size = 5;
h_compound.children.mark_background = 2;
//plot end-effector position
plot2d(g1_x(1),g1_y(1),-3);
h_compound = gce();
h_compound.children.mark_size = 1;
h_compound.children.mark_background = 3;
plot2d(g2_x(1),g2_y(1),-3);
h_compound = gce();
h_compound.children.mark_size = 1;
h_compound.children.mark_background = 3;
plot2d(g3_x(1),g3_y(1),-3);
h_compound = gce();
h_compound.children.mark_size = 1;
h_compound.children.mark_background = 3;
h_axes = gca();
h_axes.data_bounds = [-1,-1;1,1];
title('ROBOT MOTION','color','red','fontsize', 4);
xlabel("X positon (m)", "fontsize", 2,"color", "blue");
ylabel("Y position (m)", "fontsize", 2, "color", "blue");


// Animation Loop
i = 1;
while i<=length(x)
    drawlater();
    //plot joint 1 position
    plot2d(t(i),x(i),-3);
    h_compound = gce();
    h_compound.children.mark_size = 5;
    h_compound.children.mark_background = 2;
    
    //plot joint 2 position
    plot2d(u(i),y(i),-3);
    h_compound = gce();
    h_compound.children.mark_size = 5;
    h_compound.children.mark_background = 2;

    //plot joint 3 position
    plot2d(v(i),z(i),-3);
    h_compound = gce();
    h_compound.children.mark_size = 5;
    h_compound.children.mark_background = 2;
    
    //plot end-effector position
    plot2d(g1_x(i),g1_y(i),-3);
    h_compound = gce();
    h_compound.children.mark_size = 1;
    h_compound.children.mark_background = 3;
    
    plot2d(g2_x(i),g2_y(i),-3);
    h_compound = gce();
    h_compound.children.mark_size = 1;
    h_compound.children.mark_background = 3;
    
    plot2d(g3_x(i),g3_y(i),-3);
    h_compound = gce();
    h_compound.children.mark_size = 1;
    h_compound.children.mark_background = 3;
    
    drawnow();
    
    i = i+1;
end
