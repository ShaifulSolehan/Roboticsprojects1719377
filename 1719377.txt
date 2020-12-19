%Modelling the Scara robot
L1= Link('d',0,'a',5,'alpha',0); %For initialize DH variable for link 1 
L2= Link('d',0,'a',5,'alpha',pi); %For initialize DH variable for link 2
L3= Link([0,4,0,0,1],'standard'); %For initialize DH variable for Link 3 using the standard initialization because to use primastic function
L3.qlim=[0 3]; %For prismatic function
L4= Link('d',0,'a',0,'alpha',0); %For initialize DH variable for Link 4
Rbt= SerialLink([L1 L2 L3 L4],'name','MYROBOT'); %Function to construct the Scara Robot
q1=deg2rad(30);q2=deg2rad(40);q3=2.34;q4=0; %To initialize the theta 
Rbt.plot([q1 q2 q3 q4],'workspace',[-10 10 -10 10 -10 10]); %To plot the graph of the Scara Robot
Rbt; %To call the graph

%Inverse Kinematic
ThetaInt= [0 0 0 0]; %To initialize the theta(angle) to put in the ikine function
 
T1= transl(1,3,0.3); %To show the last coordinate of the Scara robot
T2= transl(2,-3,-5);
T3= transl(3,5,1.2);

IK1= Rbt.ikine(T1,ThetaInt,[1,1,1,0,0,0]); %The ikine function to get the theta(angle) based on the last coordinate
IK2= Rbt.ikine(T2,IK1,[1,1,1,0,0,0]);
IK3= Rbt.ikine(T3,IK2,[1,1,1,0,0,0]);

%Forward Kinematic
FK1=Rbt.fkine(IK1);  %The fkine function to get the last coordinate based on the theta(angle) 
FK2=Rbt.fkine(IK2);
FK3=Rbt.fkine(IK3);

%To know if the coordinate is equal 
co1=round(FK1(1:3,4),2); an1=round(T1(1:3,4),2);
a= isequal(co1,an1);
co2=round(FK2(1:3,4),2); an2=round(T2(1:3,4),2);
b= isequal(co2,an2);
co3=round(FK3(1:3,4),2); an3=round(T3(1:3,4),2);
c= isequal(co3,an3);

%Animation is from robotdemo 
t=[0:0.07:3]';
pause(0.1);
Anim1=jtraj(ThetaInt,IK1,t); 
Rbt.plot(Anim1,'workspace',[-15 15 -15 15 -10 10]);
pause(0.1);
Anim2=jtraj(IK1,IK2,t);
Rbt.plot(Anim2,'workspace',[-15 15 -15 15 -10 10]);
pause(0.1);
Anim3=jtraj(IK2,IK3,t);
Rbt.plot(Anim3,'workspace',[-15 15 -15 15 -10 10]);
