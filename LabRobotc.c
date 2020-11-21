float calibrate_gyro(int loops);
float gyroRate(int readings);
float control(float Theta, float Psi, float Theta_dot, float Psi_dot, float Theta_int);

task main()
{
	float offset=calibrate_gyro(5)*PI/180;
	wait1Msec(1000);
	float u;
	float ThetaList[2];
	ThetaList[0]=0;
	ThetaList[1]=0;
	int ThetaTimeList[2];
	ThetaTimeList[0]=0;
	ThetaTimeList[1]=0;
	float Psi_dotList[2];
	Psi_dotList[0]=0;
	Psi_dotList[1]=0;
	int Psi_dotTimeList[2];
	Psi_dotTimeList[0]=0;
	Psi_dotTimeList[1]=0;
	int count=0;
	float Theta;
	float Psi;
	float Theta_dot;
	float Psi_dot;
	float Theta_int;
	float integrate_Psi=0;
	float integrate_Theta=0;
	float angleDx;
	float angleSx;
	float ThetaM;
	float t;
	float millisecondsStart=time1[T1];
	while(true){
		t=time1[T1];
		if((t-millisecondsStart)>10000 && Psi>0){
			moveMotorTarget(motorB, -getMotorEncoder(motorB), -100);
			setMotorSpeed(motorA, 0);
			setMotorSpeed(motorD, 0);
			sleep(1000);
			break;
		}
		if(count==0){
			Theta=-8.7*PI/180;
			Psi=-8.7*PI/180;
			Theta_dot=0;
			Psi_dot=0;
			Theta_int=0;
			Psi_dotTimeList[1]=time1[T1];
			ThetaTimeList[1]=time1[T1];
		}
		u=control(Theta, Psi, Theta_dot, Psi_dot, Theta_int);
		count=count+1;
		u=u*100/9;
		if (u>100)
			u=100;
		if (u<-100)
			u=-100;
		setMotorSpeed(motorA, u);
		setMotorSpeed(motorD, u);
		if (count==150){
			//rackUp
			moveMotorTarget(motorB, 110, 80);
		}
		if(count%250==0){
			angleDx=getMotorEncoder(motorA);
			angleSx=getMotorEncoder(motorB);
			ThetaM=(angleDx+angleSx)/2;
			ThetaM=ThetaM*PI/180;
			//writeDebugStream("ThetaM is: %f\n", ThetaM);
			Psi_dot=getGyroRate(S4);
			Psi_dot=Psi_dot*PI/180;
			Psi_dot=Psi_dot-offset;
			//writeDebugStream("Psi_dot is: %f\n", Psi_dot);
			Psi_dotList[0]=Psi_dotList[1];
			Psi_dotList[1]=Psi_dot;
			Psi_dotTimeList[0]=Psi_dotTimeList[1];
			Psi_dotTimeList[1]=time1[T1];
			integrate_Psi+=Psi_dotList[1]*(Psi_dotTimeList[1]-Psi_dotTimeList[0]);
			Psi=integrate_Psi;
			//writeDebugStream("Psi is: %f\n", Psi);
			Theta=ThetaM+Psi;
			//writeDebugStream("Theta is: %f\n", Theta);
			ThetaList[0]=ThetaList[1];
			ThetaList[1]=Theta;
			ThetaTimeList[0]=ThetaTimeList[1];
			ThetaTimeList[1]=time1[T1];
			Theta_dot=(ThetaList[1]-ThetaList[0])/(ThetaTimeList[1]-ThetaTimeList[0]);
			//writeDebugStream("Theta_dot is: %f\n", Theta_dot);
			integrate_Theta+=ThetaList[1]*(ThetaTimeList[1]-ThetaTimeList[0]);
			Theta_int=integrate_Theta;
			//writeDebugStream("Theta_int is: %f\n", Theta_int);
			//writeDebugStream("------------------------------------\n");
		}
	}
}

float gyroRate(int readings){
	float mean=0;
	int i=0;
	for(i=0; i<readings; i++)
		mean=mean+getGyroRate(S4);
	mean=mean/readings;
	return mean;
}


float calibrate_gyro(int loops){
	float mean=0;
	int i=0;
	for(i=0; i<loops; i++)
		mean=mean+gyroRate(5);
	mean=mean/loops;
	return mean;
}

float control(float Theta, float Psi, float Theta_dot, float Psi_dot,float Theta_int){
	float u=(-0.8559*Theta)+(-44.7896*Psi)+(-0.9936*Theta_dot)+(-4.6061*Psi_dot)+(-0.500*Theta_int);
	return u;
}
