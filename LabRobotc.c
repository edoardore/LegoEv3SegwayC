float calibrate_gyro(int loops);
float gyroRate(int readings);
float control(float Theta, float Psi, float Theta_dot, float Psi_dot, float Theta_int);

task main()
{
	float offset=calibrate_gyro(5)*PI/180;
	wait1Msec(1000);
	float ThetaTimeList[2];
	float Psi_dotTimeList[2];
	float u;
	float ThetaList[2];
	ThetaList[0]=0;
	ThetaList[1]=0;
	float Psi_dotList[2];
	Psi_dotList[0]=0;
	Psi_dotList[1]=0;
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
		if((t-millisecondsStart)>10000){
			moveMotorTarget(motorB, 110, -100);
			count++;
			break;
		}
		if(count==0){
			Theta=8.7*PI/180;
			Psi=8.7*PI/180;
			Theta_dot=0;
			Psi_dot=0;
			Theta_int=0;
			ThetaTimeList[1]=time1[T1];
			Psi_dotTimeList[1]=time1[T1];
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
		if (count==100){
			//rackUp
			moveMotorTarget(motorB, 110, 100);

		}
		ThetaTimeList[0]=ThetaTimeList[1];
		Psi_dotTimeList[0]=Psi_dotTimeList[1];
		//sensor
		int i=0;
		angleDx=getMotorEncoder(motorA);
		angleSx=getMotorEncoder(motorD);

		ThetaTimeList[1]=time1[T1];
		ThetaM=(angleDx+angleSx)/2;
		ThetaM=ThetaM*PI/180;
		Psi_dot=getGyroRate(S4);
		Psi_dotTimeList[1]=time1[T1];
		Psi_dot=Psi_dot*PI/180;
		Psi_dot=Psi_dot-offset;
		Psi_dotList[0]=Psi_dotList[1];
		Psi_dotList[1]=Psi_dot;
		//integrate_Psi+=Psi_dotList[1]*(Psi_dotTimeList[1]-Psi_dotTimeList[0])/1000;
		integrate_Psi+=Psi_dotList[1]*0.0009;
		Psi=integrate_Psi;
		Theta=ThetaM+Psi;
		ThetaList[0]=ThetaList[1];
		ThetaList[1]=Theta;
		//Theta_dot=(ThetaList[1]-ThetaList[0])*1000/(ThetaTimeList[1]-ThetaTimeList[0]);
		Theta_dot=(ThetaList[1]-ThetaList[0])/0.0009;
		//integrate_Theta+=ThetaList[1]*(ThetaTimeList[1]-ThetaTimeList[0])/1000;
		integrate_Theta+=ThetaList[1]*0.0009;
		Theta_int=integrate_Theta;

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
