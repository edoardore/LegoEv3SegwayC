#pragma config(Sensor, S4,     Gyro,           sensorEV3_Gyro, modeEV3Gyro_RateAndAngle)
#pragma config(Motor,  motorA,          rightMotor,    tmotorEV3_Large, openLoop, driveRight, encoder)
#pragma config(Motor,  motorD,          leftMotor,     tmotorEV3_Large, openLoop, driveLeft, encoder)

float calibrate_gyro();

task main(){
	eraseDisplay();
	displayCenteredBigTextLine(2, "Calibrate Zero");
	float offset=calibrate_gyro();
	writeDebugStream("offset is: %f\n", offset);
	resetGyro(Gyro);
	sleep(4000);
	playSound(soundBeepBeep);
	eraseDisplay();
	displayCenteredBigTextLine(2, "Put on rack");
	sleep(5000);
	resetMotorEncoder(rightMotor);
	resetMotorEncoder(leftMotor);
	eraseDisplay();
	displayCenteredBigTextLine(2, "Balancing");
	float u;
	float ThetaList[2];
	ThetaList[0]=0;
	ThetaList[1]=0;
	int count=0;
	float t=0.01;
	float Theta;
	float Psi=0;
	float Theta_dot=0;
	float Psi_dot;
	float Theta_int=0;
	float angleDx;
	float angleSx;
	float ThetaM;
	float ThetaRad;
	float PsiRad;
	float Psi_dotRad;
	float Theta_dotRad;
	float Theta_intRad;
	time1[T1]=0;
	while(true){
		if(time1[T1]>10000 && Psi<= 0){
			moveMotorTarget(motorB, -getMotorEncoder(motorB), -100);
			setMotorSpeed(rightMotor, 0);
			setMotorSpeed(leftMotor, 0);
			sleep(1000);
			break;
		}
		//datalogDataGroupStart();
		Psi_dot=-getGyroRate(Gyro)-offset;
		//datalogAddValue(0, Psi_dot);
		//writeDebugStream("Psi_dot is: %f\n", Psi_dot);
		Psi=-getGyroDegrees(Gyro);
		//datalogAddValue(1, Psi);
		//writeDebugStream("Psi is: %f\n", Psi);
		angleDx=getMotorEncoder(rightMotor);
		angleSx=getMotorEncoder(leftMotor);
		ThetaM=(angleDx+angleSx)/2;
		//writeDebugStream("ThetaM is: %f\n", ThetaM);
		Theta=ThetaM+Psi;
		//datalogAddValue(2, Theta);
		//writeDebugStream("Theta is: %f\n", Theta);
		ThetaList[0]=ThetaList[1];
		ThetaList[1]=Theta;
		if (count>1){
			Theta_dot=(ThetaList[1]-ThetaList[0])/t;
			//writeDebugStream("Theta_dot is: %f\n", Theta_dot);
			Theta_int=Theta_int+(Theta*(t));
			//writeDebugStream("Theta_int is: %f\n", Theta_int);
		}
		//datalogAddValue(3, Theta_int);
		//datalogAddValue(4, Theta_dot);
		//datalogDataGroupEnd();
		//writeDebugStream("------------------------------------\n");
		count=count+1;
		//writeDebugStream("count is %d ", count);
		ThetaRad=Theta*PI/180;
		PsiRad=Psi*PI/180;
		Theta_dotRad=Theta_dot*PI/180;
		Psi_dotRad=Psi_dot*PI/180;
		Theta_intRad=Theta_int*PI/180;
		u=(-0.8559*ThetaRad)+(-44.7896*PsiRad)+(-0.9936*Theta_dotRad)+(-4.6061*Psi_dotRad)+(-0.500*Theta_intRad);
		u=-u*100/getBatteryVoltage();
		//writeDebugStream("u is: %f\n", u);
		setMotorSpeed(rightMotor, u);
		setMotorSpeed(leftMotor, u);
		if (count==15){
			//rackUp
			moveMotorTarget(motorB, 100, 10);
		}
		sleep(10);
	}
}


float calibrate_gyro(){
	float angle=0;
	float rate=0;
	time1[T1]=0;
	while(time1[T1]<5000){
		angle=angle+(-getGyroRate(Gyro)*0.01);
		sleep(10);
	}
	rate=angle/5;
	return rate;
}