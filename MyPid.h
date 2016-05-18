#ifndef PID_H
#define PID_H
	class MyPID{
		int outMax;
		int outMin;
		float lastInput;
		double ITerm;
	public:
		double kp;
		double ki;
		double kd;
		double Setpoint;
		MyPID(){
			outMax = 255;
			outMin = -255;
			lastInput = 0;
			ITerm =0;
			kp = 1;
			ki = 1;
			kd = 1;
			Setpoint = 0;
		}	
		int Compute(double input)
		{
			  double error = Setpoint - input;
			  ITerm+= (ki * error);
			  if(ITerm > outMax) ITerm= outMax;
			  else if(ITerm < outMin) ITerm= outMin;
			  double dInput = (input - lastInput);
		 
			  // Compute PID Output
			  double output = kp * error + ITerm + kd * dInput;
			  
			  if(output > outMax) output = outMax;
			  else if(output < outMin) output = outMin;
			  
			  // Remember some variables for next time
			  lastInput = input;
			  return (int)output;
		};
		void SetSetpoint(double d){
			Setpoint = d; 
		};
		double GetSetPoint(){
			return Setpoint;

		};
	};
#endif
