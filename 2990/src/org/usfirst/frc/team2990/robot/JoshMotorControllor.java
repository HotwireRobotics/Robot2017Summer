package org.usfirst.frc.team2990.robot;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.VictorSP;
public class JoshMotorControllor {
	public VictorSP victor; 
	public Talon talon;
	public float accelValue;
	public float target;
	public boolean usingVictor;
	
	public JoshMotorControllor(int motorpwm, float AcelerationMax, boolean usingVictor)
	{
		this.usingVictor = usingVictor;
		if (usingVictor) {
			victor = new VictorSP(motorpwm);
		} else {
			talon = new Talon(motorpwm);
		}
		
		accelValue = AcelerationMax;
	}
		
	public void UpdateMotor()
	{
		if (victor != null || talon != null) {
			double curr = 0;
			if(usingVictor) {
				curr = victor.get();
			} else {	
				curr = talon.get();
			}
			
			float newValue = Lerp((float)curr,target,accelValue);
			
			float epsilon = 0.001f;
			if (newValue < epsilon && newValue > -epsilon) 
			{
				newValue = 0.0f;
			}
	
			if (usingVictor) {
				victor.set(newValue);
			} else {
				talon.set(newValue);
			}
		}
	}
	
	public float Lerp(float v0, float v1, float t)
	{
		return (v0 + t*(v1-v0));
	}
	
}