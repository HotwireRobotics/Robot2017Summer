package org.usfirst.frc.team2990.robot;
import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RPMControl {
	public Timer encoderTimer;
	public float currentSpeed = 0.2f;
	public float encoderZero;
	public CANTalon talon;
	public float targetRPM;
	public String name;
	
	public boolean running;
	
	public boolean debug;
	
	public float P = 0.21f;
	public float I = 0.0f;
	public float D = 5.0f;
	public float F = 0.35f;

	public RPMControl(String name, int talonNumber, boolean debug,
				float P, float I, float D, float F, float targetRPM) {
		
		talon = new CANTalon(talonNumber);
		
		running = false;
		
		this.P = P;
		this.I = I;
		this.D = D;
		this.F = F;
		this.targetRPM = targetRPM;
		
		talon.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		talon.configEncoderCodesPerRev(360);
		talon.reverseSensor(false);
		talon.setProfile(0);
		talon.setP(P);
		talon.setI(I);
		talon.setD(D);
		talon.setF(F);
		
		talon.reverseOutput(true);
		
		talon.configNominalOutputVoltage(0.0f, 0.0f);
		talon.configPeakOutputVoltage(0.0f, -12.0f);
		
		talon.setPosition(0);
		
		talon.changeControlMode(CANTalon.TalonControlMode.Speed);
		
		this.debug = debug;

		encoderTimer = new Timer();
		encoderTimer.start();
		encoderZero = talon.getEncPosition();

		this.name = name;
		

		SmartDashboard.putNumber(name, targetRPM);
		SmartDashboard.putNumber(name + "P", P);
		SmartDashboard.putNumber(name + "I", I);
		SmartDashboard.putNumber(name + "D", D);
		SmartDashboard.putNumber(name + "F", F);		
		//talon.set(0.2f);
	}

	@SuppressWarnings("deprecation")
	public void UpdateRPMControl() {
		
		//targetRPM = (float) SmartDashboard.getNumber(name);
		P = (float) SmartDashboard.getNumber(name + "P");
		I = (float) SmartDashboard.getNumber(name + "I");
		D = (float) SmartDashboard.getNumber(name + "D");
		F = (float) SmartDashboard.getNumber(name + "F");
		
		talon.setP(P);
		talon.setI(I);
		talon.setD(D);
		talon.setF(F);
		
		if (running) {
			talon.set(targetRPM);
		} else {
			talon.set(0.0f);
		}
		
		// gather and disaplay the current rpm
		float encoderPerRotation = 4080;

		if (encoderTimer.get() >= 0.1) {
			float encoderDelta = talon.getEncPosition() - encoderZero;
			//System.out.println(talon.getAnalogInVelocity());
			float RPM = (encoderDelta * 60 * 10) / encoderPerRotation;
			
			if (RPM != 0 && debug) {		
				SmartDashboard.putNumber(name + "RPM", RPM);
				System.out.println(name + "RPM SPEED " + RPM + ";");
			}
			
			encoderTimer.reset();
			encoderTimer.start();
			encoderZero = talon.getEncPosition();
		}
	}
}
