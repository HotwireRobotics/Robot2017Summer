package org.usfirst.frc.team2990.robot;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Relay;
import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import static java.lang.Math.sqrt;
import static java.lang.Math.abs;

public class Robot extends IterativeRobot implements PIDOutput {
	//SENSORS
	public AHRS navxDevice;
	public Compressor compressor;

	//Cameras
	CameraServer camera;

	//PID
	PIDController turnController;

	//MOTORS:
	// MotorThree is the top motor, it must move opposite the others
	public JoshMotorControllor leftMotorTop;
	public JoshMotorControllor leftMotorBottom;
	public JoshMotorControllor rightMotorTop;
	public JoshMotorControllor rightMotorBottom;
	public JoshMotorControllor climber;

	//Controllers
	public Joystick driveControllerL;
	public Joystick driveControllerR;
	public Joystick xboxController;

	//Shifting
	public DoubleSolenoid rightShifter;
	public DoubleSolenoid gearShifter;
	public boolean gearShifterReleased;
	public DoubleSolenoid gearDrop;

	//Encoders
	//public Encoder Lencoder;
	public Encoder Rencoder;

	public void robotInit() {
		//Sensors
		{
			navxDevice = new AHRS(SPI.Port.kMXP);
			compressor = new Compressor();
		}

		// Straight Drive PID d=.0002
		{
			turnController = new PIDController(0.05, 0.0, 0.00020, 0, navxDevice, this);
			turnController.setInputRange(-180.0f, 180.0f);
			turnController.setOutputRange(-1.0f, 1.0f);
			turnController.setAbsoluteTolerance(2.0);
			turnController.setContinuous(true);
			turnController.disable();
		}

		// Camera
		{
			camera = CameraServer.getInstance();
			UsbCamera usbCam = camera.startAutomaticCapture(); 
			usbCam.setResolution(250,  210);
			usbCam.setFPS(30);
		}

		//Controllers
		{
			xboxController = new Joystick(2);
			driveControllerR = new Joystick(1);
			driveControllerL = new Joystick(0);	
		}

		//Shifters
		{
			rightShifter = new DoubleSolenoid(2, 3);
			gearShifter = new DoubleSolenoid(0, 1);
			gearDrop = new DoubleSolenoid(4,5);
		}

		//Encoders
		{
			//Lencoder = new Encoder(0, 1);
			Rencoder = new Encoder(2, 3);
		}

		//Motor Controllers
		{
			//float lerpSpeed = 0.5f;
			float lerpSpeed = 0.2f;
			leftMotorTop = new JoshMotorControllor(8, lerpSpeed, false);
			leftMotorBottom = new JoshMotorControllor(9, lerpSpeed, true);
			rightMotorTop = new JoshMotorControllor(6, lerpSpeed, false);
			rightMotorBottom = new JoshMotorControllor(7, lerpSpeed,true);			
			climber = new JoshMotorControllor(5, lerpSpeed, false);			
			//stirer = new Talon(2);
		}

		//Shooter
		{
			float p = 0.2f;
			float i = .00000003f;
			float d = 6.06275f;
			float f = 0.35f;

		}

		SmartDashboard.putBoolean("Show Diagnostics", false);

	}

	public void teleopInit() {
		float lerpSpeed = 0.5f;
		leftMotorTop.accelValue = lerpSpeed;
		leftMotorBottom.accelValue = lerpSpeed;
		rightMotorTop.accelValue = lerpSpeed;
		rightMotorBottom.accelValue = lerpSpeed;			
		climber.accelValue = lerpSpeed;	

		SmartDashboard.putNumber("Four (Agitator)", -1.0f);
		SmartDashboard.putNumber("Three (Injector)", -1.0f);
		gearDrop.set(DoubleSolenoid.Value.kReverse);
		ShiftUp();
	}

	public void teleopPeriodic() {

		if (SmartDashboard.getBoolean("Show Diagnostics")) {
			SmartDashboard.putNumber("Right Encoder:", Rencoder.get());
			SmartDashboard.putNumber("Navx:", navxDevice.getYaw());
		}

		//Update Motors
		{
			UpdateMotors();
			climber.UpdateMotor();
		}

		}
		//Logictechs
		//Driving
		{
			float horJoystick = 0;
			float verJoystick = 0;

			float epsilon = 0.2f;
			float a = TranslateController((float)driveControllerL.getRawAxis(0));
			if (a > epsilon || a < -epsilon) {
				horJoystick = a;
			}
			float b = TranslateController((float)driveControllerL.getRawAxis(1));
			if (b > epsilon || b < -epsilon) {
				verJoystick = b;
			}

			float c = (float) sqrt(abs(a * a) + abs(b * b));

			SetLeftMotors(verJoystick + horJoystick);
			SetRightMotors(-verJoystick + horJoystick);
		}   
		// Shooting driving aligning
		{
			float driveSensitivity = 0.2f;
			if (driveControllerL.getRawButton(2)) {
				ShiftDown();
				SetLeftMotors(driveSensitivity);
				SetRightMotors(-driveSensitivity);
			} else if (driveControllerL.getRawButton(3)) {
				ShiftDown();
				SetLeftMotors(-driveSensitivity);
				SetRightMotors(driveSensitivity);
			} 

			float turnSensitivity = 0.15f;
			if (driveControllerL.getRawButton(4)) {
				ShiftDown();
				SetLeftMotors(-turnSensitivity);
				SetRightMotors(-turnSensitivity);
			} else if (driveControllerL.getRawButton(5)) {
				ShiftDown();
				SetLeftMotors(turnSensitivity);
				SetRightMotors(turnSensitivity);
			}
		}

		//Shifting
		{
			if (driveControllerR.getRawButton(4)) {
				ShiftUp();
			}
			if (driveControllerR.getRawButton(5)) {
				ShiftDown();
			}
		}

		//xBox
		//Gear Gobbler neumatic
		{
			if ((xboxController.getRawButton(1) && gearShifterReleased) || (driveControllerR.getRawButton(1) && gearShifterReleased)) {
				if(gearShifter.get() == DoubleSolenoid.Value.kReverse) {
					gearShifterReleased = false;
					gearShifter.set(DoubleSolenoid.Value.kForward);
				} else {
					gearShifterReleased =  false;
					gearShifter.set(DoubleSolenoid.Value.kReverse);
				}
			}
			if (!xboxController.getRawButton(1) && !driveControllerR.getRawButton(1)) {
				gearShifterReleased = true;
			}
		}
		//Shooting Toggle
		{
			if (xboxController.getRawButton(2) || driveControllerL.getRawButton(8) ) {
				//ShooterToggle(true, SmartDashboard.getNumber("One (shooter)"), SmartDashboard.getNumber("Two (feeder)"), -1, -1.0);
				ShooterToggle(true, 4300, 4000, -1, -1.0);
			} else {
				ShooterToggle(false, 0, 0, 0, 0);
			}
		}
		//Climber
		{
			climber.target = 0.0f;
			if (xboxController.getRawButton(4) || driveControllerL.getRawButton(9)) {
				climber.target = 1;
			}
		}

		//Gear Drop
		{
			if(driveControllerL.getRawButton(1)){
				gearDrop.set(DoubleSolenoid.Value.kForward);
			}else{
				gearDrop.set(DoubleSolenoid.Value.kReverse);
			}
		}
	

	// NOTE the yaw must be reset before using this
	public void DriveStraight(float speed, boolean reverse) {
		float pidError = (float)turnController.get();
		SetLeftMotors((speed * pidError) + speed); //0.6972
		SetRightMotors(((speed) - (speed * pidError)) * -1); //-0.583

		speed = -speed;
		if(reverse){
			speed = -speed;
		}

		LogInfo("STRAIGHT YAW " + navxDevice.getYaw());
	}

	public void ClearRotation() {
		navxDevice.zeroYaw();
		turnController.setSetpoint(0);
	}

	public void ShiftDown() {
		rightShifter.set(DoubleSolenoid.Value.kReverse);
	}

	public void ShiftUp() {
		rightShifter.set(DoubleSolenoid.Value.kForward);
	}

	public float TranslateController(float input) {
		float deadzone = 0.15f;
		if (input > -deadzone && input < deadzone) {
			input = 0.0f;
		}
		float a = 0.7f;
		float output = (a * input * input * input) + (1 - a) * input; 
		return output;
	}

	public void ShooterToggle(boolean toggle, double motorOne, double motorTwo, double motorThree, double motorFour) {
		shooterRPMControl.running = toggle;
		feederRPMControl.running = toggle;

		shooterRPMControl.targetRPM = (float)motorOne;
		feederRPMControl.targetRPM = (float)motorTwo;

		//LogInfo("SHOOTER " + (float)motorOne);
		//LogInfo("FEEDER " + (float)motorTwo);

		if (toggle) {
			agitator.set(motorFour);
			injector.set(motorThree);
			//stirer.set(1.0f);
			//agitator.set(SmartDashboard.getNumber("Four (Agitator)"));
			//injector.set(SmartDashboard.getNumber("Three (Injector)"));
		} else {
			agitator.set(0.0f);
			injector.set(0.0f);
			//stirer.set(0.0f);
		}
	}

	public void SetLeftMotors(float speed){
		leftMotorTop.target = -speed;
		leftMotorBottom.target = speed;
	}

	public void SetRightMotors(float speed) {
		rightMotorTop.target = -speed;
		rightMotorBottom.target = speed;
	}

	public void UpdateMotors() {
		leftMotorTop.UpdateMotor();
		leftMotorBottom.UpdateMotor();
		rightMotorTop.UpdateMotor();
		rightMotorBottom.UpdateMotor();

	}

	public void LogInfo(String info) {
		System.out.println(info + ";    ");
	}

	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub

	}
}