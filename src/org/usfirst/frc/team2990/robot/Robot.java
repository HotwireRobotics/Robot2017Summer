package org.usfirst.frc.team2990.robot;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;
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


public class Robot extends IterativeRobot implements PIDOutput {
	//SENSORS
	public AnalogInput ultrasonic;
	public AHRS navxDevice;
	public Compressor compressor;
	//Cameras
	CameraServer camera;
	//public Servo cameraServo;

	//PID
	boolean rotateToAngle;
	PIDController turnController;

	//MOTORS:
	// MotorThree is the top motor, it must move opposite the others
	public JoshMotorControllor leftMotorTop;
	public JoshMotorControllor leftMotorBottom;
	public JoshMotorControllor rightMotorTop;
	public JoshMotorControllor rightMotorBottom;
	public Talon agitator;
	public JoshMotorControllor climber;

	//Controllers
	public Joystick driveControllerL;
	public Joystick driveControllerR;
	public Joystick xboxController;

	//Shifting
	public DoubleSolenoid leftShifter;
	public DoubleSolenoid rightShifter;
	public DoubleSolenoid gearShifter;
	public boolean gearShifterReleased;

	//Encoders
	public Encoder Lencoder;
	public Encoder Rencoder;

	//Timers
	public Timer autoPause;
	public Timer shooterPause;
	public double pauseTime = 0.1;

	// Shooter Stuff
	public RPMControl feederRPMControl;
	public RPMControl shooterRPMControl;
	public CANTalon injector;

	//Auto enums
	// Middle Peg
	public enum AutoStraightState {
		WaitingForNavX, PlaceGear, End
	}
	// Right Peg 
	public enum AutoRightState {
		WaitingForNavX, Forward, Rotate, Approach, End
	}
	// Left Peg
	public enum AutoLeftState {
		WaitingForNavX, Forward, Rotate, Approach, End
	}
	// Shooting, then going near left peg
	public enum AutoShootingState {
		Pause, Forward, Turn, Fire, TurnTowardTower, Approach, End
	} 
	public enum AutoHopperState {
		Pause, Forward, TurnTowardHopper, Approach, BackUp, TurnTowardBoiler, Fire, End
	}
	public AutoStraightState autoStraightState;
	public AutoRightState autoRightState;
	public AutoLeftState autoLeftState;
	public AutoShootingState autoShootingState;
	public AutoHopperState autoHopperState;
	// Which autonomous program we're using. From the perspective of the drivers.
	public enum AutonomousUsingState {
		PegOnlyMiddle, PegOnlySide, ShootingOnly, None, NearShootOnly, FarShootOnly, ShootingBlue, ShootingRed, HopperRed, HopperBlue
	}
	public enum AutoSideChoice {
		Red, Blue, NearShootOnly, FarShootOnly
	}
	public enum AutoPegChoice {
		Left, Right, Middle
	}
	public AutonomousUsingState autonomousUsingState = AutonomousUsingState.None;

	public SendableChooser autoSideSelection;
	public AutoSideChoice sideChoice;
	public SendableChooser autoPegSelection;
	public AutoPegChoice pegChoice;
	public Relay relay;
	public void robotInit() {
		relay = new Relay(0);
		//Sensors
		{
			navxDevice = new AHRS(SPI.Port.kMXP);
			compressor = new Compressor();
			ultrasonic = new AnalogInput(2);
		}

		// Straight Drive PID
		{
			turnController = new PIDController(0.2, 0, 0.01, 0, navxDevice, this);
			turnController.setInputRange(-180.0f, 180.0f);
			turnController.setOutputRange(-1.0f, 1.0f);
			turnController.setAbsoluteTolerance(2.0);
			turnController.setContinuous(true);
		}

		// Camera
		{
			//cameraServo = new Servo(4);
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
			leftShifter = new DoubleSolenoid(0, 1);
			rightShifter = new DoubleSolenoid(2, 3);
			gearShifter = new DoubleSolenoid(4, 5);
		}

		//Encoders
		{
			Lencoder = new Encoder(0, 1);
			Rencoder = new Encoder(2, 3);
		}

		//Motor Controllers
		{
			float lerpSpeed = 0.5f;
			leftMotorTop = new JoshMotorControllor(8, lerpSpeed, false);
			leftMotorBottom = new JoshMotorControllor(9, lerpSpeed, true);
			rightMotorTop = new JoshMotorControllor(6, lerpSpeed, false);
			rightMotorBottom = new JoshMotorControllor(7, lerpSpeed,true);			
			climber = new JoshMotorControllor(5, lerpSpeed, false);			
			agitator = new Talon(4);
		}

		//Shooter
		{
			float p = 0.2f;
			float i = .00000003f;
			float d = 6.06275f;
			float f = 0.35f;
			feederRPMControl = new RPMControl("Two (feeder)", 5, true, p, i, d, f, 4000);
			shooterRPMControl = new RPMControl("One (shooter)", 2, true, p, i, d, f, 4300); 
			injector = new CANTalon(9);

			agitator.set(0.0f);
			injector.set(-0.0f);
		}

		SmartDashboard.putNumber("Pause Timer", pauseTime);

		// These are from the perspective of the driver!
		{
			autoSideSelection = new SendableChooser();
			autoSideSelection.addDefault("Center Peg", AutonomousUsingState.PegOnlyMiddle);
			autoSideSelection.addObject("Right Peg", AutonomousUsingState.PegOnlySide);
			autoSideSelection.addObject("FarShootOnly", AutonomousUsingState.FarShootOnly);
			autoSideSelection.addObject("NearShootOnly", AutonomousUsingState.NearShootOnly);
			autoSideSelection.addObject("ShootBlue", AutonomousUsingState.ShootingBlue);
			autoSideSelection.addObject("ShootRed", AutonomousUsingState.ShootingRed);
			autoSideSelection.addObject("HopperBlue", AutonomousUsingState.HopperBlue);
			autoSideSelection.addObject("HopperRed", AutonomousUsingState.HopperRed);
			SmartDashboard.putData("Autonomous Side SELECT", autoSideSelection);

			autoPegSelection = new SendableChooser();
			autoPegSelection.addDefault("Left", AutoPegChoice.Left);
			autoPegSelection.addObject("Middle", AutoPegChoice.Middle);
			autoPegSelection.addObject("Right", AutoPegChoice.Right);
			SmartDashboard.putData("Autonomous Peg Selection", autoPegSelection);
		}
	}

	public void autonomousInit() {
		LogInfo("AUTO INIT");

		//Initializations
		
		autoPause = new Timer();
		//State inits
		autoStraightState = AutoStraightState.WaitingForNavX;
		autoRightState = AutoRightState.WaitingForNavX;
		autoLeftState = AutoLeftState.WaitingForNavX;
		autoShootingState = AutoShootingState.Pause;
		autoHopperState = AutoHopperState.Pause;
		//Shifter inits
		ShiftUp();
		gearShifter.set(DoubleSolenoid.Value.kForward);

		//Zeroing of sensors
		//ultrasonic.setAutomaticMode(true);
		navxDevice.zeroYaw();
		Rencoder.reset();
		Lencoder.reset();

		//SmartDashboard Auto selections
		//Get SDB Inputs
		pauseTime = SmartDashboard.getNumber("Pause Timer");
		if (pauseTime <= 0) {
			pauseTime = 0.1;
		}
		shooterPause = new Timer();
		autonomousUsingState = (AutonomousUsingState)autoSideSelection.getSelected();

		// Peg choice
		pegChoice = (AutoPegChoice)autoPegSelection.getSelected();


		//LogInfo("SIDE" + sideChoice.toString());
		//LogInfo("PEG" + pegChoice.toString());

		//Calculate Inputs
		/*
		if (pegChoice == AutoPegChoice.Shoot){
			autonomousUsingState = AutonomousUsingState.Shooting;
		}else if (pegChoice == AutoPegChoice.Middle) {
			autonomousUsingState = AutonomousUsingState.Straight;
		}


		if (pegChoice == AutoPegChoice.Left || pegChoice == AutoPegChoice.Right) {
			autonomousUsingState = AutonomousUsingState.PegOnlySide;
		} else {
			autonomousUsingState = AutonomousUsingState.PegOnlyMiddle;
		}
		if (sideChoice == AutoSideChoice.FarShootOnly){

		}if (sideChoice == AutoSideChoice.NearShootOnly){

		}

		 */

		LogInfo("AUTO STATE: " + autonomousUsingState);

		autoPause.start();

	}

	public void autonomousPeriodic() {
		LogInfo("TOP OF AUTONOMOUS");
		ShooterToggle(false, 0, 0, 0, 0);
		SetLeftMotors(0.0f);
		SetRightMotors(0.0f);

		if (autonomousUsingState == AutonomousUsingState.PegOnlyMiddle) {
			LogInfo("Straight State - " + autoStraightState);

			if (autoStraightState == AutoStraightState.WaitingForNavX) {
				if (autoPause.get() >= pauseTime) {
					autoStraightState = AutoStraightState.PlaceGear;
				} else {
					navxDevice.zeroYaw();
				}

			} else if (autoStraightState == AutoStraightState.PlaceGear) {

				float encodersValue = Rencoder.get();				
				LogInfo(""	+ " " + ultrasonic.getValue());
				if (ultrasonic.getValue() >= 83) {
					DriveStraight(0.4f, false);
				} else { 
					autoStraightState = AutoStraightState.End;
				}
			} else if(autoStraightState == AutoStraightState.End) {
				ClearRotation();
				//do nothing
			}
		} else if (autonomousUsingState == AutonomousUsingState.PegOnlySide) {

			LogInfo("Side State - " + autoRightState);

			if (autoRightState == AutoRightState.WaitingForNavX) {
				if (autoPause.get() >= pauseTime) {
					autoRightState = AutoRightState.Forward;
				} else {
					navxDevice.zeroYaw();
				}

			} else if(autoRightState == AutoRightState.Forward){
				float encodersValue = Rencoder.get();
				System.out.println("Encoders " + encodersValue);

				float encoderTargetR = 15900;
				//float encoderTargetL = 13500;
				if (pegChoice == AutoPegChoice.Left && encodersValue <= encoderTargetR) {
					DriveStraight(0.4f, false);
				} else if (pegChoice == AutoPegChoice.Right && encodersValue <= encoderTargetR) {
					DriveStraight(0.4f, false);
				}else{ 
					autoRightState = AutoRightState.Rotate;
					ClearRotation();
				}
			} else if(autoRightState == AutoRightState.Rotate){

				float target = -54;
				float robotYaw1 = navxDevice.getYaw();
				LogInfo("Yaw " + robotYaw1);

				boolean hitTarget = false;
				int sideRotation = 1;
				if (pegChoice == AutoPegChoice.Left) {
					sideRotation = -1;

					if (robotYaw1 < target * sideRotation) {
						hitTarget = true;
					}
				} else {
					sideRotation = 1;

					if (robotYaw1 > target * sideRotation) {
						hitTarget = true;
					}
				}

				if(hitTarget) 
				{
					float speed = 0.4f;
					float speedL = -speed * sideRotation;
					float speedR = -speed * sideRotation;

					SetLeftMotors(speedL);
					SetRightMotors(speedR);
				} else {
					Rencoder.reset();
					autoRightState = AutoRightState.Approach;
					ClearRotation();
				}
			}else if(autoRightState == AutoRightState.Approach){
				if (ultrasonic.getValue() >= 65){
					LogInfo("Ultrasonic " + ultrasonic.getValue());
					float speed = 0.4f;
					float speedL = speed;
					float speedR = -speed;
					SetLeftMotors(speedL);
					SetRightMotors(speedR);
				} else {
					autoRightState = AutoRightState.End;
					ClearRotation();
				}				
			} else if(autoRightState == AutoRightState.End){
				ClearRotation();
				//do nothing 
			}
		}else if(autonomousUsingState == AutonomousUsingState.ShootingBlue || autonomousUsingState == AutonomousUsingState.ShootingRed){
			LogInfo("State - " + autoShootingState);
			if (autoShootingState == AutoShootingState.Pause) {
				if (autoPause.get() >= pauseTime) {
					autoShootingState = AutoShootingState.Forward;
				} else {
					navxDevice.zeroYaw();
				}

			}else if(autoShootingState == AutoShootingState.Forward){
				double encodersValue = Rencoder.get();
				double encoderTarget;
				encoderTarget = 2000;
				if (encodersValue <= encoderTarget) {
					DriveStraight(0.4f, false);
				}else{
					autoShootingState = AutoShootingState.Turn;
					ClearRotation();
				}
			}else if(autoShootingState == AutoShootingState.Turn){
				float robotYaw1 = navxDevice.getYaw();
				LogInfo("Yaw " + robotYaw1);
				if(robotYaw1 <= 75 && autonomousUsingState == AutonomousUsingState.ShootingBlue){
					float speed = 0.4f;
					float speedL = speed;
					float speedR = speed;
					SetLeftMotors(speedL);
					SetRightMotors(speedR);
				}else if(robotYaw1 >= -62 && autonomousUsingState == AutonomousUsingState.ShootingRed){
					float speed = 0.4f;
					float speedL = speed;
					float speedR = speed;
					SetLeftMotors(-speedL);
					SetRightMotors(-speedR);
				}else{
					autoShootingState = AutoShootingState.Fire;
					shooterPause.start();
				}
			}else if(autoShootingState == AutoShootingState.Fire){
				if(shooterPause.get() <= 7){
					ShooterToggle(true, 3975, 3925, -1, -0.5);
				}else{
					autoShootingState = AutoShootingState.TurnTowardTower;
					ClearRotation();
				}
			}else if(autoShootingState == AutoShootingState.TurnTowardTower){
				float target = -80;
				float robotYaw1 = navxDevice.getYaw();
				LogInfo("Yaw " + robotYaw1);
				if(robotYaw1 <= target && autonomousUsingState == AutonomousUsingState.ShootingBlue){
					float speed = 0.4f;
					float speedL = -speed;
					float speedR = -speed;
					SetLeftMotors(speedL);
					SetRightMotors(speedR);
				}else if(robotYaw1 <= target *-1 && autonomousUsingState == AutonomousUsingState.ShootingRed){
					float speed = 0.4f;
					float speedL = speed;
					float speedR = speed;
					SetLeftMotors(speedL);
					SetRightMotors(speedR);
				}else{
					autoShootingState = AutoShootingState.Approach;
					ClearRotation();
				}
			}else if(autoShootingState == AutoShootingState.Approach){
				double encodersValue = Rencoder.get();
				double encoderTarget;
				encoderTarget = 12000;
				if (encodersValue <= encoderTarget) {
					DriveStraight(0.4f, false);	
				}else{
					autoShootingState = AutoShootingState.End;
				}
			}
		}else if(autonomousUsingState == AutonomousUsingState.HopperRed || autonomousUsingState == AutonomousUsingState.HopperBlue){
			LogInfo("State -" + autoHopperState);
			if(autoHopperState == AutoHopperState.Pause){
				if (autoPause.get() >= pauseTime) {
					autoHopperState = AutoHopperState.Forward;
				} else {
					navxDevice.zeroYaw();
				}	
			}else if(autoHopperState == AutoHopperState.Forward){
				double encodersValue = Rencoder.get();
				double encoderTarget;
				encoderTarget = 13300;
				if (encodersValue <= encoderTarget) {
					DriveStraight(0.4f, false);
				}else{
					autoHopperState = AutoHopperState.TurnTowardHopper;
					ClearRotation();
				}
			}else if(autoHopperState == AutoHopperState.TurnTowardHopper){
				float target = 45;
				float robotYaw1 = navxDevice.getYaw();
				LogInfo("Yaw " + robotYaw1);
				if(robotYaw1 <= target && autonomousUsingState == AutonomousUsingState.HopperBlue){
					float speed = 0.4f;
					float speedL = -speed;
					float speedR = -speed;
					SetLeftMotors(speedL);
					SetRightMotors(speedR);
				}else if(robotYaw1 >= target *-1 && autonomousUsingState == AutonomousUsingState.HopperRed){
					float speed = 0.4f;
					float speedL = speed;
					float speedR = speed;
					SetLeftMotors(speedL);
					SetRightMotors(speedR);
				}else{
					autoHopperState = AutoHopperState.Approach;
					ClearRotation();
				}
			}else if(autoHopperState == AutoHopperState.Approach){
				double ultrasonicTarget = 200;
				if (ultrasonic.getValue() >= ultrasonicTarget) {
					DriveStraight(0.4f, false);
				}else{
					ClearRotation();
					autoHopperState = AutoHopperState.BackUp;
				}
			}else if(autoHopperState == AutoHopperState.BackUp){
				double encodersValue = Rencoder.get();
				double encoderTarget;
				encoderTarget = 3175;
				if (encodersValue <= encoderTarget) {
					DriveStraight(-0.4f, true);
				}else{
					autoHopperState = AutoHopperState.TurnTowardBoiler;
					ClearRotation();
				}
			}else if(autoHopperState == AutoHopperState.TurnTowardBoiler){
				float target = -66;
				float robotYaw1 = navxDevice.getYaw();
				LogInfo("Yaw " + robotYaw1);
				if(robotYaw1 <= target && autonomousUsingState == AutonomousUsingState.ShootingBlue){
					float speed = 0.4f;
					float speedL = -speed;
					float speedR = -speed;
					SetLeftMotors(speedL);
					SetRightMotors(speedR);
				}else if(robotYaw1 <= target *-1 && autonomousUsingState == AutonomousUsingState.ShootingRed){
					float speed = 0.4f;
					float speedL = speed;
					float speedR = speed;
					SetLeftMotors(speedL);
					SetRightMotors(speedR);
				}else{
					autoHopperState = AutoHopperState.Fire;
					ClearRotation();
					shooterPause.start();
				}
			}
		}else if(autoHopperState == AutoHopperState.Fire){
			ShooterToggle(true, 3975, 3925, -1, -0.5);
			if(shooterPause.get() == 7){
				autoHopperState = AutoHopperState.End;
			}
		}else if(autoHopperState == AutoHopperState.End){
			//nothing
		}
	
	UpdateMotors();
	feederRPMControl.UpdateRPMControl();
	shooterRPMControl.UpdateRPMControl();
}

public void teleopInit() {
	SmartDashboard.putBoolean("Arcade Drive", true);
	SmartDashboard.putNumber("Four (Agitator)", -1.0f);
	SmartDashboard.putBoolean("Xbox", false);
	SmartDashboard.putNumber("Three (Injector)", -1.0f);
}

public void teleopPeriodic() {
	//Printing
	{
		LogInfo("TOP OF TELEOP");
		LogInfo("COMPRESSOR ENABLED: " + compressor.enabled());
	}

	//Update Motors
	{
		UpdateMotors();
		feederRPMControl.UpdateRPMControl();
		shooterRPMControl.UpdateRPMControl();
		climber.UpdateMotor();
	}

	{
		if(ultrasonic.getValue() <= 92){
			relay.set(Relay.Value.kForward);
		}else{
			relay.set(Relay.Value.kReverse);
		}
	}
	//Logictechs
	//Driving
	{
		float horJoystick = 0;
		float verJoystick = 0;

		float epsilon = 0.2f;
		float leftInput = TranslateController((float)driveControllerL.getRawAxis(0));
		if (leftInput > epsilon || leftInput < -epsilon) {
			horJoystick = leftInput;
		}
		float rightInput = TranslateController((float)driveControllerR.getRawAxis(1));
		if (rightInput > epsilon || rightInput < -epsilon) {
			verJoystick = rightInput;
		}

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
	//Servo buttons
	//{
		//if(driveControllerR.getRawButton(3)) {
			//cameraServo.set(1);
		//} else if(driveControllerR.getRawButton(2)) {
			//cameraServo.set(0);
		//}
	//}
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
		if (xboxController.getRawButton(1) && gearShifterReleased) {
			if(gearShifter.get() == DoubleSolenoid.Value.kReverse) {
				gearShifterReleased = false;
				gearShifter.set(DoubleSolenoid.Value.kForward);
			} else {
				gearShifterReleased =  false;
				gearShifter.set(DoubleSolenoid.Value.kReverse);
			}
		}
		if (!xboxController.getRawButton(1)) {
			gearShifterReleased = true;
		}
	}
	//Shooting Toggle
	{
		if (xboxController.getRawButton(2)) {
			ShooterToggle(true, SmartDashboard.getNumber("One (shooter)"), SmartDashboard.getNumber("Two (feeder)"), -1, -.5);
		} else {
			ShooterToggle(false, 0, 0, 0, 0);
		}
	}
	//Climber
	{
		climber.target = 0.0f;
		if (xboxController.getRawButton(4)) {
			System.out.println("SETTING");
			climber.target = -1;
		}
	}

}

// NOTE the yaw must be reset before using this
public void DriveStraight(float speed, boolean reverse) {

	if (!turnController.isEnabled()) {
		turnController.enable();
	}

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
	navxDevice.reset();
	turnController.setSetpoint(0);
}

public void ShiftDown() {
	leftShifter.set(DoubleSolenoid.Value.kForward);
	rightShifter.set(DoubleSolenoid.Value.kForward);
}

public void ShiftUp() {
	leftShifter.set(DoubleSolenoid.Value.kReverse);
	rightShifter.set(DoubleSolenoid.Value.kReverse);
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
	if (toggle) {
		agitator.set(SmartDashboard.getNumber("Four (Agitator)"));
		injector.set(SmartDashboard.getNumber("Three (Injector)"));
	} else {
		agitator.set(0.0f);
		injector.set(0.0f);
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


/**
 * This function is called periodically during test mode
 */
public void testPeriodic() {
	// LiveWindow.run();
}

@Override
public void pidWrite(double output) {
	// TODO Auto-generated method stub

}
}
