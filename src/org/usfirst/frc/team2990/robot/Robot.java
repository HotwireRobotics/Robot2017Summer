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

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;


public class Robot extends IterativeRobot implements PIDOutput {
	//Sensors
	public AnalogInput ultrasonic;
	public AHRS navxDevice;

	//PID
	boolean rotateToAngle;
	PIDController turnController;

	//MOTORS:
	// MotorThree is the top motor, it must move opposite the others
	public boolean usingXbox;
	public JoshMotorControllor leftMotorTop;
	public JoshMotorControllor leftMotorBottom;
	public JoshMotorControllor rightMotorTop;
	public JoshMotorControllor rightMotorBottom;
	public Talon shooterFeeder;

	//Climber motor
	public JoshMotorControllor drum;

	//Cameras
	CameraServer camera;
	public Servo cameraServo;

	//Controllers
	public Joystick driveController;
	public Joystick driveControllerL;
	public Joystick driveControllerR;
	public Joystick xboxController;

	//Shifting
	public DoubleSolenoid leftShifter;
	public DoubleSolenoid rightShifter;
	public DoubleSolenoid gearShifter;
	public boolean gearShifterReleased;

	//navx zeroing correctly
	public boolean navxReady;
	public int loopCount;

	//Encoders
	public Encoder Lencoder;
	public Encoder Rencoder;

	//Timers
	public Timer autoPause;
	public Timer waitGear;
	public Timer waitRotate;
	public Timer gearBrake;
	public Timer finalBrake;
	public Timer rightBrake;
	public Timer leftBrake;
	public double pauseTime = 0.1;

	// Shooter Stuff
	public RPMControl feederRPMControl;
	public RPMControl shooterRPMControl;

	//Auto enums
	public enum AutoStraightState // Run over harambe's face
	{
		WaitingForNavX, PlaceGear, WaitGear, ReverseGear, WaitRotate, Rotate, CrossLine, End
	}

	public enum AutoRightState // Doing it right
	{
		WaitingForNavX, Forward, Rotate, Approach, End
	}
	public enum AutoLeftState //Left in the dust
	{
		WaitingForNavX, Forward, Rotate, Approach, End
	}
	public enum AutoShootingState
	{
		Pause, Shoot 
	} 
	public AutoStraightState autoStraightState;
	public AutoRightState autoRightState;
	public AutoLeftState autoLeftState;
	public AutoShootingState autoShootingState;

	// Which autonomous program we're using. From the perspective of the
	// drivers.
	public enum AutonomousUsingState {
		Straight, Right, Left, Shooting, None
	}
	public AutonomousUsingState autonomousUsingState = AutonomousUsingState.None;
	
	public enum AutoSideChoice {
		Red, Blue
	}
	public enum AutoPegChoice {
		Left, Right, Middle, None
	}

	public SendableChooser autoSideSelection;
	public AutoSideChoice sideChoice;
	public SendableChooser autoPegSelection;
	public AutoPegChoice pegChoice;

	public int blue;

	public void robotInit() {
		cameraServo = new Servo(4);

		navxDevice = new AHRS(SPI.Port.kMXP);

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
			camera = CameraServer.getInstance();
			UsbCamera usbCam = camera.startAutomaticCapture(); 
			usbCam.setResolution(160,  120);
			usbCam.setFPS(30);
		}

		ultrasonic = new AnalogInput(0);

		if (usingXbox) {
			xboxController = new Joystick(0);
		} else {
			driveControllerR = new Joystick(1);
			driveControllerL = new Joystick(0);	
		}

		//Shifters
		{
			leftShifter = new DoubleSolenoid(0, 1);
			rightShifter = new DoubleSolenoid(2, 3);
			gearShifter = new DoubleSolenoid(4, 5);
		}

		//Timers
		{
			waitGear = new Timer();
			waitRotate = new Timer();
			finalBrake = new Timer();
		}

		//Encoders
		{
			Lencoder = new Encoder(0, 1);
			Rencoder = new Encoder(2, 3);
		}

		//Motor Controllers
		{
			float lerpSpeed = 1.0f;
			leftMotorTop = new JoshMotorControllor(8, lerpSpeed, false);
			leftMotorBottom = new JoshMotorControllor(9, lerpSpeed, true);
			rightMotorTop = new JoshMotorControllor(6, lerpSpeed, false);
			rightMotorBottom = new JoshMotorControllor(7, lerpSpeed,true);
			drum = new JoshMotorControllor(5, 0.2f, false);
			shooterFeeder = new Talon(3);
		}

		//Shooter
		{
			float p = 0.2f;
			float i = .00000003f;
			float d = 6.06275f;
			float f = 0.35f;
			feederRPMControl = new RPMControl("RPM Feeder", 5, true, p, i, d, f, 4500);
			shooterRPMControl = new RPMControl("RPM Shooter", 2, true, p, i, d, f, 4500); 
		}
		
		// These are from the perspective of the driver!
		autoSideSelection = new SendableChooser();
		autoSideSelection.addDefault("Red", AutoSideChoice.Red);
		autoSideSelection.addObject("Blue", AutoSideChoice.Blue);
		SmartDashboard.putData("Autonomous Side Selection", autoSideSelection);
		
		autoPegSelection = new SendableChooser();
		autoPegSelection.addDefault("Left", AutoPegChoice.Left);
		autoPegSelection.addObject("Middle", AutoPegChoice.Middle);
		autoPegSelection.addObject("Right", AutoPegChoice.Right);
		SmartDashboard.putData("Autonomous Peg Selection", autoPegSelection);
	}
	
	public void autonomousInit() {
		LogInfo("AUTO INIT");
		navxDevice.zeroYaw();

		blue = +1;
		
		//ultrasonic.setAutomaticMode(true);
		Rencoder.reset();
		Lencoder.reset();

		ShiftUp();
		gearShifter.set(DoubleSolenoid.Value.kForward);

		navxReady = false;
		loopCount = 0;

		autoStraightState = AutoStraightState.WaitingForNavX;
		autoRightState = AutoRightState.WaitingForNavX;
		autoLeftState = AutoLeftState.WaitingForNavX;
		autoShootingState = AutoShootingState.Pause;
		
		autoPause = new Timer();
		waitRotate = new Timer();
		gearBrake = new Timer();
		finalBrake = new Timer();
		rightBrake = new Timer();
		leftBrake = new Timer();
		LogInfo("Auto start");

		sideChoice = (AutoSideChoice)autoSideSelection.getSelected();
		pegChoice = (AutoPegChoice)autoPegSelection.getSelected();
		LogInfo("SIDE" + sideChoice.toString());
		LogInfo("PEG" + pegChoice.toString());
		
		if (pegChoice == AutoPegChoice.None){
			autonomousUsingState = AutonomousUsingState.Shooting;
		}else if (pegChoice == AutoPegChoice.Middle) {
			autonomousUsingState = AutonomousUsingState.Straight;
		}
		if (sideChoice == AutoSideChoice.Blue) {
			blue = -1;
			if (pegChoice == AutoPegChoice.Left) {
				autonomousUsingState = AutonomousUsingState.Right;
			}
			if (pegChoice == AutoPegChoice.Right) {
				autonomousUsingState = AutonomousUsingState.Left;
			}
		} else {
			if (pegChoice == AutoPegChoice.Left) {
				autonomousUsingState = AutonomousUsingState.Left;
			}
			if (pegChoice == AutoPegChoice.Right) {
				autonomousUsingState = AutonomousUsingState.Right;
			}
		}
		
		LogInfo("AUTO STATE: " + autonomousUsingState + blue);
		
		autoPause.start();
		
	}

	public void autonomousPeriodic() {
		LogInfo("TOP OF AUTONOMOUS");
		
		//System.out.println(Rencoder.get());
		SetLeftMotors(0.0f);
		SetRightMotors(0.0f);

		if (autonomousUsingState == AutonomousUsingState.Straight)
		{
			LogInfo("Straight State - " + autoStraightState);
			
			if (autoStraightState == AutoStraightState.WaitingForNavX) {
				if (autoPause.get() >= pauseTime) {
					autoStraightState = AutoStraightState.PlaceGear;
				} else {
					navxDevice.zeroYaw();
				}
				
			} else if (autoStraightState == AutoStraightState.PlaceGear) {

				float encodersValue = Rencoder.get();				
				LogInfo(""
						+ " " + ultrasonic.getValue());
				if (ultrasonic.getValue() >= 83) {
					DriveStraight(0.3f);
				} else { 
					autoStraightState = AutoStraightState.WaitGear;

					SetLeftMotors(0);
					SetRightMotors(0);

					ClearRotation();
					waitGear.start();
				}
			} else if (autoStraightState == AutoStraightState.WaitGear) {
				if(waitGear.get() > 1.5f) {
					autoStraightState = AutoStraightState.ReverseGear;
					ClearRotation();
				}
			} else if(autoStraightState == AutoStraightState.ReverseGear) {
				float encodersValue = Rencoder.get();
				LogInfo("Encoder " + encodersValue);

				float encoderTarget = 6500;
				if (encodersValue >= encoderTarget) {
					float speed = 0.4f;
					float speedL = -speed;
					float speedR = speed;
					SetLeftMotors(speedL);
					SetRightMotors(speedR);
				} else {
					autoStraightState = AutoStraightState.WaitRotate;
					waitRotate.start();
					ClearRotation();
				}
			} else if(autoStraightState == AutoStraightState.WaitRotate) {
				if(waitRotate.get() > 1f) {
					autoStraightState = AutoStraightState.Rotate;
					ClearRotation();
				}
			} else if(autoStraightState == AutoStraightState.Rotate) {
				float target = 35;
				float robotYaw1 = navxDevice.getYaw();
				LogInfo("Yaw " + robotYaw1);
				if(robotYaw1 < target) {
					float speed = 0.6f;
					float speedL = speed;
					float speedR = speed;

					SetLeftMotors(speedL);
					SetRightMotors(speedR);
				} else {
					autoStraightState = AutoStraightState.CrossLine;
					Rencoder.reset();
					ClearRotation();
				}
			} else if(autoStraightState == AutoStraightState.CrossLine) {
				float encoderTarget = 24000;

				float encodersValue = Rencoder.get();
				LogInfo("Encoders " + encodersValue);
				if (encodersValue < encoderTarget) {
					float speed = 0.4f;
					float speedL = speed;
					float speedR = -speed;
					SetLeftMotors(speedL);
					SetRightMotors(speedR);
				} else { 
					autoStraightState = AutoStraightState.End;
					finalBrake.start();
				}
			} else if(autoStraightState == AutoStraightState.End) {
				ClearRotation();
				//do nothing
			}
		} else if (autonomousUsingState == AutonomousUsingState.Right) {

			LogInfo("Right State - " + autoRightState);

			if (autoRightState == AutoRightState.WaitingForNavX) {
				if (autoPause.get() >= pauseTime) {
					autoRightState = AutoRightState.Forward;
				} else {
					navxDevice.zeroYaw();
				}
				
			} else if(autoRightState == AutoRightState.Forward){
				float encodersValue = Rencoder.get();
				System.out.println("Encoders " + encodersValue);

				if (ultrasonic.getValue() >= 83) {
					DriveStraight(0.3f);
				} else {
					autoRightState = AutoRightState.Rotate;
					ClearRotation();
				}
			} else if(autoRightState == AutoRightState.Rotate){
				float target = -51 * blue;
				float robotYaw1 = navxDevice.getYaw();
				LogInfo("Yaw " + robotYaw1);
				if(robotYaw1 > target) 
				{
					float speed = 0.4f;
					float speedL = -speed * blue;
					float speedR = -speed  * blue;

					SetLeftMotors(speedL);
					SetRightMotors(speedR);
				} else {
					Rencoder.reset();
					autoRightState = AutoRightState.Approach;
					ClearRotation();
				}
			}else if(autoRightState == AutoRightState.Approach){
				if (ultrasonic.getValue() >= 110){
					LogInfo("Ultrasonic " + ultrasonic.getValue());
					float speed = 0.4f;
					float speedL = speed;
					float speedR = -speed;
					SetLeftMotors(speedL);
					SetRightMotors(speedR);
				} else {
					autoRightState = AutoRightState.End;
					rightBrake.start();
					ClearRotation();
				}				
			} else if(autoRightState == AutoRightState.End){
				ClearRotation();
				//do nothing 
			}
		} else if(autonomousUsingState == AutonomousUsingState.Left){

			LogInfo("Left State - " + autoLeftState);

			if (autoLeftState == AutoLeftState.WaitingForNavX) {
				if (autoPause.get() >= pauseTime) {
					autoLeftState = AutoLeftState.Forward;
				} else {
					navxDevice.zeroYaw();
				}
				
			} else if(autoLeftState == AutoLeftState.Forward){
				float encodersValue = Rencoder.get();
				LogInfo("Encoder " + encodersValue + ";");

				float encoderTarget = -15275;
				if (encodersValue <= encoderTarget) {
					DriveStraight(0.4f);
				} else {
					autoLeftState = AutoLeftState.Rotate;
					ClearRotation();
				}
			} else if(autoLeftState == AutoLeftState.Rotate){
				float target = 49 * blue;
				float robotYaw1 = navxDevice.getYaw();
				LogInfo("Yaw " + robotYaw1);

				if(robotYaw1 < target) {
					float speed = 0.4f;
					float speedL = speed * blue;
					float speedR = speed * blue;

					SetLeftMotors(speedL);
					SetRightMotors(speedR);
				} else {
					Rencoder.reset();
					autoLeftState = AutoLeftState.Approach;
					ClearRotation();
				}
			} else if(autoLeftState == AutoLeftState.Approach){
				float encodersValue = Rencoder.get();
				LogInfo("Encoder " + encodersValue);

				float encoderTarget = 6000;
				if (encodersValue <= encoderTarget) {
					DriveStraight(0.4f);
				}else{
					autoLeftState = AutoLeftState.End;
					leftBrake.start();
				}
			} else if(autoLeftState == AutoLeftState.End){
				ClearRotation();
			}
		}else if(autonomousUsingState == AutonomousUsingState.Shooting)
		{
			autoShootingState = AutoShootingState.Pause;
			
			if(autoShootingState == AutoShootingState.Pause){
				if (autoPause.get() >= pauseTime) {
					autoShootingState = AutoShootingState.Shoot;
				}
			}else if(autoShootingState == AutoShootingState.Shoot){
				//shoot
			}
		}

		UpdateMotors();
	}

	public void teleopInit() {
		SmartDashboard.putBoolean("Arcade Drive", true);
		SmartDashboard.putNumber("Shooter Feeder Speed", 0.3f);
		SmartDashboard.putBoolean("Xbox", false);
	}

	public void teleopPeriodic() {
		LogInfo("TOP OF TELEOP");
		
		UpdateMotors();

		//feederRPMControl.UpdateRPMControl();
		//shooterRPMControl.UpdateRPMControl();
		
		//shooterFeeder.set(SmartDashboard.getNumber("Shooter Feeder Speed"));
		
		//drum.UpdateMotor();

		float horJoystick = 0;
		float verJoystick = 0;

		if (SmartDashboard.getBoolean("Xbox")) {
			usingXbox = true;
		}else if (!SmartDashboard.getBoolean("Xbox"))
		{
			usingXbox = !true;
		}
		
		if (SmartDashboard.getBoolean("Arcade Drive")) {
			// Arcade drive
			if (usingXbox) {
				horJoystick = TranslateController((float) xboxController.getRawAxis(0));
				verJoystick = TranslateController((float) xboxController.getRawAxis(5));

				drum.target = (0);
				if (xboxController.getRawAxis(3) > 0.5) {
					drum.target = (float) xboxController.getRawAxis(3);
				} else if (xboxController.getRawAxis(2) > 0.5) {
					drum.target = (float) -xboxController.getRawAxis(2);
				}
			} else {
				if (driveControllerL.getRawButton(3)) {
					drum.target = 1;
				} else if (driveControllerL.getRawButton(2)) {
					drum.target = -1;
				} else {
					drum.target = 0;
				}

				if(driveControllerR.getRawButton(3)) {
					cameraServo.set(1);
				} else if(driveControllerR.getRawButton(2)) {
					cameraServo.set(0);
				}

				float epsilon = 0.2f;
				float leftInput = TranslateController((float)driveControllerL.getRawAxis(0));
				if (leftInput > epsilon || leftInput < -epsilon) {
					horJoystick = leftInput;
				}
				float rightInput = TranslateController((float)driveControllerR.getRawAxis(1));
				if (rightInput > epsilon || rightInput < -epsilon) {
					verJoystick = rightInput;
				}
			}

			SetLeftMotors(verJoystick + horJoystick);
			SetRightMotors(-verJoystick + horJoystick);
		} else {
			// Tank drive
			if (usingXbox) {
				horJoystick = (float) xboxController.getRawAxis(1);
				verJoystick = (float) xboxController.getRawAxis(5);
			} else {
				horJoystick = (float) driveControllerL.getRawAxis(0);
				verJoystick = (float) driveControllerR.getRawAxis(1);
			}

			SetLeftMotors(-horJoystick);
			SetRightMotors(verJoystick);
		}

		boolean shiftUp = false;
		boolean shiftDown = false;

		if (usingXbox) {
			if (xboxController.getRawButton(6)) {
				shiftUp = true;
			} else if (xboxController.getRawButton(5)) {
				shiftDown = true;
			}
			if(xboxController.getRawButton(10) && gearShifterReleased) {
				if(gearShifter.get() == DoubleSolenoid.Value.kReverse) {
					gearShifterReleased = false;
					gearShifter.set(DoubleSolenoid.Value.kForward);
				} else {
					gearShifterReleased =  false;
					gearShifter.set(DoubleSolenoid.Value.kReverse);
				}
			}
			if (!xboxController.getRawButton(10)) {
				gearShifterReleased = true;
			}
		} else {
			if (driveControllerR.getRawButton(4)) {
				shiftUp = true;
				LogInfo("Shifting up");
			}
			if (driveControllerR.getRawButton(5)) {
				shiftDown = true;
				LogInfo("Shifting down");
			}
			if(driveControllerR.getRawButton(1) && gearShifterReleased) {
				if(gearShifter.get() == DoubleSolenoid.Value.kReverse) {
					gearShifterReleased = false;
					gearShifter.set(DoubleSolenoid.Value.kForward);
				} else {
					gearShifterReleased =  false;
					gearShifter.set(DoubleSolenoid.Value.kReverse);
				}
			}
			if (!driveControllerR.getRawButton(1)) {
				gearShifterReleased = true;
			}
		}
		
		shooterRPMControl.running = true;
		feederRPMControl.running = true;

		if (shiftUp) {
			ShiftUp();
		}
		if (shiftDown) {
			ShiftDown();
		}
		
		//drum.target = (float)SmartDashboard.getNumber("Shooter Feeder Speed");
	}

	// NOTE the yaw must be reset before using this
	public void DriveStraight(float speed) {

		if (!turnController.isEnabled()) {
			turnController.enable();
		}

		float pidError = (float)turnController.get();
		SetLeftMotors((speed * pidError) + speed); //0.6972
		SetRightMotors(((speed) - (speed * pidError)) * -1); //-0.583

		speed = -speed;
		
		LogInfo("STRAIGHT YAW " + navxDevice.getYaw());
	}

	public void ClearRotation() {
		navxDevice.reset();
		turnController.setSetpoint(0);
	}

	public void ShiftDown() {
		leftShifter.set(DoubleSolenoid.Value.kForward);
		rightShifter.set(DoubleSolenoid.Value.kReverse);
	}

	public void ShiftUp() {
		leftShifter.set(DoubleSolenoid.Value.kReverse);
		rightShifter.set(DoubleSolenoid.Value.kForward);
	}

	public float TranslateController(float input)
	{
		float deadzone = 0.10f;
		if (input > -deadzone && input < deadzone) {
			input = 0.0f;
		}
		float a = 0.7f;
		float output = (a * input * input * input) + (1 - a) * input; 
		return output;
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
