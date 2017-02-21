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
import edu.wpi.first.wpilibj.Compressor;


public class Robot extends IterativeRobot implements PIDOutput {
	public Compressor compressor;

	//Sensors
	public AnalogInput ultrasonic;
	public AHRS navxDevice;

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

	//Cameras
	CameraServer camera;
	public Servo cameraServo;

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
	public double pauseTime = 0.1;

	// Shooter Stuff
	public RPMControl feederRPMControl;
	public RPMControl shooterRPMControl;
	public CANTalon injector;

	//Auto enums
	// Run over harambe's face
	public enum AutoStraightState {
		WaitingForNavX, PlaceGear, End
	}

	// Doing it right 
	public enum AutoRightState {
		WaitingForNavX, Forward, Rotate, Approach, End
	}

	//Left in the dust 
	public enum AutoLeftState {
		WaitingForNavX, Forward, Rotate, Approach, End
	}

	public enum AutoShootingState {
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

	public enum AutoSideChoice {
		Red, Blue
	}

	public enum AutoPegChoice {
		Left, Right, Middle, Shoot
	}

	public AutonomousUsingState autonomousUsingState = AutonomousUsingState.None;

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

		compressor = new Compressor();

		// Camera
		{
			camera = CameraServer.getInstance();
			UsbCamera usbCam = camera.startAutomaticCapture(); 
			usbCam.setResolution(160,  120);
			usbCam.setFPS(30);
		}

		ultrasonic = new AnalogInput(0);

		xboxController = new Joystick(3);
		driveControllerR = new Joystick(1);
		driveControllerL = new Joystick(0);	

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
			float lerpSpeed = 1.0f;
			leftMotorTop = new JoshMotorControllor(8, lerpSpeed, false);
			leftMotorBottom = new JoshMotorControllor(9, lerpSpeed, true);
			rightMotorTop = new JoshMotorControllor(6, lerpSpeed, false);
			rightMotorBottom = new JoshMotorControllor(7, lerpSpeed,true);
			climber = new JoshMotorControllor(5, lerpSpeed, false);
			agitator = new Talon(3);
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

			agitator.set(0.4f);
			injector.set(-0.3f);
		}

		SmartDashboard.putNumber("Pause Timer", pauseTime);

		// These are from the perspective of the driver!
		autoSideSelection = new SendableChooser();
		autoSideSelection.addDefault("Red", AutoSideChoice.Red);
		autoSideSelection.addObject("Blue", AutoSideChoice.Blue);
		SmartDashboard.putData("Autonomous Side Selection", autoSideSelection);

		autoPegSelection = new SendableChooser();
		autoPegSelection.addDefault("Left", AutoPegChoice.Left);
		autoPegSelection.addObject("Middle", AutoPegChoice.Middle);
		autoPegSelection.addObject("Right", AutoPegChoice.Right);
		autoPegSelection.addObject("Shoot", AutoPegChoice.Shoot);
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

		autoStraightState = AutoStraightState.WaitingForNavX;
		autoRightState = AutoRightState.WaitingForNavX;
		autoLeftState = AutoLeftState.WaitingForNavX;
		autoShootingState = AutoShootingState.Pause;

		autoPause = new Timer();
		LogInfo("Auto start");

		pauseTime = SmartDashboard.getNumber("Pause Time");
		if (pauseTime <= 0) {
			pauseTime = 0.1;
		}

		sideChoice = (AutoSideChoice)autoSideSelection.getSelected();
		pegChoice = (AutoPegChoice)autoPegSelection.getSelected();
		LogInfo("SIDE" + sideChoice.toString());
		LogInfo("PEG" + pegChoice.toString());

		if (pegChoice == AutoPegChoice.Shoot){
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
					autoStraightState = AutoStraightState.End;
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
				}
			} else if(autoLeftState == AutoLeftState.End){
				ClearRotation();
			}
		} else if(autonomousUsingState == AutonomousUsingState.Shooting) {
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
		SmartDashboard.putNumber("Four (Agitator)", agitator.get());
		SmartDashboard.putBoolean("Xbox", false);
		SmartDashboard.putNumber("Three (Injector)", agitator.get());
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
		//Servo buttons
		{
			if(driveControllerR.getRawButton(3)) {
				cameraServo.set(1);
			} else if(driveControllerR.getRawButton(2)) {
				cameraServo.set(0);
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
		//Gear Gobbler Pnoomatic
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
				ShooterToggle(!shooterRPMControl.running);
			}
		}
		//Climber
		{
			climber.target = 0;
			if (xboxController.getRawButton(4)) {
				climber.target = 1;
			}
		}
		
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

	public void ShooterToggle(boolean toggle) {
		shooterRPMControl.running = toggle;
		feederRPMControl.running = toggle;
		if (toggle) {
			agitator.set(SmartDashboard.getNumber("Agitator"));
			injector.set(SmartDashboard.getNumber("Injector"));
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
