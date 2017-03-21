package org.usfirst.frc.team2990.robot;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;
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
	public Servo cameraServo;

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
	public Encoder Lencoder;
	public Encoder Rencoder;

	//Timers
	public Timer autoPause;
	public Timer shooterPause;
	public double pauseTime = 0.1;
	public Timer smashTimer;
	public Timer escapeTimer;

	// Shooter Stuff
	public RPMControl feederRPMControl;
	public RPMControl shooterRPMControl;
	public CANTalon injector;
	public Spark stirer;
	public Spark agitator;

	//Auto enums
	// Middle Peg
	public enum AutoStraightState {
		WaitingForNavX, PlaceGear, BackUp, Rotate, Fire, ApproachBoiler, Turn, Escape, baseLineRotate, GetAway, End
	}
	// Right Peg 
	public enum AutoRightState {
		WaitingForNavX, Forward, Rotate, Approach, Backup, End
	}
	// Left Peg
	public enum AutoLeftState {
		WaitingForNavX, Forward, Rotate, Approach, BackUp, TurnTowardBoiler, Shoot, End
	}
	// Shooting, then going near left peg
	public enum AutoShootingState {
		Pause, Forward, Turn, Fire, TurnTowardAirship, Approach, CrossLine, End
	} 
	public enum AutoHopperState {
		Pause, Forward, TurnTowardHopper, Approach, HopperSmash, BackUp, TurnTowardBoiler, Fire, End
	}
	public AutoStraightState autoStraightState;
	public AutoRightState autoRightState;
	public AutoLeftState autoLeftState;
	public AutoShootingState autoShootingState;
	public AutoHopperState autoHopperState;
	// Which autonomous program we're using. From the perspective of the drivers.
	public enum AutonomousUsingState {
		PegOnlyMiddleBlue, PegOnlyMiddleRed, PegOnlySide, None, ShootingBlue, ShootingRed, HopperRed, HopperBlue,
	}
	public AutonomousUsingState autonomousUsingState = AutonomousUsingState.None;

	public SendableChooser autoSelection;
	public Relay relay;

	public void robotInit() {
		relay = new Relay(0);
		//Sensors
		{
			navxDevice = new AHRS(SPI.Port.kMXP);
			compressor = new Compressor();
			ultrasonic = new AnalogInput(1);
		}

		// Straight Drive PID
		{
			turnController = new PIDController(0.05, 0, 0.00007, 0, navxDevice, this);
			turnController.setInputRange(-180.0f, 180.0f);
			turnController.setOutputRange(-1.0f, 1.0f);
			turnController.setAbsoluteTolerance(2.0);
			turnController.setContinuous(true);
		}

		// Camera
		{
			cameraServo = new Servo(4);
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
			agitator = new Spark(3);
			stirer = new Spark(2);
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

			injector.set(-0.0f);
			agitator.set(0.0f);
			stirer.set(0.0f);

		}
		escapeTimer = new Timer();
		SmartDashboard.putNumber("Initial Pause", pauseTime);

		// These are from the perspective of the driver!
		{
			autoSelection = new SendableChooser();
			autoSelection.addDefault("Center Blue", AutonomousUsingState.PegOnlyMiddleBlue);
			autoSelection.addDefault("Center Red", AutonomousUsingState.PegOnlyMiddleRed);
			autoSelection.addObject("Right Peg", AutonomousUsingState.PegOnlySide);
			autoSelection.addObject("ShootBlue", AutonomousUsingState.ShootingBlue);
			autoSelection.addObject("ShootRed", AutonomousUsingState.ShootingRed);
			autoSelection.addObject("HopperBlue", AutonomousUsingState.HopperBlue);
			autoSelection.addObject("HopperRed", AutonomousUsingState.HopperRed);
			SmartDashboard.putData("Autonomous Side Select", autoSelection);
		}
	}

	public void autonomousInit() {
		LogInfo("AUTO INIT");

		ShooterToggle(false, 0, 0, 0, 0);

		//Initializations

		autoPause = new Timer();
		//State inits
		autoStraightState = AutoStraightState.WaitingForNavX;
		autoRightState = AutoRightState.WaitingForNavX;
		autoLeftState = AutoLeftState.WaitingForNavX;
		autoShootingState = AutoShootingState.Pause;
		autoHopperState = AutoHopperState.Pause;
		//Shifter inits
		ShiftDown();
		gearShifter.set(DoubleSolenoid.Value.kForward);

		//Zeroing of sensors
		//ultrasonic.setAutomaticMode(true);
		navxDevice.zeroYaw();
		Rencoder.reset();
		Lencoder.reset();

		//SmartDashboard Auto selections
		//Get SDB Inputs
		pauseTime = SmartDashboard.getNumber("Initial Pause");
		if (pauseTime <= 0) {
			pauseTime = 0.1;
		}
		shooterPause = new Timer();
		autonomousUsingState = (AutonomousUsingState)autoSelection.getSelected();

		smashTimer = new Timer();

		LogInfo("AUTO STATE: " + autonomousUsingState);

		autoPause.start();
	}

	public void autonomousPeriodic() {
		LogInfo("TOP OF AUTONOMOUS");
		SetLeftMotors(0.0f);
		SetRightMotors(0.0f);

		if (autonomousUsingState == AutonomousUsingState.PegOnlyMiddleBlue || autonomousUsingState == AutonomousUsingState.PegOnlyMiddleRed) {
			LogInfo("Straight State - " + autoStraightState);

			if (autoStraightState == AutoStraightState.WaitingForNavX) {
				if (autoPause.get() >= pauseTime) {
					autoStraightState = AutoStraightState.PlaceGear;
				} else {
					navxDevice.zeroYaw();

					if (!turnController.isEnabled()) {
						turnController.enable();
					}
				}

			} else if (autoStraightState == AutoStraightState.PlaceGear) {

				float encodersValue = Rencoder.get();				
				LogInfo(""	+ " " + ultrasonic.getValue());
				if (ultrasonic.getValue() >= 100) {
					DriveStraight(1.0f, false);
				} else { 
					autoStraightState = AutoStraightState.BackUp;
					gearDrop.set(DoubleSolenoid.Value.kForward);
				}
			}else if(autoStraightState == AutoStraightState.BackUp){
				LogInfo("Encoders:"  + Rencoder.get());
				float encodersValue = Rencoder.get();				
				if(encodersValue >= 10000){
					DriveStraight(-1f, true);	
				}else{
					autoStraightState = AutoStraightState.Rotate;
				}
			}else if(autoStraightState == AutoStraightState.Rotate){
				if(autonomousUsingState == AutonomousUsingState.PegOnlyMiddleBlue){
					float robotYaw1 = navxDevice.getYaw();
					LogInfo("Yaw:" + navxDevice.getYaw());
					if(robotYaw1 <= 60){
						float speed = 0.4f;
						float speedL = speed;
						float speedR = speed;
						SetLeftMotors(speedL);
						SetRightMotors(speedR);
					}else{
						autoStraightState = AutoStraightState.Fire;
						shooterPause.start();
					}
				}else if(autonomousUsingState == AutonomousUsingState.PegOnlyMiddleRed){
					LogInfo("Yaw:" + navxDevice.getYaw());
					if(navxDevice.getYaw() >= -53){
						float speed = 0.4f;
						float speedL = -speed;
						float speedR = -speed;
						SetLeftMotors(speedL);
						SetRightMotors(speedR);
					}else{
						autoStraightState = AutoStraightState.Fire;
						shooterPause.start();
						gearDrop.set(DoubleSolenoid.Value.kReverse);
					}
				}
			}else if(autoStraightState == AutoStraightState.Fire){
				if(shooterPause.get() <= 6)
				{
					ShooterToggle(true, 4300, 4350, -1, -1.0f);
					gearDrop.set(DoubleSolenoid.Value.kReverse);
				}else{
					ShooterToggle(false, 0, 0, 0, 0f);
					autoStraightState = AutoStraightState.ApproachBoiler;
					ClearRotation();
				}
			}else if(autoStraightState == AutoStraightState.ApproachBoiler){
				LogInfo("Encoders:"  + Rencoder.get());
				if(autonomousUsingState == AutonomousUsingState.PegOnlyMiddleBlue){
					float encodersValue = Rencoder.get();				
					if(encodersValue >= -8800){
						float speed = 1f;
						float speedL = -speed;
						float speedR = speed;
						SetLeftMotors(speedL);
						SetRightMotors(speedR);
					}else{
						autoStraightState = AutoStraightState.Turn;
						ClearRotation();
					}
				}else if(autonomousUsingState == AutonomousUsingState.PegOnlyMiddleRed){
					float encodersValue = Rencoder.get();				
					if(encodersValue >= -500){
						float speed = 1f;
						float speedL = -speed;
						float speedR = speed *0.75f;
						SetLeftMotors(speedL);
						SetRightMotors(speedR);
					}else{
						autoStraightState = AutoStraightState.Turn;
						ClearRotation();

					}
				}
			}else if(autoStraightState == AutoStraightState.Turn){
				if(autonomousUsingState == AutonomousUsingState.PegOnlyMiddleBlue){
					float robotYaw1 = navxDevice.getYaw();
					LogInfo("Yaw:" + navxDevice.getYaw());
					if(robotYaw1 >= -47){
						float speed = 1f;
						float speedL = -speed;
						float speedR = -speed;
						SetLeftMotors(speedL);
						SetRightMotors(speedR);
					}else{
						escapeTimer.start();
						autoStraightState = AutoStraightState.Escape;
						ClearRotation();
					}
				}else if(autonomousUsingState == AutonomousUsingState.PegOnlyMiddleRed){
					LogInfo("Yaw:" + navxDevice.getYaw());
					if(navxDevice.getYaw() <= 62){
						float speed = 1f;
						float speedL = speed;
						float speedR = speed;
						SetLeftMotors(speedL);
						SetRightMotors(speedR);
					}else{
						escapeTimer.start();
						autoStraightState = AutoStraightState.Escape;
						ClearRotation();
					}
				}
			}else if(autoStraightState == AutoStraightState.Escape){
				if (escapeTimer.get() <= .25) {
					ShiftUp();
					ClearRotation();
				} else {

					LogInfo("Encoders:"  + Rencoder.get());
					float encodersValue = Rencoder.get();				
					if(encodersValue <= 5000){
						DriveStraight(1.0f, false);	
					}else{
						autoStraightState = AutoStraightState.baseLineRotate;
						ClearRotation();
						ShiftDown();
					}
				}
			}else if(autoStraightState == AutoStraightState.baseLineRotate){
				LogInfo("Encoders:"  + Rencoder.get());
				if(autonomousUsingState == AutonomousUsingState.PegOnlyMiddleBlue){
					LogInfo("Yaw:" + navxDevice.getYaw());
					if(navxDevice.getYaw() <= 10){
						float speed = 1f;
						float speedL = speed;
						float speedR = speed;
						SetLeftMotors(speedL);
						SetRightMotors(speedR);
					}else{
						autoStraightState = AutoStraightState.GetAway;
						ShiftUp();
						ClearRotation();
					}
				}else if(autonomousUsingState == AutonomousUsingState.PegOnlyMiddleRed){
					LogInfo("Yaw:" + navxDevice.getYaw());
					if(navxDevice.getYaw() >= -10){
						float speed = 1f;
						float speedL = -speed;
						float speedR = -speed;
						SetLeftMotors(speedL);
						SetRightMotors(speedR);
					}else{
						autoStraightState = AutoStraightState.GetAway;
						ShiftUp();
						ClearRotation();
					}
				}
			}else if(autoStraightState == AutoStraightState.GetAway){
				LogInfo("Encoders:"  + Rencoder.get());
				float encodersValue = Rencoder.get();				
				if(encodersValue <= 30000){
					float speed = 1f;
					float speedL = speed;
					float speedR = -speed;
					SetLeftMotors(speedL);
					SetRightMotors(speedR);
				}else{
					autoStraightState = AutoStraightState.End;
					ClearRotation();
					ShiftDown();
				}
			}else if(autoStraightState == AutoStraightState.End) {
			ClearRotation();
			//do nothing
			ShiftUp();
		}
	} else if (autonomousUsingState == AutonomousUsingState.PegOnlySide) {

		LogInfo("Side State - " + autoLeftState);

		if (autoLeftState == AutoLeftState.WaitingForNavX) {
			if (autoPause.get() >= pauseTime) {
				autoLeftState = AutoLeftState.Forward;
			} else {
				ClearRotation();
				if (!turnController.isEnabled()) {
					turnController.enable();
				}
			}

		} else if(autoLeftState == AutoLeftState.Forward){
			float encodersValue = Rencoder.get();
			System.out.println("Encoders " + encodersValue);

			float encoderTargetR = 14000;
			if (encodersValue <= encoderTargetR) {
				DriveStraight(0.5f, false);
			}else{ 
				autoLeftState = AutoLeftState.Rotate;
				ClearRotation();
			}
		} else if(autoLeftState == AutoLeftState.Rotate){

			float target = 54;
			float robotYaw1 = navxDevice.getYaw();
			LogInfo("Yaw " + robotYaw1);

			boolean hitTarget = false;

			if (robotYaw1 > target) {
				hitTarget = true;
			}

			if(hitTarget) 
			{
				float speed = 0.4f;
				float speedL = speed;
				float speedR = speed;

				SetLeftMotors(speedL);
				SetRightMotors(speedR);
			} else {
				Rencoder.reset();
				autoLeftState = AutoLeftState.Approach;
				ClearRotation();
			}
		}else if(autoLeftState == AutoLeftState.Approach){
			if (ultrasonic.getValue() >= 65){
				LogInfo("Ultrasonic " + ultrasonic.getValue());
				float speed = 0.4f;
				float speedL = speed;
				float speedR = -speed;
				SetLeftMotors(speedL);
				SetRightMotors(speedR);
			} else {
				gearDrop.set(DoubleSolenoid.Value.kForward);
				autoLeftState = AutoLeftState.BackUp;
				ClearRotation();
			}				
		}else if(autoLeftState == AutoLeftState.BackUp){
			LogInfo("Encoders:"  + Rencoder.get());
			float encodersValue = Rencoder.get();				
			if(encodersValue >= 10000){
				float speed = 0.4f;
				float speedL = -speed;
				float speedR = speed;
				SetLeftMotors(speedL);
				SetRightMotors(speedR);

			}else{
				autoLeftState = AutoLeftState.TurnTowardBoiler;
			}
		}else if(autoLeftState == AutoLeftState.TurnTowardBoiler){
			if(navxDevice.getYaw() >= -12){
				float speed = 1f;
				float speedL = -speed;
				float speedR = -speed;
				SetLeftMotors(speedL);
				SetRightMotors(speedR);
			}else{
				autoLeftState = AutoLeftState.Shoot;
				ClearRotation();
				shooterPause.start();
			}
		}else if(autoLeftState == AutoLeftState.Shoot){
			if(shooterPause.get() <= 6)
			{
				ShooterToggle(true, 4300, 4350, -1, -1.0f);
				gearDrop.set(DoubleSolenoid.Value.kReverse);
			}else{
				ShooterToggle(false, 0, 0, 0, 0f);
				autoLeftState = AutoLeftState.End;
				ClearRotation();
			}
		}else if(autoLeftState == AutoLeftState.End){
			ClearRotation();
			ShiftUp();
			gearDrop.set(DoubleSolenoid.Value.kReverse);
			//do nothing 
		}
	}else if(autonomousUsingState == AutonomousUsingState.ShootingBlue || autonomousUsingState == AutonomousUsingState.ShootingRed){
		LogInfo("State - " + autoShootingState);
		if (autoShootingState == AutoShootingState.Pause) {
			if (autoPause.get() >= 1.0f) {
				autoShootingState = AutoShootingState.Forward;
			} else {
				navxDevice.zeroYaw();
			}
		}else if(autoShootingState == AutoShootingState.Forward){
			double encodersValue = Rencoder.get();
			double encoderTarget;
			encoderTarget = 1500;
			if (encodersValue <= encoderTarget) {
				DriveStraight(0.3f, false);
			}else{
				autoShootingState = AutoShootingState.Turn;
				ClearRotation();
			}
		}else if(autoShootingState == AutoShootingState.Turn){
			float robotYaw1 = navxDevice.getYaw();
			LogInfo("Yaw " + robotYaw1);
			if(robotYaw1 <= 73 && autonomousUsingState == AutonomousUsingState.ShootingBlue){
				float speed = 0.4f;
				float speedL = speed;
				float speedR = speed;
				SetLeftMotors(speedL);
				SetRightMotors(speedR);
			}else if(robotYaw1 >= -68 && autonomousUsingState == AutonomousUsingState.ShootingRed){
				float speed = 0.4f;
				float speedL = speed;
				float speedR = speed;
				SetLeftMotors(-speedL);
				SetRightMotors(-speedR);
			}else{
				autoShootingState = AutoShootingState.Approach;
				ClearRotation();
			}
		}else if(autoShootingState == AutoShootingState.Approach){
			double encodersValue = Rencoder.get();
			LogInfo("ENCODER "+Rencoder.get());
			if ((encodersValue >= -10000 && autonomousUsingState == AutonomousUsingState.ShootingBlue) || 
					(encodersValue >= -3000 && autonomousUsingState == AutonomousUsingState.ShootingRed)) {
				SetLeftMotors(-0.4f);
				SetRightMotors(0.4f);
			}else{
				autoShootingState = AutoShootingState.Fire;
				shooterPause.start();
			}		
		}else if(autoShootingState == AutoShootingState.Fire){
			if(shooterPause.get() <= 6.5f){
				ShooterToggle(true, 4400, 4850, -1, -1.0f);
			}else{
				autoShootingState = AutoShootingState.TurnTowardAirship;
				ShooterToggle(false, 0.0f, 0.0f, 0.0f, 0.0f);
				ClearRotation();
			}
		}else if(autoShootingState == AutoShootingState.TurnTowardAirship){
			float robotYaw1 = navxDevice.getYaw();
			LogInfo("Yaw " + robotYaw1);
			if(robotYaw1 >= -28 && autonomousUsingState == AutonomousUsingState.ShootingBlue){
				float speed = 0.4f;
				float speedL = -speed;
				float speedR = -speed;
				SetLeftMotors(speedL);
				SetRightMotors(speedR);
			}else if(robotYaw1 <= 22 && autonomousUsingState == AutonomousUsingState.ShootingRed){
				float speed = 0.4f;
				float speedL = speed;
				float speedR = speed;
				SetLeftMotors(speedL);
				SetRightMotors(speedR);
			}else{
				autoShootingState = AutoShootingState.CrossLine;
				ClearRotation();
			}		
		} else if(autoShootingState == AutoShootingState.CrossLine) {
			ShiftUp();
			//12293
			double encodersValue = Rencoder.get();
			double encoderTarget;
			encoderTarget = 12293;
			LogInfo("ENCODER "+Rencoder.get());
			if (encodersValue <= encoderTarget) {
				SetLeftMotors(1.0f);
				SetRightMotors(-1.0f);
			}else{
				autoShootingState = AutoShootingState.End;
			}
		} else if(autoShootingState == AutoShootingState.End){
			//do nothing
			ShiftUp();
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
				DriveStraight(0.3f, false);
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
				float speedL = speed;
				float speedR = speed;
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
			LogInfo("Ultrasonic " + ultrasonicTarget);
			if (ultrasonic.getValue() >= ultrasonicTarget) {
				DriveStraight(0.3f, false);
			}else{
				ClearRotation();
				autoHopperState = AutoHopperState.HopperSmash;
				smashTimer.start();
			}
		}else if (autoHopperState == AutoHopperState.HopperSmash){
			if (smashTimer.get() > 1.0f) {
				ShiftDown();
				SetLeftMotors(0.6f);
				SetRightMotors(-0.6f);
			} else {
				autoHopperState = AutoHopperState.BackUp;
			}
		}else if(autoHopperState == AutoHopperState.BackUp){
			double encodersValue = Rencoder.get();
			double encoderTarget;
			encoderTarget = 3175;
			LogInfo("Encoders " + encodersValue);
			if (encodersValue <= encoderTarget) {
				DriveStraight(-0.3f, true);
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
		}else if(autoHopperState == AutoHopperState.Fire){
			ShooterToggle(true, 3975, 3925, -1, -0.5);
		}
	}



	UpdateMotors();
	feederRPMControl.UpdateRPMControl();
	shooterRPMControl.UpdateRPMControl();
}

public void teleopInit() {
	SmartDashboard.putNumber("Four (Agitator)", -1.0f);
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
			relay.set(Relay.Value.kReverse);
		}else{
			relay.set(Relay.Value.kForward);
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
		if (xboxController.getRawButton(2)) {
			ShooterToggle(true, SmartDashboard.getNumber("One (shooter)"), SmartDashboard.getNumber("Two (feeder)"), -1, -1.0);
		} else {
			ShooterToggle(false, 0, 0, 0, 0);
		}
	}
	//Climber
	{
		climber.target = 0.0f;
		if (xboxController.getRawButton(4) || driveControllerR.getRawButton(8)) {
			System.out.println("SETTING");
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
		stirer.set(1.0f);
		//agitator.set(SmartDashboard.getNumber("Four (Agitator)"));
		//injector.set(SmartDashboard.getNumber("Three (Injector)"));
	} else {
		agitator.set(0.0f);
		injector.set(0.0f);
		stirer.set(0.0f);
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
