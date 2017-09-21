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
import edu.wpi.first.wpilibj.Ultrasonic;

public class Robot extends IterativeRobot implements PIDOutput {
	//SENSORS
	public Ultrasonic ultrasonic;
	//public Ultrasonic Rultrasonic;
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
	public Joystick xboxOneController;
	public Joystick xbox360Controller;

	//Shifting
	public DoubleSolenoid rightShifter;
	public DoubleSolenoid gearShifter;
	public boolean gearShifterReleased;
	public DoubleSolenoid gearDrop;

	//Encoders
	//public Encoder Lencoder;
	public Encoder Rencoder;

	//Timers
	public Timer autoPause;
	public Timer shooterPause;
	public double pauseTime = 0.5;
	public Timer smashTimer;
	public Timer escapeTimer;
	public Timer passiveTimer;

	// Shooter Stuff
	public RPMControl feederRPMControl;
	public RPMControl shooterRPMControl;
	public CANTalon injector;
	public Talon stirer;
	public Spark agitator;
	public Timer gearWait;

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
		WaitingForNavX, Forward, Rotate, Approach, BackUp, TurnTowardBoiler, Shoot, End, 
		TurnTowardHopper, BaragePause, AttackHopper, CollectBalls, Aim
	}
	// Shooting, then going near left peg
	public enum AutoShootingState {
		Pause, Forward, Turn, Fire, TurnTowardAirship, Approach, CrossLine, End
	} 
	public enum AutoHopperState {
		Pause, Forward, TurnTowardHopper, Approach, HopperSmash, BackUp, TurnTowardBoiler, Fire, End
	}
	public enum AutoForwardState{
		Pause, Forward, End
	}
	public AutoStraightState autoStraightState;
	public AutoRightState autoRightState;
	public AutoLeftState autoLeftState;
	public AutoShootingState autoShootingState;
	public AutoHopperState autoHopperState;
	public AutoForwardState autoForwardState;
	// Which autonomous program we're using. From the perspective of the drivers.
	public enum AutonomousUsingState {
		PegOnlyMiddleBlue, PegOnlyMiddleRed, PegOnlySideBlue, PegOnlySideRed, None, DriveStraightOnly, PegAndHopperRed,
		PegOnlySideBluePassive, PegOnlySideRedPassive,
	}
	public AutonomousUsingState autonomousUsingState = AutonomousUsingState.None;

	public SendableChooser autoSelection;
	public Relay relay;

	public void robotInit() {
		//Sensors
		{
			navxDevice = new AHRS(SPI.Port.kMXP);
			compressor = new Compressor();
			ultrasonic = new Ultrasonic(9,8);
			ultrasonic.setEnabled(true);
			ultrasonic.setAutomaticMode(true);

			relay = new Relay(0);
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
			cameraServo = new Servo(4);
			camera = CameraServer.getInstance();
			UsbCamera usbCam = camera.startAutomaticCapture(); 
			usbCam.setResolution(250,  210);
			usbCam.setFPS(30);
		}

		//Controllers
		{
			xboxOneController = new Joystick(2);
			driveControllerR = new Joystick(1);
			driveControllerL = new Joystick(0);	
			xbox360Controller = new Joystick(3);
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
			leftMotorTop = new JoshMotorControllor(12, lerpSpeed, false);
			leftMotorBottom = new JoshMotorControllor(9, lerpSpeed, false);
			
			rightMotorTop = new JoshMotorControllor(15, lerpSpeed, false);
			rightMotorBottom = new JoshMotorControllor(7, lerpSpeed,false);

			climber = new JoshMotorControllor(5, lerpSpeed, false);	
			
			agitator = new Spark(3);
			//stirer = new Talon(2);
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
			//stirer.set(0.0f);

		}

		gearWait = new Timer();
		escapeTimer = new Timer();
		passiveTimer = new Timer();

		SmartDashboard.putNumber("Initial Pause", pauseTime);
		SmartDashboard.putBoolean("Show Diagnostics", false);

		// These are from the perspective of the driver!
		{
			autoSelection = new SendableChooser();
			autoSelection.addObject("Active Blue Side Peg", AutonomousUsingState.PegOnlySideBlue);
			autoSelection.addObject("Passive Blue Side Peg", AutonomousUsingState.PegOnlySideBluePassive);
			autoSelection.addObject("Active Red Side Peg", AutonomousUsingState.PegOnlySideRed);
			autoSelection.addObject("Passive Red Side Peg", AutonomousUsingState.PegOnlySideRedPassive);
			autoSelection.addObject("Drive Straight", AutonomousUsingState.DriveStraightOnly);
			SmartDashboard.putData("Autonomous Side Selection", autoSelection);
		}
	}

	public void autonomousInit() {
		LogInfo("AUTO INIT");

		ShooterToggle(false, 0, 0, 0);

		//Initializations

		autoPause = new Timer();
		//State inits
		autoForwardState = AutoForwardState.Pause;
		autoStraightState = AutoStraightState.WaitingForNavX;
		autoRightState = AutoRightState.WaitingForNavX;
		autoLeftState = AutoLeftState.WaitingForNavX;
		autoShootingState = AutoShootingState.Pause;
		autoHopperState = AutoHopperState.Pause;
		//Shifter inits
		ShiftDown();
		if(autonomousUsingState == AutonomousUsingState.PegOnlySideBluePassive)
		{
			gearShifter.set(DoubleSolenoid.Value.kForward);
		}else{
			//gearShifter.set(DoubleSolenoid.Value.kReverse);
		}
		if(autonomousUsingState == AutonomousUsingState.PegOnlySideRedPassive)
		{
			gearShifter.set(DoubleSolenoid.Value.kForward);
		}
		//Zeroing of sensors
		ultrasonic.setAutomaticMode(true);
		navxDevice.zeroYaw();
		Rencoder.reset();
		//Lencoder.reset();

		//SmartDashboard Auto selections
		//Get SDB Inputs
		pauseTime = SmartDashboard.getNumber("Initial Pause");
		LogInfo("PauseTimer" + pauseTime);
		if (pauseTime <= 0.2) {
			pauseTime = 0.3;
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
					turnController.disable();
				} else {
					navxDevice.zeroYaw();
					if (!turnController.isEnabled()) {
						turnController.enable();
					}
				}

			} else if (autoStraightState == AutoStraightState.PlaceGear) {

				float encodersValue = Rencoder.get();				
				LogInfo(""	+ "ultrasonic " + ultrasonic.getRangeMM());
				if (ultrasonic.getRangeMM() >= 290) {
					DriveStraight(0.5f, false);
				} else { 
					ClearRotation();
					autoStraightState = AutoStraightState.BackUp;
					gearDrop.set(DoubleSolenoid.Value.kForward);
					gearWait.start();

				}
			}else if(autoStraightState == AutoStraightState.BackUp){
				LogInfo("Encoders:"  + Rencoder.get());
				float encodersValue = Rencoder.get();				
				if(gearWait.get() >= 0.5){
					if(encodersValue >= 10000){
						float speed = -0.4f;
						float speedL = speed;
						float speedR = -speed;
						SetLeftMotors(speedL);
						SetRightMotors(speedR);
					}else{
						autoStraightState = AutoStraightState.Rotate;
					}
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
					if(navxDevice.getYaw() >= -51){
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
				if(autonomousUsingState == AutonomousUsingState.PegOnlyMiddleRed){
					ShooterToggle(true, 4350, 4350, -1);
					gearDrop.set(DoubleSolenoid.Value.kReverse);
				}else if(autonomousUsingState == AutonomousUsingState.PegOnlyMiddleBlue){
					ShooterToggle(true, 4400, 4350, -1);
					gearDrop.set(DoubleSolenoid.Value.kReverse);
				}

			}else if(autoStraightState == AutoStraightState.ApproachBoiler){
				LogInfo("Encoders:"  + Rencoder.get());
				gearDrop.set(DoubleSolenoid.Value.kReverse);
				if(autonomousUsingState == AutonomousUsingState.PegOnlyMiddleBlue){
					float encodersValue = Rencoder.get();				
					if(encodersValue >= -8900){
						//SonarCheck();
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
					if(encodersValue >= -600){
						//SonarCheck();
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
					if(robotYaw1 >= -40){
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
					//SonarCheck();
					LogInfo("Encoders:"  + Rencoder.get());
					float encodersValue = Rencoder.get();				
					if(encodersValue <= 12000){
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
					//SonarCheck();
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
		} else if (autonomousUsingState == AutonomousUsingState.PegOnlySideBlue || autonomousUsingState == AutonomousUsingState.PegOnlySideRed || 
				autonomousUsingState == AutonomousUsingState.PegOnlySideRedPassive || autonomousUsingState == AutonomousUsingState.PegOnlySideBluePassive) {

			LogInfo("Side State - " + autoLeftState);

			if (autoLeftState == AutoLeftState.WaitingForNavX) {
				if(autonomousUsingState == AutonomousUsingState.PegOnlySideRedPassive || autonomousUsingState == AutonomousUsingState.PegOnlySideBluePassive)
				{
					gearShifter.set(DoubleSolenoid.Value.kForward);
				}
					LogInfo("Pause Time:" + pauseTime);
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
				if(autonomousUsingState == AutonomousUsingState.PegOnlySideBlue || autonomousUsingState == AutonomousUsingState.PegOnlySideBluePassive){
					float encoderTargetR = 12100;
					if (encodersValue <= encoderTargetR) {
						DriveStraight(1.0f, false);
					}else{ 
						autoLeftState = AutoLeftState.Rotate;
						ClearRotation();
					}
				}else if(autonomousUsingState == AutonomousUsingState.PegOnlySideRed
						|| autonomousUsingState == AutonomousUsingState.PegOnlySideRedPassive){
					float encoderTargetR = 13500;
					if (encodersValue <= encoderTargetR) {
						DriveStraight(1.0f, false);
					}else{ 
						autoLeftState = AutoLeftState.Rotate;
						ClearRotation();
					}
				}
			} else if(autoLeftState == AutoLeftState.Rotate){
				if(autonomousUsingState == AutonomousUsingState.PegOnlySideBlue || autonomousUsingState == AutonomousUsingState.PegOnlySideBluePassive){
					float target = 45.5f;
					float robotYaw1 = navxDevice.getYaw();
					LogInfo("Yaw " + robotYaw1);
					if(robotYaw1 < target) 
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
						passiveTimer.start();
					}
				}else if(autonomousUsingState == AutonomousUsingState.PegOnlySideRed || autonomousUsingState == AutonomousUsingState.PegOnlySideRedPassive){
					float target = -54;
					float robotYaw1 = navxDevice.getYaw();
					LogInfo("Yaw " + robotYaw1);
					if(robotYaw1 > target) 
					{
						float speed = 0.25f;
						float speedL = -speed;
						float speedR = -speed;

						SetLeftMotors(speedL);
						SetRightMotors(speedR);
					} else {
						Rencoder.reset();
						autoLeftState = AutoLeftState.Approach;
						ClearRotation();
						passiveTimer.start();
					}
				}
			} else if(autoLeftState == AutoLeftState.Approach){
				if (ultrasonic.getRangeMM() >= 200){
					LogInfo("Ultrasonic " + ultrasonic.getRangeMM());
					float speed = 0.25f;
					float speedL = speed;
					float speedR = -speed;
					SetLeftMotors(speedL);
					SetRightMotors(speedR);
				} else {
					if(autonomousUsingState == AutonomousUsingState.PegOnlySideBluePassive || autonomousUsingState == AutonomousUsingState.PegOnlySideRedPassive) {
						if (passiveTimer.get() > 5) {
							autoLeftState = AutoLeftState.BackUp;
							ClearRotation();
							gearWait.start();
						}
					} else {
						autoLeftState = AutoLeftState.BackUp;
						gearDrop.set(DoubleSolenoid.Value.kForward);
						ClearRotation();
						gearWait.start();	
					}
				}				
			} else if(autoLeftState == AutoLeftState.BackUp){
				LogInfo("Encoders:"  + Rencoder.get());
				float encodersValue = Rencoder.get();
				if(gearWait.get() >= 1.0){
					if(encodersValue >= -50){
						float speed = 0.30f;
						float speedL = -speed;
						float speedR = speed;
						SetLeftMotors(speedL);
						SetRightMotors(speedR);
					}else{
						if (autonomousUsingState == AutonomousUsingState.PegAndHopperRed /*|| autonomousUsingState == AutonomousUsingState.PegAndHopperBlue*/) {
							autoLeftState = AutoLeftState.TurnTowardHopper;
						} else {
							autoLeftState = AutoLeftState.TurnTowardBoiler;
						} 
						ClearRotation();
					}
				}
			}else if((autoLeftState == AutoLeftState.TurnTowardBoiler && autonomousUsingState == AutonomousUsingState.PegOnlySideBlue) ||
					(autoLeftState == AutoLeftState.TurnTowardBoiler && autonomousUsingState == AutonomousUsingState.PegOnlySideBluePassive)){
				LogInfo("Yaw:" + navxDevice.getYaw());
				if(navxDevice.getYaw() >= -3){
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
			}else if((autoLeftState == AutoLeftState.TurnTowardBoiler && autonomousUsingState == AutonomousUsingState.PegOnlySideRed)||
					(autoLeftState == AutoLeftState.TurnTowardBoiler && autonomousUsingState == AutonomousUsingState.PegOnlySideRedPassive)){
				LogInfo("Yaw:" + navxDevice.getYaw());
				if(navxDevice.getYaw() <= 3.5){
					float speed = 1f;
					float speedL = speed;
					float speedR = speed;
					SetLeftMotors(speedL);
					SetRightMotors(speedR);
				}else{
					autoLeftState = AutoLeftState.Shoot;
					ClearRotation();	
				}
			}else if(autoLeftState == AutoLeftState.Shoot){
				ShiftUp();
				if(autonomousUsingState == AutonomousUsingState.PegOnlySideRed || 
						autonomousUsingState == AutonomousUsingState.PegOnlySideRedPassive){
					ShooterToggle(true, 3900, 4100, -1);
					//gearDrop.set(DoubleSolenoid.Value.kReverse);
				}else if(autonomousUsingState == AutonomousUsingState.PegOnlySideBlue || 
						autonomousUsingState == AutonomousUsingState.PegOnlySideBluePassive){
					ShooterToggle(true, 4100, 4100, -1);
					//gearDrop.set(DoubleSolenoid.Value.kReverse);
				}
			}else if(autoLeftState == AutoLeftState.End){
				ClearRotation();
				ShiftUp();;
				//do nothing 
			}else if(autoLeftState == AutoLeftState.TurnTowardHopper && autonomousUsingState == AutonomousUsingState.PegAndHopperRed){
				LogInfo("Yaw:" + navxDevice.getYaw());
				if(navxDevice.getYaw() >= -43){
					float speed = -0.5f;
					float speedL = speed;
					float speedR = speed;
					SetLeftMotors(speedL);
					SetRightMotors(speedR);
				}else{
					autoLeftState = AutoLeftState.AttackHopper;
					ClearRotation();
				}
			}else if(autoLeftState == AutoLeftState.BaragePause){
				if (smashTimer.get() < .5) {
					ClearRotation();
				}else{
					autoLeftState = AutoLeftState.AttackHopper;
				}
			}else if(autoLeftState == AutoLeftState.AttackHopper && autonomousUsingState == AutonomousUsingState.PegAndHopperRed){
				float encodersValue = Rencoder.get();
				System.out.println("Encoders " + encodersValue);
				float encoderTargetR = -10000;
				float speedL = 0.5f;
				float speedR = 0.5f;
				if (encodersValue >= encoderTargetR) {
					//DriveStraight(-0.5f, false);
					SetLeftMotors(-speedL);
					SetRightMotors(speedR);
				}else{ 
					autoLeftState = AutoLeftState.CollectBalls;
					ClearRotation();
					smashTimer.start();
				}
			}else if(autoLeftState == AutoLeftState.CollectBalls){
				float speedR = 0.5f;
				if (smashTimer.get() < .5) {
					ClearRotation();
					SetRightMotors(speedR);
				}else{
					autoLeftState = AutoLeftState.End;
				}
			}

		}else if(autonomousUsingState == AutonomousUsingState.DriveStraightOnly){
			LogInfo("Forward State - " + autoForwardState);
			if(autoForwardState == AutoForwardState.Pause) {
				if (autoPause.get() >= pauseTime) {
					autoForwardState = AutoForwardState.Forward;
				} else {
					navxDevice.zeroYaw();
					if (!turnController.isEnabled()) {
						turnController.enable();
					}
				}
			}else if(autoForwardState == AutoForwardState.Forward){
				LogInfo("Encoders:" + Rencoder.get());
				if(Rencoder.get() <= 14500){
					DriveStraight(0.5f, true);
				}else{
					autoForwardState = AutoForwardState.End;
				}
			}else if(autoForwardState == AutoForwardState.End){
				//do nothing
			}
		}


		UpdateMotors();
		feederRPMControl.UpdateRPMControl();
		shooterRPMControl.UpdateRPMControl();
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
			//LogInfo("Right Encoder : "  + Rencoder.get());
			//LogInfo("Left Encoder : "  + Lencoder.get());
			LogInfo("Ultrasonic : "  + ultrasonic.getRangeMM());
			SmartDashboard.putNumber("Left Utrasonic:", ultrasonic.getRangeMM());
			SmartDashboard.putNumber("Right Encoder:", Rencoder.get());
			//SmartDashboard.putNumber("Left Encoder:", Lencoder.get());
			SmartDashboard.putNumber("Navx:", navxDevice.getYaw());
		}

		//LogInfo("Ultrasonic:" + ultrasonic.getValue());
		//Update Motors
		{
			UpdateMotors();
			feederRPMControl.UpdateRPMControl();
			shooterRPMControl.UpdateRPMControl();
			climber.UpdateMotor();
		}

		if (false)
		{
			/*
			if(ultrasonic.getValue() <= 92){
				relay.set(Relay.Value.kForward);
			}else{
				relay.set(Relay.Value.kReverse);
			}
			 */
		}
		//Logictechs
		//Driving
		{
			float horJoystick = 0;
			float verJoystick = 0;

			float epsilon = 0.2f;
			float leftInput = TranslateController((float)xbox360Controller.getRawAxis(0));
			if (leftInput > epsilon || leftInput < -epsilon) {
				horJoystick = leftInput;
			}
			float rightInput = TranslateController((float)xbox360Controller.getRawAxis(5));
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
		//Shifting
		{
			if (xbox360Controller.getRawAxis(3) > 0) {
				ShiftUp();
			}
			if (xbox360Controller.getRawAxis(2) > 0) {
				ShiftDown();
			}
		}

		//xBox
		//Gear Gobbler neumatic
		{
			if ((xbox360Controller.getRawButton(5) && gearShifterReleased) || (xboxOneController.getRawButton(5) && gearShifterReleased)) {
				if(gearShifter.get() == DoubleSolenoid.Value.kReverse) {
					gearShifterReleased = false;
					gearShifter.set(DoubleSolenoid.Value.kForward);
				} else {
					gearShifterReleased =  false;
					gearShifter.set(DoubleSolenoid.Value.kReverse);
				}
			}
			if (!xboxOneController.getRawButton(5) && !xbox360Controller.getRawButton(5)) {
				gearShifterReleased = true;
			}
		}
		//Shooting Toggle
		{
			if (xboxOneController.getRawButton(2) || xbox360Controller.getRawButton(8) ) {
				//ShooterToggle(true, SmartDashboard.getNumber("One (shooter)"), SmartDashboard.getNumber("Two (feeder)"), -1, -1.0);
				ShooterToggle(true, 4300, 4000, -1);
			} else {
				ShooterToggle(false, 0, 0, 0);
			}
		}
		//Climber
		{
			climber.target = 0.0f;
			if (xboxOneController.getRawButton(4) || xbox360Controller.getRawButton(9)) {
				climber.target = 1;
			}
		}

		//Gear Drop
		{
			if(xbox360Controller.getRawButton(6)){
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

	public void ShooterToggle(boolean toggle, double motorOne, double motorTwo, double motorThree) {
		shooterRPMControl.running = toggle;
		feederRPMControl.running = toggle;

		shooterRPMControl.targetRPM = (float)motorOne;
		feederRPMControl.targetRPM = (float)motorTwo;

		//LogInfo("SHOOTER " + (float)motorOne);
		//LogInfo("FEEDER " + (float)motorTwo);

		if (toggle) {
			injector.set(motorThree);
			//stirer.set(1.0f);
			//agitator.set(SmartDashboard.getNumber("Four (Agitator)"));
			//injector.set(SmartDashboard.getNumber("Three (Injector)"));
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

	}
}
