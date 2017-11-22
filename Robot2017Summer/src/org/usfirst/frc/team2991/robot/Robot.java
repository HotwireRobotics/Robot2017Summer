package org.usfirst.frc.team2991.robot;

import org.usfirst.frc.team2991.robot.RIODroid.RIOadb;
import org.usfirst.frc.team2991.robot.RIODroid.RIOdroid;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;

public class Robot extends IterativeRobot implements PIDOutput {
	public DriveTrain drivetrain;
	public Joystick xbox360Controller;
	public Joystick xboxOneController;
	public Compressor compressor;
	public DoubleSolenoid gearDrop;
	public DoubleSolenoid driveShifter;
	public DoubleSolenoid gearShifter;
	public VictorSP climber1;
	public VictorSP climber2;
	CameraServer camera;
	Ultrasonic ultrasonic;
	AHRS navx;
	PIDController turnController;
	Encoder encoder;
	Timer GearDrop;
	float GobalSpeedAdjust = 1f;
	int AutoState = 0;
	Timer GearReverse;
	Boolean test;
	Timer navxPause;
	Timer failSafe;



	public void robotInit() {
		navx = new AHRS(SPI.Port.kMXP);
		//Sonar
		{
		ultrasonic = new Ultrasonic(9,8);
		ultrasonic.setEnabled(true);
		ultrasonic.setAutomaticMode(true);
		}
		//Turn Controllers
		{
		turnController = new PIDController(0.08, 0.0, 0.00020, 0, navx, (PIDOutput) this);
		turnController.setInputRange(-180.0f, 180.0f);
		turnController.setOutputRange(-1.0f, 1.0f);
		turnController.setAbsoluteTolerance(2.0);
		turnController.setContinuous(true);
		//turnController.disable();
		turnController.enable();
		}
		test = false;
		//PID
		{
		SmartDashboard.putDouble("P", turnController.getP());
		SmartDashboard.putDouble("I", turnController.getI());
		SmartDashboard.putDouble("D", turnController.getD());
		SmartDashboard.putBoolean("Testing Straight Driving", test);
		}
		//Timers
		{
			GearReverse = new Timer();
			GearDrop = new Timer();
			navxPause = new Timer();
			failSafe = new Timer();
		}

		//Drivetrain
		{
			drivetrain = new DriveTrain(1, 4, 2, 3, 6, 5);
		}
		//Controllers
		{
			xbox360Controller = new Joystick(0);
			xboxOneController = new Joystick(1);
		}
		//Neumatics
		{
			gearDrop = new DoubleSolenoid(4,5);
			driveShifter = new DoubleSolenoid(0,1);
			gearShifter = new DoubleSolenoid(2,3);
		}
		//Climber
		{
			climber1 = new VictorSP(5);
			climber2 = new VictorSP(1);
		}
		//Camera
		if (false)
		{
			camera = CameraServer.getInstance();
			UsbCamera usbCam = camera.startAutomaticCapture();
			usbCam.setResolution(250, 210);
			usbCam.setFPS(30);
		}


		//RIODroid
		{
			//RIOdroid.initUSB();
			//RIOadb.init();
		}
	}


	public void autonomousInit() {		
		failSafe.start();
		navx.zeroYaw();
		AutoState = 0;
		drivetrain.SetBreak();
		turnController.enable();
		navxPause.start();
	}


	public void autonomousPeriodic() {
		drivetrain.Update();
		LogInfo(""	+ "ultrasonic " + ultrasonic.getRangeMM());
		System.out.println(AutoState);
		if(navxPause.get() >= 1){
			if(AutoState == 0){
				if (ultrasonic.getRangeMM() >= 320) {
					System.out.println("Forward");
					ShiftDown();
					DriveStraight(.5f * GobalSpeedAdjust, false);
					if(failSafe.get() >= 6.0){
						AutoState = 3;
					}
				}else{
					drivetrain.SetLeftSpeed(0f);
					drivetrain.SetRightSpeed(0f);
					failSafe.stop();
					//not yet
					AutoState += 1;
					GearDrop.start();
				}
			}else if(AutoState == 1){
				System.out.println("Release");
				gearShifter.set(DoubleSolenoid.Value.kReverse);
				if(GearDrop.get() >= 1){
					AutoState += 1;
					GearReverse.start();
				}
			}else if(AutoState == 2){
				System.out.println("Backing Up");
				drivetrain.SetLeftSpeed(-.5f * GobalSpeedAdjust);
				drivetrain.SetRightSpeed(.5f * GobalSpeedAdjust);
				if(GearReverse.get() >= 2){
					drivetrain.SetLeftSpeed(0f);
					drivetrain.SetRightSpeed(0f);
					AutoState +=1;
				}
			}else if(AutoState == 3){
				drivetrain.SetLeftSpeed(0);
				drivetrain.SetRightSpeed(0);
				System.out.println("All Done");
			}
		}
	}

	public void teleopInit(){
		System.out.println("ADB " + RIOdroid.executeCommand("adb devices"));
		
		drivetrain.SetCoast();
	}

	public void teleopPeriodic() {
		PID();
		SmartDashboard.putNumber("Navx", navx.getYaw());
		SmartDashboard.putNumber("Sonar", ultrasonic.getRangeMM());
		//System.out.println(ultrasonic.getRangeMM());

		//Shifting
		{
			if (xbox360Controller.getRawButton(6)) {
				ShiftUp();
			}
			if (xbox360Controller.getRawButton(5)) {
				ShiftDown();
			}
		}
		//Gear Drop
		{
			if(xbox360Controller.getRawButton(4)){
				gearDrop.set(DoubleSolenoid.Value.kReverse);
			}else{
				gearDrop.set(DoubleSolenoid.Value.kForward);
			}
		}
		//Gear Gobbler
		{
			if(xbox360Controller.getRawAxis(2) > 0.5){
				gearShifter.set(DoubleSolenoid.Value.kReverse);
			}else{
				gearShifter.set(DoubleSolenoid.Value.kForward);
			}
			if(xboxOneController.getRawButton(3)){
				gearDrop.set(DoubleSolenoid.Value.kReverse);
			}else{
				gearDrop.set(DoubleSolenoid.Value.kForward);
			}
		}
		//Climber 80%
		{
			if(xboxOneController.getRawButton(8)){
				climber1.set(-0.9f);	
				climber2.set(-0.9f);	
			}else if(xboxOneController.getRawButton(7)){
				climber1.set(-0.2f);
				climber2.set(-0.2f);
			}else{
				climber1.set(0);
				climber2.set(0);
			}
		}

		//Drive Straight
		{
			if(test == true){
				if(xbox360Controller.getRawButton(1)){
					ShiftDown();
					DriveStraight(.5f, false);
				}else if(xbox360Controller.getRawButton(3)){
					ShiftDown();
					DriveStraight(-.5f, true);
				}else{
					DriveStraight(.0f, false);
				}
			}else{
				turnController.disable();
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

					drivetrain.SetLeftSpeed(verJoystick + horJoystick);
					drivetrain.SetRightSpeed(-verJoystick + horJoystick);
					drivetrain.Update();
				}   
			}
		}
		
		drivetrain.Update();
		
		//Navx reset
		{
			if(xbox360Controller.getRawButton(2)){
				//navx.zeroYaw();
			}
		}
		//Test
		{
			test = SmartDashboard.getBoolean("Testing Straight Driving");
		}

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

	public void ShiftDown() {
		driveShifter.set(DoubleSolenoid.Value.kReverse);
	}

	public void ShiftUp() {
		driveShifter.set(DoubleSolenoid.Value.kForward);
	}
	public void StraightAuto(){

	}
	public void LogInfo(String info) {
		System.out.println(info + ";    ");
	}
	public void DriveStraight(float speed, boolean reverse) {
		float pidError = (float)turnController.get();
		drivetrain.SetLeftSpeed((speed * pidError) + speed); //0.6972
		drivetrain.SetRightSpeed(((speed) - (speed * pidError)) * -1); //-0.583

		speed = -speed;
		if(reverse){
			speed = -speed;
		}

		LogInfo("STRAIGHT YAW " + navx.getYaw());
	}
	public void ClearRotation() {
		navx.zeroYaw();
		turnController.setSetpoint(0);
	}
	public void PID(){		
		float newP = (float)SmartDashboard.getDouble("P");
		float newI = (float)SmartDashboard.getDouble("I");
		float newD = (float)SmartDashboard.getDouble("D");

		turnController.setPID(newP, newI, newD);
	}


	@Override
	public void pidWrite(double output) {
		
	}

}

