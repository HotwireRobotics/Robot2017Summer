package org.usfirst.frc.team2990.robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Encoder;

import com.kauailabs.navx.frc.AHRS;


public class Robot extends IterativeRobot {
	
	public Joystick driveController;
	
	public AHRS navxDevice;	

	// MotorThree is the top motor, it must move opposite the others
	public JoshMotorControllor leftMotorOne;
	public JoshMotorControllor leftMotorTwo;
	public JoshMotorControllor leftMotorThree;
	public JoshMotorControllor drum;
	public JoshMotorControllor rightMotorOne;
	public JoshMotorControllor rightMotorTwo;
	public JoshMotorControllor rightMotorThree;
	
	public Joystick driveControllerL;
	public Joystick driveControllerR;	
	
	public Joystick xboxController;
	public boolean usingXbox = true;
	
	public DoubleSolenoid leftShifter;
	public DoubleSolenoid rightShifter;
	
	public Timer placeGear;
	
	// navx  zeroing correctly
	public boolean navxReady;
	public int loopCount;
	
	public Encoder Lencoder;
	public Encoder Rencoder;
	public Timer waitGear;
	public enum AutoStraightState
	{
		WaitingForNavx, PlaceGear, WaitGear, ReverseGear, Rotate, CrossLine
	}
	public AutoStraightState autoStraightState;
	
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		//driveControllerR = new Joystick(1);
		//driveControllerL = new Joystick(0);
		leftShifter = new DoubleSolenoid(0, 1);
		rightShifter = new DoubleSolenoid(2, 3);
		xboxController = new Joystick(0);
		waitGear = new Timer();
		Lencoder = new Encoder(0,1);
		Rencoder = new Encoder(2,3);
		
		float lerpSpeed = 0.2f;
		leftMotorOne = new JoshMotorControllor(1, lerpSpeed);
		leftMotorTwo = new JoshMotorControllor(3, lerpSpeed);
		leftMotorThree = new JoshMotorControllor(2, lerpSpeed);
		
		rightMotorOne = new JoshMotorControllor(8, lerpSpeed);
		rightMotorTwo = new JoshMotorControllor(7, lerpSpeed);
		rightMotorThree = new JoshMotorControllor(6, lerpSpeed);
		
		drum = new JoshMotorControllor(0, 0.2f);
		//drum.target = 0;
	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	public void autonomousInit() {
		navxDevice = new AHRS(SPI.Port.kMXP);
		navxDevice.reset();
		navxDevice.zeroYaw();
		
		Rencoder.reset();
		Lencoder.reset();
		
		
		
		ShiftUp();
		
		navxReady = false;
		loopCount = 0;
		
		autoStraightState = AutoStraightState.WaitingForNavx;
		placeGear = new Timer();
		
		
		placeGear.start();
	}
	

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {
		//System.out.println(Rencoder.get());
		
		SetLeftMotors(0.0f);
		SetRightMotors(0.0f);
		
		System.out.println(autoStraightState);
		
		if (autoStraightState == AutoStraightState.WaitingForNavx) {
			// do nothing wait for navx
			loopCount++;
			if  (loopCount > 7)
			{
				autoStraightState = AutoStraightState.PlaceGear;
			}
			
			navxDevice.zeroYaw();
			navxDevice.reset();
		} else if (autoStraightState == AutoStraightState.PlaceGear) {
			
			float encodersValue = Rencoder.get();
			System.out.println(encodersValue);
			
			
			float encoderTarget = 12900;
			if (encodersValue >= -encoderTarget && encodersValue <= encoderTarget) {
			
				float robotYaw = navxDevice.getYaw() / 10;
				
				float speed = 0.4f;
				float speedL = speed + (robotYaw / 2.0f);
				float speedR = speed - (robotYaw / 2.0f);
			
				SetLeftMotors(-speedL);
				SetRightMotors(speedR);
			}	
			else 
			{
				autoStraightState = AutoStraightState.WaitGear;
				waitGear.start();
			}
		} else if (autoStraightState == AutoStraightState.WaitGear){
			if(waitGear.get() > 1f)
			{
				autoStraightState = AutoStraightState.ReverseGear;
			}
		} else if(autoStraightState == AutoStraightState.ReverseGear) {
			float robotYaw = navxDevice.getYaw() / 10;
			
			float encodersValue = Rencoder.get();
			System.out.println(encodersValue);
			
			float encoderTarget = -6450;
			if (encodersValue <= encoderTarget) {
				System.out.println("Backing up");
				float speed = -0.4f;
				float speedL = speed + (robotYaw / 2.0f);
				float speedR = speed - (robotYaw / 2.0f);
				
				SetLeftMotors(-speedL);
				SetRightMotors(speedR);
			}
		}
		
		UpdateMotors();
	}

	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	public void teleopInit() {
		SmartDashboard.putBoolean("Arcade Drive", true);
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
	
		UpdateMotors();
		drum.UpdateMotor();
		float horJoystick = 0;
		float verJoystick = 0;
		
		if (SmartDashboard.getBoolean("Arcade Drive"))	
		{		
			// Arcade drive
			if (usingXbox)
			{
				horJoystick = (float) xboxController.getRawAxis(0);
				verJoystick = (float) xboxController.getRawAxis(5);
				
				
				drum.target = (0);
				if (xboxController.getRawAxis(3) > 0.5) {
					drum.target = (float) xboxController.getRawAxis(3);
				}
				else if (xboxController.getRawAxis(2) > 0.5) {
					drum.target = (float) -xboxController.getRawAxis(2);
				}
			}
			else
			{
				horJoystick = (float) driveControllerL.getRawAxis(0);
				verJoystick = (float) driveControllerR.getRawAxis(1);
			}
			
			SetLeftMotors(-verJoystick + horJoystick);
			SetRightMotors(verJoystick + horJoystick);
		}
		else
		{
			// Tank drive
			if (usingXbox)
			{
				horJoystick = (float) xboxController.getRawAxis(1);
				verJoystick = (float) xboxController.getRawAxis(5);
			}
			else
			{	
				horJoystick = (float) driveControllerL.getRawAxis(0);
				verJoystick = (float) driveControllerR.getRawAxis(1);
			}	
			
			SetLeftMotors(-horJoystick);
			SetRightMotors(verJoystick);
		}
		
		
		boolean shiftUp = false;
		boolean shiftDown = false;
		
		if (usingXbox)
		{
			if (xboxController.getRawButton(6))
			{
				shiftUp = true;
			}
			else if (xboxController.getRawButton(5))
			{
				shiftDown = true;
			}
		}
		else
		{
			if (driveControllerR.getRawButton(4))
			{
				shiftUp = true;
			}
			if (driveControllerR.getRawButton(5))
			{
				shiftDown = true;
			}
		}
		
		if (shiftUp)
		{		
			ShiftUp();
		}

		if (shiftDown)
		{
			ShiftDown();
		}
	}
	
	public void ShiftDown()
	{
		leftShifter.set(DoubleSolenoid.Value.kForward);
		rightShifter.set(DoubleSolenoid.Value.kForward);
	}
	
	public void ShiftUp()
	{
		leftShifter.set(DoubleSolenoid.Value.kReverse);
		rightShifter.set(DoubleSolenoid.Value.kReverse);	
	}
	
	public void SetRightMotors(float speed)
	{
		rightMotorOne.target = speed;
		rightMotorTwo.target = speed;
		rightMotorThree.target = -speed;
	}
	
	public void SetLeftMotors(float speed)
	{
		leftMotorOne.target = speed;
		leftMotorTwo.target = speed;
		leftMotorThree.target = -speed;
	}
	
	public void UpdateMotors()
	{
		leftMotorOne.UpdateMotor();		
		leftMotorTwo.UpdateMotor();		
		leftMotorThree.UpdateMotor();
	
		rightMotorOne.UpdateMotor();		
		rightMotorTwo.UpdateMotor();		
		rightMotorThree.UpdateMotor();
	}
	
	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		// LiveWindow.run();
	}
}
