package org.usfirst.frc.team2990.robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	
	public Joystick driveController;
	
	// MotorThree is the top motor, it must move opposite the others
	public JoshMotorControllor leftMotorOne;
	public JoshMotorControllor leftMotorTwo;
	public JoshMotorControllor leftMotorThree;
	
	public JoshMotorControllor rightMotorOne;
	public JoshMotorControllor rightMotorTwo;
	public JoshMotorControllor rightMotorThree;
	
	public Joystick driveControllerL;
	public Joystick driveControllerR;	
	
	public Joystick XboxController;
	public boolean UsingXbox = false;
	
	public DoubleSolenoid leftShifter;
	public DoubleSolenoid rightShifter;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		driveController = new Joystick(1);
		//leftTalon = new JoshMotorControllor(1, 0.2f);
		//rightTalon = new JoshMotorControllor(0, 0.2f);
		driveControllerL = new Joystick(0);
		driveControllerR = new Joystick(1);
		leftShifter = new DoubleSolenoid(0, 1);
		rightShifter = new DoubleSolenoid(2, 3);
		
		float lerpSpeed = 0.2f;
		leftMotorOne = new JoshMotorControllor(0, lerpSpeed);
		leftMotorTwo = new JoshMotorControllor(1, lerpSpeed);
		leftMotorThree = new JoshMotorControllor(2, lerpSpeed);
		
		rightMotorOne = new JoshMotorControllor(7, lerpSpeed);
		rightMotorTwo = new JoshMotorControllor(8, lerpSpeed);
		rightMotorThree = new JoshMotorControllor(9, lerpSpeed);
	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	public void autonomousInit() {

	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {

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
		
		leftMotorOne.UpdateMotor();
		leftMotorTwo.UpdateMotor();
		leftMotorThree.UpdateMotor();
		rightMotorOne.UpdateMotor();
		rightMotorTwo.UpdateMotor();
		rightMotorThree.UpdateMotor();
		
		if (SmartDashboard.getBoolean("Arcade Drive"))	
		{		
			// Arcade drive
			
			float horJoystick = (float) driveControllerL.getRawAxis(0);
			float verJoystick = (float) driveControllerR.getRawAxis(1);
	
			//leftTalon.target = -verJoystick + horJoystick;
			//rightTalon.target = verJoystick + horJoystick;
			
			leftMotorOne.target = -verJoystick + horJoystick;
			leftMotorTwo.target = -verJoystick + horJoystick;
			leftMotorThree.target = -(-verJoystick + horJoystick);
			
			rightMotorOne.target = verJoystick + horJoystick;
			rightMotorTwo.target = verJoystick + horJoystick;
			rightMotorThree.target = -(verJoystick + horJoystick);
		}
		else
		{
			// Tank drive
			
			float leftJoystick = (float) driveControllerL.getRawAxis(1);
			float rightJoystick = (float) driveControllerR.getRawAxis(1);
				
			leftMotorOne.target = leftJoystick;
			leftMotorTwo.target = leftJoystick;
			leftMotorThree.target = -leftJoystick;
			
			rightMotorOne.target = -rightJoystick;
			rightMotorTwo.target = -rightJoystick;
			rightMotorThree.target = rightJoystick;
		}
		
		if (driveControllerR.getRawButton(4))
		{		
			leftShifter.set(DoubleSolenoid.Value.kForward);
			rightShifter.set(DoubleSolenoid.Value.kForward);
		}
		else
		{
			//do nothing
		}

		if (driveControllerR.getRawButton(5))
		{
			leftShifter.set(DoubleSolenoid.Value.kReverse);
			rightShifter.set(DoubleSolenoid.Value.kReverse);
		}
		else
		{
			//do nothing
		}
	}
	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		// LiveWindow.run();
	}

}
