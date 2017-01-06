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
	
	public Joystick xboxController;
	public boolean usingXbox = false;
	
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
		xboxController = new Joystick(0);
		
		float lerpSpeed = 0.2f;
		leftMotorOne = new JoshMotorControllor(1, lerpSpeed);
		leftMotorTwo = new JoshMotorControllor(3, lerpSpeed);
		leftMotorThree = new JoshMotorControllor(2, lerpSpeed);
		
		rightMotorOne = new JoshMotorControllor(8, lerpSpeed);
		rightMotorTwo = new JoshMotorControllor(7, lerpSpeed);
		rightMotorThree = new JoshMotorControllor(6, lerpSpeed);
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
		
		float horJoystick = 0;
		float verJoystick = 0;
		
		if (SmartDashboard.getBoolean("Arcade Drive"))	
		{		
			// Arcade drive
			
			
			if (usingXbox)
			{
				horJoystick = (float) xboxController.getRawAxis(0);
				verJoystick = (float) xboxController.getRawAxis(5);
			}
			else
			{
				horJoystick = (float) driveControllerL.getRawAxis(0);
				verJoystick = (float) driveControllerR.getRawAxis(1);
			}
	
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
			leftMotorOne.target = -horJoystick;
			leftMotorTwo.target = -horJoystick;
			leftMotorThree.target = horJoystick;
			
			rightMotorOne.target = verJoystick;
			rightMotorTwo.target = verJoystick;
			rightMotorThree.target = -verJoystick;
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
			leftShifter.set(DoubleSolenoid.Value.kForward);
			rightShifter.set(DoubleSolenoid.Value.kForward);
		}
		else
		{
			//do nothing
		}

		if (shiftDown)
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
