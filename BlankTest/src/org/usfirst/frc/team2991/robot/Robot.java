package org.usfirst.frc.team2991.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	public Joystick cont;

	public DoubleSolenoid solOne;
	public DoubleSolenoid solTwo;
	public DoubleSolenoid solThree;
	
	public JoshMotorControllor motor;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		solOne = new DoubleSolenoid(0, 1);
		solTwo = new DoubleSolenoid(2, 3);
		solThree = new DoubleSolenoid(4, 5);
		
		cont = new Joystick(0);
		
		motor = new JoshMotorControllor(6, 0.5f, false);
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {

	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {

	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		motor.UpdateMotor();
		
		motor.target = 0.0f;
		if (cont.getRawButton(1))
		{
			motor.target = 0.4f;
			//solOne.set(DoubleSolenoid.Value.kForward);
			//solTwo.set(DoubleSolenoid.Value.kForward);
			//solThree.set(DoubleSolenoid.Value.kForward);
		} else
		{
			solOne.set(DoubleSolenoid.Value.kReverse);
			solTwo.set(DoubleSolenoid.Value.kReverse);
			solThree.set(DoubleSolenoid.Value.kReverse);
		}
		
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
}

