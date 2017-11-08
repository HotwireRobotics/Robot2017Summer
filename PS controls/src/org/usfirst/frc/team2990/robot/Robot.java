package org.usfirst.frc.team2990.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;
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

import org.usfirst.frc.team2991.robot.JoshMotorControllor;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Compressor;

public class Robot extends IterativeRobot {
	public JoshMotorControllor leftMotorTop;
	public JoshMotorControllor leftMotorBottom;
	public JoshMotorControllor rightMotorTop;
	public JoshMotorControllor rightMotorBottom;
	public Joystick xboxController;
	public AHRS navx;
	public Encoder encoder;
	public boolean isPressed;
	public boolean buttonReleased;
	public boolean buttonPressed;
	public int target;
	public void robotInit(){
		float lerpSpeed = 0.2f;
		leftMotorTop = new JoshMotorControllor(8, lerpSpeed, false);
		leftMotorBottom = new JoshMotorControllor(9, lerpSpeed, true);
		rightMotorTop = new JoshMotorControllor(6, lerpSpeed, false);
		rightMotorBottom = new JoshMotorControllor(7, lerpSpeed,true);
		xboxController = new Joystick(1);
		encoder = new Encoder(2,3);
	}
	public void autonomousInit() {
	}

	public void autonomousPeriodic() {
		System.out.println("Auto");

	}
	public void teleopInit() {

	}

	public void teleopPeriodic() {
		System.out.println(encoder.get());
		if(xboxController.getRawButton(8)){
			encoder.reset();
		}
		if(xboxController.getRawButton(4)){
			System.out.println("BUTTON DOWN");
			SetLeftMotors(0);
			SetRightMotors(0);
			isPressed = true;
			if(isPressed = true){
				if(encoder.get() < 500){
					System.out.println("IN HERE");
					SetLeftMotors(.5);
					SetRightMotors(-.5);

				}if(encoder.get() >= 500){					
					isPressed = false;
				}
			}if(isPressed = false){
				SetLeftMotors(0);
				SetRightMotors(0);
				encoder.reset();
			}
		}

		if(xboxController.getRawButton(4)){
			buttonPressed= true;
		}
		if(xboxController.getRawButton(4) && buttonPressed == true){
			target = encoder.get() + 500;
			buttonReleased = false;
		}else if (!xboxController.getRawButton(4) && buttonPressed == true){
			buttonReleased = true;
		}
		SetLeftMotors(0);
		SetRightMotors(0);
		if(buttonReleased == true){
			if(encoder.get() < target){
				SetLeftMotors(.3);
				SetRightMotors(.3);
			}else{
				SetLeftMotors(0);
				SetRightMotors(0);
			}
		}
		SetLeftMotors(0);
		SetRightMotors(0);
		
		//if(xboxController.getRawButton(3));{
		//if(navx.getYaw() >){

		//}
		//}

		leftMotorTop.UpdateMotor();
		leftMotorBottom.UpdateMotor();
		rightMotorBottom.UpdateMotor();
		rightMotorTop.UpdateMotor();
	}
	public void SetLeftMotors(double d){
		leftMotorTop.target = (float) -d;
		leftMotorBottom.target = (float) d;
	}

	public void SetRightMotors(double d) {
		rightMotorTop.target = (float) -d;
		rightMotorBottom.target = (float) d;
	}
}


