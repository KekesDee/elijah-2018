/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1844.robot;

import  org.usfirst.frc.team1844.robot.commands.*;
import org.usfirst.frc.team1844.robot.RobotMap;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot
 */
public class OI {

	private XboxController driveStick = new XboxController(0);

	// We don't need to worry so much about magic numbers here, since we are keeping
	// all of the button code together
	  
	private Button AButton = new JoystickButton(driveStick, 0);
	private Button BButton = new JoystickButton(driveStick, 1);
  
	private Button intakeIn = new JoystickButton(driveStick, 4);
	private Button intakeOut = new JoystickButton(driveStick, 5);
  
	//x
	private Button go_to_switch = new JoystickButton(driveStick, 2);
	//y
	private Button go_to_scale = new JoystickButton(driveStick, 3);
	//start
	private Button go_to_origin = new JoystickButton(driveStick, 7);
	
	private static double LR_speed = 0.25;
	private static double LR_time_lim = 2;
	

	public OI() {
		// Here is where we bind buttons to commands using whileHeld, whenPressed, etc.
		
		/*the controlled lift command works like this:
		 *it moves (lowers/lift) the arm to the corresponding position of the button pressed, regardless of the current
		 *position of the arm
		*/
		go_to_switch.whenPressed(new ControlledLift (LR_speed, LR_time_lim, RobotMap.DIO_ARM_SWITCH));
		go_to_scale.whenPressed(new ControlledLift (LR_speed, LR_time_lim, RobotMap.DIO_ARM_SCALE));
		go_to_origin.whenPressed(new ControlledLift (LR_speed, LR_time_lim, RobotMap.DIO_ARM_ORIGIN));
    AButton.whenPressed(new AutoStraightDrive(24, 3));
		BButton.whenPressed(new TurntoAngle(90.0, 3.0));
   	intakeIn.whileHeld(new IntakeInWithX());
		intakeOut.whileHeld(new IntakeOutWithY());
	}

	public XboxController getDriverJoystick() {

		return driveStick;
	}
}
