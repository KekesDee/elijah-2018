/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1844.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

	private XboxController driveStick = new XboxController(0);


	// We don't need to worry so much about magic numbers here, since we are keeping
	// all of the button code together
	private Button exampleButton = new JoystickButton(driveStick, 0);

	public OI() {
		// Here is where we bind buttons to commands using whileHeld, whenPressed, etc.
	}

	public XboxController getDriverJoystick() {
		return driveStick;
	}
}
