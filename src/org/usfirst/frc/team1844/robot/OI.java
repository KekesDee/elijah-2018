/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1844.robot;

import org.usfirst.frc.team1844.robot.commands.intake.RunIntake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot
 */
public class OI {

	private XboxController m_driveStick = new XboxController(0);

	private Button intakeInSlow = new JoystickButton(m_driveStick, 1);	// A
	private Button intakeInFast = new JoystickButton(m_driveStick, 3);	// X
	private Button intakeOutSlow = new JoystickButton(m_driveStick, 2);	// B
	private Button intakeOutFast = new JoystickButton(m_driveStick, 4);	// Y
	
	public OI() {
		// Here is where we bind buttons to commands using whileHeld, whenPressed, etc.

		intakeInSlow.whileHeld(new RunIntake(0.3));
		intakeInFast.whileHeld(new RunIntake(0.8));
		intakeOutSlow.whileHeld(new RunIntake(-0.3));
		intakeOutFast.whileHeld(new RunIntake(-0.9));
	}

	public XboxController getDriverJoystick() {
		return m_driveStick;
	}
}
