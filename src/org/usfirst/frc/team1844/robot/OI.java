/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1844.robot;

import org.usfirst.frc.team1844.robot.commands.driveTrain.AutoStraightDrive;
import org.usfirst.frc.team1844.robot.commands.driveTrain.TurnToAngle;
import org.usfirst.frc.team1844.robot.commands.intake.RunIntake;
import org.usfirst.frc.team1844.robot.commands.intake.IntakeUntilSwitch;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot
 */
public class OI {

	private XboxController m_driveStick = new XboxController(0);

	// We don't need to worry so much about magic numbers here, since we are keeping
	// all of the button code together

	private Button L1 = new JoystickButton(m_driveStick, 4);				// L1 
	private Button R1 = new JoystickButton(m_driveStick, 5);				// R1 
	private Button intakeIn = new JoystickButton(m_driveStick, 0);			// A 
	private Button intakeIn_Backup = new JoystickButton(m_driveStick, 2);	// X
	private Button intakeOutSlow = new JoystickButton(m_driveStick, 1);		// B
	private Button intakeOutFast = new JoystickButton(m_driveStick, 3);		// Y

	public OI() {
		// Here is where we bind buttons to commands using whileHeld, whenPressed, etc.

		intakeIn.whileHeld(new IntakeUntilSwitch(0.8));
		intakeIn_Backup.whileHeld(new RunIntake(0.8));
		intakeOutSlow.whileHeld(new RunIntake(-0.3));
		intakeOutFast.whileHeld(new RunIntake(-0.9));

		L1.whenPressed(new AutoStraightDrive(24));
		R1.whenPressed(new TurnToAngle(90.0));
	}

	public XboxController getDriverJoystick() {
		return m_driveStick;
	}
}
