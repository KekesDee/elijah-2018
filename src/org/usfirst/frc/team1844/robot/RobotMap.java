/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1844.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	//MOTORS
	public static final int LeftDrive1 = 0;
	public static final int LeftDrive2 = 1;
	public static final int RightDrive1 = 2;
	public static final int RightDrive2 = 3;
	
	//ENCODERS
	public static final int LeftEncoder1 = 1;
	public static final int LeftEncoder2 = 2;
	public static final int RightEncoder1 = 3;
	public static final int RightEncoder2 = 4;
	
	// Let's establish a naming convention here.
	// Each constant will be of the format INTERFACE_SUBSYSTEM_COMPONENT.
	// Remember also that all of these should be public static final ints.
	
	// For example,	PWM_DRIVETRAIN_FLMOTOR
	//				CAN_SHOOTER_MOTOR
	//				DIO_SHOOTER_ENCODER
	//				PCM_SHOOTER_HOOD
	//				RELAY_VISION_LED
	
}
