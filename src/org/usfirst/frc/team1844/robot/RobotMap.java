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

	// Let's establish a naming convention here.
	// Each constant will be of the format INTERFACE_SUBSYSTEM_COMPONENT.
	// Remember also that all of these should be public static final ints.
	
	// For example,	PWM_DRIVETRAIN_FLMOTOR
	//				CAN_SHOOTER_MOTOR
	//				DIO_SHOOTER_ENCODER
	//				PCM_SHOOTER_HOOD
	//				RELAY_VISION_LED
	
	//Motors
	public static final int Motor_one = 7;
	public static final int Motor_two = 8;
	
	//Encoders
	public static final int Encoder_one = 6;

	
	
}
