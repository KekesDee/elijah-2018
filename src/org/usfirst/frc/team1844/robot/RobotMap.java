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
	//
	
	// Motors
	public static final int CAN_DRIVE_FLMOTOR = 0;
	public static final int CAN_DRIVE_BLMOTOR = 1;
	public static final int CAN_DRIVE_FRMOTOR = 8;
	public static final int CAN_DRIVE_BRMOTOR = 9;
	
	public static final int CAN_INTAKE_LEFTMOTOR = 4;
	public static final int CAN_INTAKE_RIGHTMOTOR = 5;
	public static final int CAN_INTAKE_TOP = 50; // Don't think we'll have this
	
	public static final int CAN_ARM_LEFTMOTOR = 2;
	public static final int CAN_ARM_RIGHTMOTOR = 7;
	
	public static final int CAN_CLIMBER_LEFTMOTOR = 3;
	public static final int CAN_CLIMBER_RIGHTMOTOR = 6;
	
	// Encoders
	public static final int DIO_DRIVE_LEFTENC_A = 0;
	public static final int DIO_DRIVE_LEFTENC_B = 1;
	public static final int DIO_DRIVE_RIGHTENC_A = 2;
	public static final int DIO_DRIVE_RIGHTENC_B = 3;
	
	// Limit Switches
	public static final int DIO_ARM_BOTLIM = 4;
	public static final int DIO_ARM_TOPLIM = 5;
	public static final int DIO_INTAKE_LIM = 6;
	
}
