package org.usfirst.frc.team1844.robot.subsystems;

import org.usfirst.frc.team1844.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Intake subsystem
 */
public class Intake extends Subsystem {

	private SpeedController m_leftMotor;
	private SpeedController m_rightMotor;
	private DigitalInput m_cubeSwitchL;
	private DigitalInput m_cubeSwitchR;

	public Intake() {
		m_leftMotor = new WPI_TalonSRX(RobotMap.CAN_INTAKE_LEFTMOTOR);
		m_rightMotor = new WPI_TalonSRX(RobotMap.CAN_INTAKE_RIGHTMOTOR);
		// m_rightMotor.setInverted(true);
		// Apparently this doesn't work on WPI_TalonSRX

		m_cubeSwitchL = new DigitalInput(RobotMap.DIO_INTAKE_LIM_L);
		m_cubeSwitchR = new DigitalInput(RobotMap.DIO_INTAKE_LIM_R);
	}

	public void initDefaultCommand() {
	}

	public void set(double speed) {
		m_leftMotor.set(speed);
		m_rightMotor.set(-speed);
	}

	public void stop() {
		m_leftMotor.stopMotor();
		m_rightMotor.stopMotor();
	}

	public boolean getHasCube() {
		return !(m_cubeSwitchL.get() && m_cubeSwitchR.get());
	}
}
