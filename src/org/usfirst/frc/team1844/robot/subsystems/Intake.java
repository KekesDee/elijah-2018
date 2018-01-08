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
	// SpeedController m_topMotor;
	private DigitalInput m_cubeSwitchL;
	private DigitalInput m_cubeSwitchR;

	public Intake() {
		m_leftMotor = new WPI_TalonSRX(RobotMap.CAN_INTAKE_LEFTMOTOR);
		m_rightMotor = new WPI_TalonSRX(RobotMap.CAN_INTAKE_RIGHTMOTOR);
		// m_topMotor = new WPI_TalonSRX(RobotMap.CAN_INTAKE_TOP);
		
		m_cubeSwitchL = new DigitalInput(RobotMap.DIO_INTAKE_LIM_L);
		m_cubeSwitchR = new DigitalInput(RobotMap.DIO_INTAKE_LIM_R);
	}

	public void initDefaultCommand() {
	}

	public void setSpeed(double speed) {
		m_leftMotor.set(-speed);
		m_rightMotor.set(speed);
		// m_topMotor.set(speed);
	}

	public void stopMotors() {
		m_leftMotor.stopMotor();
		m_rightMotor.stopMotor();
		// m_topMotor.stopMotor();
	}

	public boolean getHasCube() {
		return !(m_cubeSwitchL.get() && m_cubeSwitchR.get());
	}
}
