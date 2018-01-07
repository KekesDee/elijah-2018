package org.usfirst.frc.team1844.robot.subsystems;

import org.usfirst.frc.team1844.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Climber extends Subsystem {

	private SpeedControllerGroup m_motors;

	public Climber() {
		m_motors = new SpeedControllerGroup(new WPI_TalonSRX(RobotMap.CAN_CLIMBER_LEFTMOTOR),
				new WPI_TalonSRX(RobotMap.CAN_CLIMBER_RIGHTMOTOR));
	}

	public void initDefaultCommand() {
	}

	public void setSpeed(double speed) {
		m_motors.set(speed);
	}

	public void stop() {
		m_motors.stopMotor();
	}
}
