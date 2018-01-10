package org.usfirst.frc.team1844.robot.subsystems;

import org.usfirst.frc.team1844.robot.RobotMap;
import org.usfirst.frc.team1844.robot.commands.climb.ClimbWithJoystick;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Climber extends Subsystem {

	private SpeedController m_motor;

	public Climber() {
		m_motor = new WPI_TalonSRX(RobotMap.CAN_CLIMBER_MOTOR);
		// m_motor.setInverted(true);
		// Apparently this doesn't work on WPI_TalonSRX
	}

	public void initDefaultCommand() {
		setDefaultCommand(new ClimbWithJoystick());
	}

	public void set(double speed) {
		m_motor.set(-speed);
	}

	public void stop() {
		m_motor.stopMotor();
	}
}
