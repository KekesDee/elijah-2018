package org.usfirst.frc.team1844.robot.subsystems;

import org.usfirst.frc.team1844.robot.RobotMap;
import org.usfirst.frc.team1844.robot.commands.lift.LiftWithJoystick;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class LiftArm extends Subsystem {

	private WPI_TalonSRX m_leftMotor = new WPI_TalonSRX(RobotMap.CAN_ARM_LEFTMOTOR);
	private WPI_TalonSRX m_rightMotor = new WPI_TalonSRX(RobotMap.CAN_ARM_RIGHTMOTOR);
	private DigitalInput m_botSwitch;

	public LiftArm() {
		m_botSwitch = new DigitalInput(RobotMap.DIO_ARM_BOTLIM);
	}

	public void initDefaultCommand() {
		setDefaultCommand(new LiftWithJoystick());
	}

	public boolean getArmIsBottom() {
		return m_botSwitch.get();
	}

	public void set(double speed) {
		m_leftMotor.set(speed);
		m_rightMotor.set(-speed);
	}

	public void stop() {
		m_leftMotor.set(0);
		m_rightMotor.set(0);
	}
}
