package org.usfirst.frc.team1844.robot.subsystems;

import org.usfirst.frc.team1844.robot.RobotConstants;
import org.usfirst.frc.team1844.robot.RobotMap;
import org.usfirst.frc.team1844.robot.commands.driveTrain.TeleopDrive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveTrain extends Subsystem {

	private SpeedControllerGroup m_leftMotors;
	private SpeedControllerGroup m_rightMotors;
	private PIDController m_leftPID;
	private PIDController m_rightPID;
	private Encoder m_leftEnc;
	private Encoder m_rightEnc;

	public void updateTunings() {
		m_leftPID.setPID(RobotConstants.driveP, RobotConstants.driveI, RobotConstants.driveD);
		m_rightPID.setPID(RobotConstants.driveP, RobotConstants.driveI, RobotConstants.driveD);
	}

	public DriveTrain() {
		// Setup the encoders
		m_leftEnc = new Encoder(RobotMap.DIO_DRIVE_LEFTENC_A, RobotMap.DIO_DRIVE_LEFTENC_B, true, EncodingType.k4X);
		m_rightEnc = new Encoder(RobotMap.DIO_DRIVE_RIGHTENC_A, RobotMap.DIO_DRIVE_RIGHTENC_B, true, EncodingType.k4X);
		m_leftEnc.setPIDSourceType(PIDSourceType.kDisplacement);
		m_rightEnc.setPIDSourceType(PIDSourceType.kDisplacement);
		m_leftEnc.setDistancePerPulse(RobotConstants.ENCODER_DISTANCE_PER_PULSE);
		m_rightEnc.setDistancePerPulse(RobotConstants.ENCODER_DISTANCE_PER_PULSE);

		// Setup the motor controllers
		m_leftMotors = new SpeedControllerGroup(new WPI_TalonSRX(RobotMap.CAN_DRIVE_FLMOTOR),
				new WPI_TalonSRX(RobotMap.CAN_DRIVE_BLMOTOR));
		m_leftMotors.setInverted(true);

		m_rightMotors = new SpeedControllerGroup(new WPI_TalonSRX(RobotMap.CAN_DRIVE_FRMOTOR),
				new WPI_TalonSRX(RobotMap.CAN_DRIVE_BRMOTOR));

		// Setup the PID controllers
		m_leftPID = new PIDController(RobotConstants.driveP, RobotConstants.driveI, RobotConstants.driveD, m_leftEnc,
				m_leftMotors);
		m_rightPID = new PIDController(RobotConstants.driveP, RobotConstants.driveI, RobotConstants.driveD, m_rightEnc,
				m_rightMotors);
		m_leftPID.setAbsoluteTolerance(1);
		m_rightPID.setAbsoluteTolerance(1);

		double maxSpeed = 0.5;
		m_leftPID.setOutputRange(-maxSpeed, maxSpeed);
		m_rightPID.setOutputRange(-maxSpeed, maxSpeed);
	}

	public void initDefaultCommand() {
		setDefaultCommand(new TeleopDrive());
	}

	public void stop() {
		m_leftMotors.set(0);
		m_rightMotors.set(0);
	}

	public void drive(double forwardSpeed, double turnSpeed) {
		m_leftMotors.set(forwardSpeed - turnSpeed);
		m_rightMotors.set(forwardSpeed + turnSpeed);
	}

	public void setTargetDistance(double targetDist) {
		m_leftPID.setSetpoint(targetDist + m_leftEnc.getDistance());
		m_rightPID.setSetpoint(-targetDist + m_rightEnc.getDistance());
	}

	public void setTargetAngle(double targetAngle) {
		double targetRad = Math.toRadians(targetAngle);
		double targetDist = targetRad * RobotConstants.RADIUS_OF_ROBOT;
		m_leftPID.setSetpoint(targetDist + m_leftEnc.getDistance());
		m_rightPID.setSetpoint(targetDist + m_rightEnc.getDistance());
	}

	public boolean getPIDOnTarget() {
		return m_leftPID.onTarget() && m_rightPID.onTarget();
	}

	public void enablePID() {
		m_leftPID.enable();
		m_rightPID.enable();
	}

	public void disablePID() {
		m_leftPID.disable();
		m_rightPID.disable();
	}

	public void resetPID() {
		m_leftPID.reset();
		m_rightPID.reset();
	}

	public void resetEncoders() {
		m_leftEnc.reset();
		m_rightEnc.reset();
	}

	public double getLeftEncRaw() {
		return m_leftEnc.get();
	}

	public double getRightEncRaw() {
		return m_rightEnc.get();
	}

	public double getLeftEncDist() {
		return m_leftEnc.getDistance();
	}

	public double getRightEncDist() {
		return m_rightEnc.getDistance();
	}
}