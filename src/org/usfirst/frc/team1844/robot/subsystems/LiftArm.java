package org.usfirst.frc.team1844.robot.subsystems;

import org.usfirst.frc.team1844.robot.RobotMap;
import org.usfirst.frc.team1844.robot.commands.lift.LiftWithJoystick;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class LiftArm extends Subsystem {

	public static enum ArmPositions {
		kBottomPos, kScalePos, kSwitchPos
	}

	

	WPI_TalonSRX left = new WPI_TalonSRX(RobotMap.CAN_ARM_LEFTMOTOR);
	WPI_TalonSRX right = new WPI_TalonSRX(RobotMap.CAN_ARM_RIGHTMOTOR);

	private DigitalInput m_botSwitch, m_topSwitch;

	// private DigitalInput switch_pos;
	// private DigitalInput scale_pos;
	// private boolean at_switch, at_scale;
	// private double time_to_switch, time_to_scale, time_scale_to_switch;
	// private Timer timer;
	// private boolean start_timer;

	// if we are using encoders:
	// protected Encoder lmenc;
	// private PIDController LMPID;

	public LiftArm() {

		// at_switch = false;
		// at_scale = false;
		// start_timer = false;
		// time_to_switch = 0;
		// time_to_scale = 0;
		// time_scale_to_switch = 0;
		// Break_PID = false;

		// lmenc = new Encoder (RobotMap.Lift_Motor_one, RobotMap.Lift_Motor_two, false,
		// EncodingType.k4X);

		WPI_TalonSRX left = new WPI_TalonSRX(RobotMap.CAN_ARM_LEFTMOTOR);
		WPI_TalonSRX right = new WPI_TalonSRX(RobotMap.CAN_ARM_RIGHTMOTOR);

		m_botSwitch = new DigitalInput(RobotMap.DIO_ARM_BOTLIM);
		m_topSwitch = new DigitalInput(RobotMap.DIO_ARM_TOPLIM);

		// switch_pos = new DigitalInput(RobotMap.DIO_ARM_SWITCH);
		// scale_pos = new DigitalInput(RobotMap.DIO_ARM_SCALE);

		// timer = new Timer();
		// LMPID = new PIDController (Kp, Ki, Kd, lmenc, LMGroup);
		// LMPID.setAbsoluteTolerance(1);
		// LMPID.setOutputRange(-1.0, 1.0);
	}

	public void initDefaultCommand() {
		setDefaultCommand(new LiftWithJoystick());
	}

	public void setSpeed(double speed) {
		// if (speed < 0 && m_botSwitch.get())
		// m_motors.set(0);
		// else if (speed > 0 && m_topSwitch.get())
		// m_motors.set(0);
		// else
		left.set(speed);
		right.set(-speed);
	}

	public double getSpeed() {
		return left.get();
	}

	public void stop() {
		left.set(0);
		right.set(0);
	}

	// public boolean lift_to_pos(ArmPositions targetPos, double speed) {
	//
	// if (targetPos == ArmPositions.kSwitchPos) {
	//
	// // from origin up to switch
	// if (!at_scale && !at_switch)
	// setSpeed(speed);
	// // from scale lower to switch
	// else if (!at_switch && at_scale)
	// setSpeed(-speed);
	// else
	// return true;
	//
	// // since we do not want to reset timer every time in the execute in command
	// if (!start_timer) {
	// timer.reset();
	// start_timer = true;
	// }
	//
	// // assume limit switch setup as false from default
	// if (switch_pos.get()) {
	// stop();
	//
	// if (!at_scale && !at_switch)
	// time_to_switch = timer.get();
	// else if (!at_switch && at_scale)
	// time_scale_to_switch = timer.get();
	//
	// at_switch = true;
	// at_scale = false;
	// start_timer = false;
	// }
	// }
	//
	// else if (targetPos == ArmPositions.kScalePos) {
	// if (at_scale && !at_switch)
	// return true;
	//
	// // from origin up to scale && from switch up to scale, both are up!
	// setSpeed(speed);
	//
	// if (!start_timer) {
	// timer.reset();
	// start_timer = true;
	// }
	//
	// // assume limit switch setup as false from default
	// if (switch_pos.get()) {
	// stop();
	//
	// if (!at_scale && !at_switch)
	// time_to_scale = timer.get();
	// else if (at_switch && !at_scale)
	// time_scale_to_switch = timer.get();
	//
	// at_switch = false;
	// at_scale = true;
	// start_timer = false;
	// }
	// }
	//
	// else if (targetPos == ArmPositions.kBottomPos) {
	// if (!start_timer) {
	// timer.reset();
	// start_timer = true;
	// }
	//
	// setSpeed(-speed);
	//
	// if (at_scale) {
	// if (timer.get() >= time_to_scale) {
	// stop();
	// at_scale = false;
	// start_timer = false;
	// }
	// }
	//
	// else if (at_switch) {
	// if (timer.get() >= time_to_switch) {
	// stop();
	// at_scale = false;
	// start_timer = false;
	// }
	// }
	// }
	//
	// return true;
	// }

	/*
	 * private double Compute_PID (double desired_outut, double actual_output,
	 * double delta_t) { err = desired_outut - actual_output; i_err = i_err + err *
	 * delta_t; d_err = (prev_err - err) / delta_t; prev_err = err;
	 * 
	 * return Kp * err + Ki * i_err + Kd * d_err + bias; }
	 */

	/*
	 * public void lift(double speed, double move_angle, boolean reverse, double
	 * delta_t) { double prev_t = 0, prev_enc = 0;
	 * 
	 * lmenc.reset(); lmenc.setDistancePerPulse(ENC_ANG_PER_PULSE);
	 * 
	 * if(reverse) lmenc.setReverseDirection(true); set_speed(speed);
	 * 
	 * while(lmenc.getDistance() <= move_angle) { if(timer.get() - prev_t >= delta_t
	 * && lmenc.getDistance() < move_angle) { prev_t = timer.get(); double d_enc =
	 * lmenc.getDistance() - prev_enc; prev_enc = lmenc.getDistance();
	 * 
	 * //use PID to correct the speed of motor by comparing the expected distance
	 * rotated to the actual distance rotated set_speed ( Compute_PID(speed*delta_t,
	 * d_enc, delta_t)/delta_t ); } }
	 * 
	 * //stopMotor(); while(!Break_PID) { if(timer.get() - prev_t >= delta_t &&
	 * lmenc.getDistance() < move_angle) { prev_t = timer.get(); double d_enc =
	 * lmenc.getDistance() - prev_enc; prev_enc = lmenc.getDistance(); //now it is
	 * supposed to stop so we expect an encoder reading of 0 set_speed (
	 * Compute_PID(0, d_enc, delta_t) / delta_t ); } } }
	 * 
	 * /* public void blind_lift(double speed, boolean reverse, double delta_t) {
	 * double prev_t = 0, prev_enc = 0;
	 * 
	 * lmenc.reset(); lmenc.setDistancePerPulse(ENC_ANG_PER_PULSE);
	 * 
	 * if(reverse) lmenc.setReverseDirection(true); set_speed(speed);
	 * 
	 * while(!Break_PID) { if(timer.get() - prev_t >= delta_t) { prev_t =
	 * timer.get(); double d_enc = lmenc.getDistance() - prev_enc; prev_enc =
	 * lmenc.getDistance();
	 * 
	 * set_speed ( Compute_PID(speed, d_enc/delta_t, delta_t) ); } } }
	 */

}
