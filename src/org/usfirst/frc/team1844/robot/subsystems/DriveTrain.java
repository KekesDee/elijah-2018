package org.usfirst.frc.team1844.robot.subsystems;

import org.usfirst.frc.team1844.robot.RobotMap;

import com.ctre.CANTalon;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 *
 */
public class DriveTrain extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	SpeedController fl; 
	SpeedController bl;
	SpeedController fr;
	SpeedController br;
	SpeedControllerGroup SCGLeft;
	SpeedControllerGroup SCGRight;
	//private static final 
	PIDController driveleftPID;
	PIDController driverightPID;
	Encoder Lenc;
	Encoder Renc;

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
		//TODO add this later^
    	fl = new WPI_TalonSRX(RobotMap.CAN_DRIVE_FLMOTOR);
    	bl = new WPI_TalonSRX(RobotMap.CAN_DRIVE_BLMOTOR);
    	fr = new WPI_TalonSRX(RobotMap.CAN_DRIVE_FRMOTOR);
    	br = new WPI_TalonSRX (RobotMap.CAN_DRIVE_BRMOTOR);
    	
    	//Encoders
    	Lenc = new Encoder (RobotMap.DIO_DRIVE_LEFTENC_A, RobotMap.DIO_DRIVE_LEFTENC_B,
			/*change this if needed*/ true, EncodingType.k4X);
    	Renc = new Encoder (RobotMap.DIO_DRIVE_RIGHTENC_A, RobotMap.DIO_DRIVE_RIGHTENC_B,
    			false, EncodingType.k4X);
    	Lenc.setPIDSourceType(PIDSourceType.kDisplacement);
    	Renc.setPIDSourceType(PIDSourceType.kDisplacement);
    	
    	SCGLeft = new SpeedControllerGroup(fl, bl);
    	SCGRight = new SpeedControllerGroup(fr, br);
    	SCGLeft.setInverted(true);
    	
    	driveleftPID = new PIDController (0.1, 0, 0, Lenc, SCGLeft);
    	driverightPID = new PIDController (0.1, 0, 0, Renc, SCGRight);
    	driveleftPID.setAbsoluteTolerance(1);
    	driverightPID.setAbsoluteTolerance(1);
    	driveleftPID.setOutputRange(-1.0, 1.0);
    	driverightPID.setOutputRange(-1.0, 1.0);
    	
    }
    public void spin()
    {
    	SCGLeft.set(-1);
    	SCGRight.set(1);
    }
    public void stop()
    {
    	SCGLeft.set(0);
    	SCGRight.set(0);
    }
    public void tankdrive(double leftspeed, double rightspeed)
    {
    	SCGLeft.set(leftspeed);
    	SCGRight.set(rightspeed);
    }
    public void setdist(double targetdist)
    {
    	driveleftPID.setSetpoint(targetdist);
    	driverightPID.setSetpoint(targetdist);
    }
    public void PIDDrive()
    {
    	driveleftPID.enable();
    	driverightPID.enable();
    }
    public void PIDStop()
    {
    	driveleftPID.disable();
    	driverightPID.disable();
    }
}

