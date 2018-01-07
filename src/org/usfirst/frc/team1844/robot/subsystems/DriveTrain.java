package org.usfirst.frc.team1844.robot.subsystems;

import org.usfirst.frc.team1844.robot.RobotMap;

import com.ctre.CANTalon;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
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
	DifferentialDrive drive;
	//private static final 
	
	Encoder Lenc;
	Encoder Renc;

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
		//TODO add this later^
    	fl = new WPI_TalonSRX(RobotMap.LeftDrive1);
    	bl = new WPI_TalonSRX(RobotMap.LeftDrive2);
    	fr = new WPI_TalonSRX(RobotMap.RightDrive1);
    	br = new WPI_TalonSRX (RobotMap.RightDrive2);
    	
    	//Encoders
    	Lenc = new Encoder (RobotMap.LeftEncoder1, RobotMap.LeftEncoder2,
			/*change this if needed*/ true, EncodingType.k4X);
    	Renc = new Encoder (RobotMap.RightEncoder1, RobotMap.RightEncoder2,
    			false, EncodingType.k4X);
    	SCGLeft = new SpeedControllerGroup(fl, bl);
    	SCGRight = new SpeedControllerGroup(fr, br);
    	SCGLeft.setInverted(true);
    	drive = new DifferentialDrive(SCGLeft, SCGRight);    	
    }
    public void spin()
    {
    	drive.tankDrive(-1, 1);
    }
    public void stop()
    {
    	drive.stopMotor();
    }
}

