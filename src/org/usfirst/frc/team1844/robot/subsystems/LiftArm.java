package org.usfirst.frc.team1844.robot.subsystems;

import org.usfirst.frc.team1844.robot.RobotConstants;
import org.usfirst.frc.team1844.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;


/**
 *
 */
public class LiftArm extends Subsystem {

	
	private SpeedController LM1;
	private SpeedController LM2;
	private SpeedControllerGroup LMGroup;
	
	private DigitalInput switch_pos;
	private DigitalInput scale_pos;
	private boolean at_switch, at_scale;
	private double time_to_switch, time_to_scale, time_scale_to_switch;
	private Timer timer;
	private boolean start_timer;

	//if we are using encoders:
	//protected Encoder lmenc;	
	//private PIDController LMPID;
	
	private boolean one_motor;
	
	/*
	private static final double ENC_ANG_PER_PULSE = Math.PI/RobotConstants.ENCODER_PPR * RobotConstants.GEARBOX_REDUCTION;
	private static final double Kp = 0.1, Ki = 0, Kd = 0, bias = 0.01;
	private double prev_err, err, d_err, i_err;
	private Timer timer;
	private boolean Break_PID;
	*/
	
    public void initDefaultCommand() {
       one_motor = false;
       at_switch = false; at_scale = false; start_timer = false;
       time_to_switch = 0; time_to_scale = 0; time_scale_to_switch = 0;
     //  Break_PID = false;
       
       LM1 = new WPI_TalonSRX (RobotMap.Lift_Motor_one);
       LM2 = new WPI_TalonSRX (RobotMap.Lift_Motor_two);
       
      // lmenc = new Encoder (RobotMap.Lift_Motor_one, RobotMap.Lift_Motor_two, false, EncodingType.k4X);
     
       LMGroup = new SpeedControllerGroup (LM1, LM2);
       switch_pos = new DigitalInput(RobotMap.DIO_ARM_SWITCH);
       scale_pos = new DigitalInput(RobotMap.DIO_ARM_SCALE);
       
       
       timer = new Timer();
      // LMPID = new PIDController (Kp, Ki, Kd, lmenc, LMGroup);
       //LMPID.setAbsoluteTolerance(1);
       //LMPID.setOutputRange(-1.0, 1.0);
    }
    
    //call this if we are only using one motor in the design
    public void set_one_motor()
    {
    	one_motor = true;
    	at_switch = false; at_scale = false; start_timer = false;
        time_to_switch = 0; time_to_scale = 0; time_scale_to_switch = 0;
        
    	LM1 = new WPI_TalonSRX (RobotMap.Lift_Motor_one);
    	//Break_PID = false;
    	
    	 switch_pos = new DigitalInput(RobotMap.DIO_ARM_SWITCH);
         scale_pos = new DigitalInput(RobotMap.DIO_ARM_SCALE);
    	//lmenc = new Encoder (RobotMap.Lift_Motor_one, RobotMap.Lift_Motor_one, false, EncodingType.k4X);
    	
        timer = new Timer();
        //LMPID = new PIDController (Kp, Ki, Kd, lmenc, LM1);
        //LMPID.setAbsoluteTolerance(1);
        //LMPID.setOutputRange(-1.0, 1.0);
    }
    
    /*
    public void set_break_PID(boolean BREAK)
    {
    	Break_PID = BREAK;
    }
    */
    
    public void set_speed (double speed)
    {
    	if(-1 <= speed && speed < 1)
    	{
	    	if(one_motor)
	    		LM1.set(speed);
	    	else
	    		LMGroup.set(speed);
    	}
    }
    
    public double get_speed(int motor_index)
    {
    	if(motor_index == 1)
    		return LM1.get();
    	else if (motor_index == 2)
    		return LM2.get();
    	return 0;
    }
    
    
    public void set_invert(boolean invert)
    {
    	LM1.setInverted(invert);
    	if(!one_motor)
    		LM2.setInverted(invert);
    }
    
    public void stopMotor()
    {
    	if(one_motor)
    		LM1.stopMotor();
    	else
    		LMGroup.stopMotor();
    }
    
    
    public boolean lift_to_pos (int pos_index, double speed)
    {
    	if(pos_index == RobotMap.DIO_ARM_SWITCH)
    	{
    		//from origin up to switch
    		if(!at_scale && !at_switch)
    			set_speed(speed);
    		//from scale lower to switch
    		else if (!at_switch && at_scale)
    			 set_speed(-speed);
    		else
    			return true;
    		
    		//since we do not want to reset timer every time in the execute in command
    		if(!start_timer)
    		{
    			timer.reset();
    			start_timer = true;
    		}
    		
    		//assume limit switch setup as false from default
    		if(switch_pos.get())
    		{
	    		stopMotor();
	    		
	    		if(!at_scale && !at_switch)
	    			time_to_switch = timer.get();
	    		else if(!at_switch && at_scale)
	    			time_scale_to_switch = timer.get();
	    		
	    		at_switch = true;
	    		at_scale = false;
	    		start_timer = false;
    		}
    	}
    	
    	else if(pos_index == RobotMap.DIO_ARM_SCALE)
    	{
    		if (at_scale && !at_switch)
        		return true;
    		
    		//from origin up to scale && from switch up to scale, both are up!
    		set_speed(speed);
    		
    		if(!start_timer)
    		{
    			timer.reset();
    			start_timer = true;
    		}
    		
    		//assume limit switch setup as false from default
    		if(switch_pos.get())
    		{
	    		stopMotor();
	    		
	    		if(!at_scale && !at_switch)
	    			time_to_scale = timer.get();
	    		else if(at_switch && !at_scale)
	    			time_scale_to_switch = timer.get();
	    			
	    		at_switch = false;
	    		at_scale = true;
	    		start_timer = false;
    		}
    	}
    	
    	else if(pos_index == RobotMap.DIO_ARM_ORIGIN)
    	{
    		if(!start_timer)
    		{
    			timer.reset();
    			start_timer = true;
    		}
    		
    		set_speed(-speed);
    		
    		if(at_scale)
    		{
    			if(timer.get() >= time_to_scale)
    			{
    				stopMotor();
    				at_scale = false;
    				start_timer = false;
    			}
    		}
    		
    		else if(at_switch)
    		{
    			if(timer.get() >= time_to_switch)
    			{
    				stopMotor();
    				at_scale = false;
    				start_timer = false;
    			}
    		}
    	}
    	
    	return true;
    }
    
    
    /*										
    private double Compute_PID (double desired_outut, double actual_output, double delta_t)
    {
    	 err = desired_outut - actual_output;
    	 i_err = i_err + err * delta_t;
    	 d_err = (prev_err - err) / delta_t;
    	 prev_err = err;

    	return Kp * err + Ki * i_err + Kd * d_err + bias;
    }
    */
    
    /*
    public void lift(double speed, double move_angle, boolean reverse, double delta_t)
    {
    	double prev_t = 0, prev_enc = 0;
    	
    	lmenc.reset();
    	lmenc.setDistancePerPulse(ENC_ANG_PER_PULSE);
    	
    	if(reverse)
	    	lmenc.setReverseDirection(true);
    	set_speed(speed);
    	
    	while(lmenc.getDistance() <= move_angle) 
    	{
    		if(timer.get() - prev_t >= delta_t && lmenc.getDistance() < move_angle)
    		{
    			prev_t = timer.get();
    			double d_enc = lmenc.getDistance() - prev_enc;
    			prev_enc = lmenc.getDistance();
    			
    			//use PID to correct the speed of motor by comparing the expected distance rotated to the actual distance rotated
    			set_speed ( Compute_PID(speed*delta_t, d_enc, delta_t)/delta_t );
    		}
    	}
    	
    	//stopMotor();
    	while(!Break_PID)
    	{
    		if(timer.get() - prev_t >= delta_t && lmenc.getDistance() < move_angle)
    		{
    			prev_t = timer.get();
    			double d_enc = lmenc.getDistance() - prev_enc;
    			prev_enc = lmenc.getDistance();
    					 //now it is supposed to stop so we expect an encoder reading of 0
    			set_speed ( Compute_PID(0, d_enc, delta_t) / delta_t );
    		}
    	}
    }
    
    /*
    public void blind_lift(double speed, boolean reverse, double delta_t)
    {
    	double prev_t = 0, prev_enc = 0;
    	
    	lmenc.reset();
    	lmenc.setDistancePerPulse(ENC_ANG_PER_PULSE);
    	
    	if(reverse)
	    	lmenc.setReverseDirection(true);
    	set_speed(speed);
    	
    	while(!Break_PID)
    	{
    		if(timer.get() - prev_t >= delta_t)
    		{
    			prev_t = timer.get();
    			double d_enc = lmenc.getDistance() - prev_enc;
    			prev_enc = lmenc.getDistance();
    			
    			set_speed ( Compute_PID(speed, d_enc/delta_t, delta_t) );
    		}
    	}
    }
    */
 
    
}

