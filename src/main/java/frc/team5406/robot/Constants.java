/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team5406.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //CAN IDs
    public static final int LEFT_DRIVE_MOTOR_ONE = 1; //SparkMax, NEO
    public static final int LEFT_DRIVE_MOTOR_TWO = 2; //SparkMax, NEO
    public static final int RIGHT_DRIVE_MOTOR_ONE = 3; //SparkMax, NEO
    public static final int RIGHT_DRIVE_MOTOR_TWO = 4; //SparkMax, NEO
    public static final int INTAKE_ROLLER_MOTOR = 5; //SparkMax, NEO550
    public static final int HOPPER_BRUSH_MOTOR = 6; //SparkMax, NEO550
    public static final int THROAT_SERIALIZER_MOTOR_ONE = 7; //TalonSRX, BAG
    public static final int TURRET_AZIMUTH_MOTOR = 9; //SparkMax, NEO550
    public static final int HOOD_MOTOR = 10; //SparkMax, NEO550
    public static final int BOOSTER_ROLLER_MOTOR = 11; //SparkMax, NEO
    public static final int SHOOTER_WHEEL_MOTOR_ONE = 12; //SparkMax, NEO
    public static final int SHOOTER_WHEEL_MOTOR_TWO = 13; //SparkMax, NEO
    public static final int LEFT_CLIMBER_MOTOR = 14; //SparkMax, NEO
    public static final int RIGHT_CLIMBER_MOTOR = 15; //SparkMax, NEO
    public static final int THROAT_SERIALIZER_MOTOR_TWO = 21; //TalonSRX, BAG
    public static final int UPPER_FEEDER_MOTOR = 22; //SparkMax, NEO
    

    //CANcoder PORTS
    public static final int HOOD_ENCODER = 30;
    public static final int TURRET_ENCODER = 31;

    //PCM PORTS
    public static final int INTAKE_CYLINDER = 0;
    public static final int CLIMB_CYLINDER = 1;

    //XBoxController Ports
    public static final int OPERATOR_CONTROLLER = 0;
    public static final int DRIVER_CONTROLLER = 1;

    //Gear Ratios
    public static final double DRIVE_GEAR_RATIO = 11;
    public static final double INTAKE_GEAR_RATIO = 4;
    public static final double HOPPER_GEAR_RATIO = 12;
    public static final double TURRET_GEAR_RATIO = 20.0*(210.0/22.0); //20:1 gearbox + 22 tooth pinion driving a 210 tooth ring
    public static final double HOOD_GEAR_RATIO = (23.1*10)/360;//(19.25*10)/360; //Doesn't match CAD?
    public static final double BOOSTER_GEAR_RATIO = 1.0/2.0; //overdriven
    public static final double SHOOTER_GEAR_RATIO = 3.0/4.0; //overdriven
    public static final double HOOD_ENC_GEAR_RATIO = 3.0/.9;
    public static final double FEEDER_GEAR_RATIO = 5.0;

    //Wheel Diameters
    public static final double DRIVE_WHEEL_DIAMETER = 6.0; //inches
    public static final double COLOUR_WHEEL_DIAMETER = 2.25; //inches
    
    //Other Constants
    public final static int CAN_TIMEOUT = 15; //ms
    public static final double OUTPUT_RANGE_MIN = -1;
    public static final double OUTPUT_RANGE_MAX = 1;

    //Fixed speeds
    public static final double INTAKE_ROLLER_OUTPUT = 1;
    public static final double HOPPER_BRUSH_OUTPUT = 0.8;
    public static final double SERIALIZER_OUTPUT = 0.4;
    public static final double UPPER_FEEDER_OUTPUT = 1;
    
    public static final boolean INTAKE_EXTEND = true;
    public static final boolean INTAKE_RETRACT = false;
    public static final boolean CLIMB_EXTEND = true;
    public static final boolean CLIMB_RETRACT = false;

    //Closed-loop Ramp Rates
    public static final double SHOOTER_CLOSED_LOOP_RAMP_RATE = 0.01; //seconds
    public static final double DRIVE_CLOSED_LOOP_RAMP_RATE = 0.05; //seconds

    //PIDs
    public static final double SHOOTER_PID0_P = 9e-4;
    public static final double SHOOTER_PID0_I = 0;
    public static final double SHOOTER_PID0_D = 0;
    public static final double SHOOTER_PID0_F = 1.9e-4;
    
    public static final double FEEDER_PID0_P = 3e-5;
    public static final double FEEDER_PID0_I = 0;
    public static final double FEEDER_PID0_D = 0;
    public static final double FEEDER_PID0_F = 1.1e-4;

    public static final double BOOSTER_PID0_P = 3e-5;
    public static final double BOOSTER_PID0_I = 0;
    public static final double BOOSTER_PID0_D = 0;
    public static final double BOOSTER_PID0_F = 1.9e-4;

    public static final double HOOD_PID0_P = 9e-1;
    public static final double HOOD_PID0_I = 0;
    public static final double HOOD_PID0_D = 0;
    public static final double HOOD_PID0_F = 0.04;

    public static final double LEFT_CLIMBER_PID0_P = 3e-4;
    public static final double LEFT_CLIMBER_PID0_I = 0;
    public static final double LEFT_CLIMBER_PID0_D = 0;
    public static final double LEFT_CLIMBER_PID0_F = 1.9e-3;

    public static final double RIGHT_CLIMBER_PID0_P = 3e-4;
    public static final double RIGHT_CLIMBER_PID0_I = 0;
    public static final double RIGHT_CLIMBER_PID0_D = 0;
    public static final double RIGHT_CLIMBER_PID0_F = 1.9e-3;


    //Current Limits
    public static final int NEO550_CURRENT_LIMIT = 40;
    public static final int NEO_CURRENT_LIMIT = 60;
    public static final int BAG_CURRENT_LIMIT = 15;
    public static final int PEAK_CURRENT_DURATION = 50;
    public static final int SHOOTER_CURRENT_LIMIT = 50;
    public static final int BOOSTER_CURRENT_LIMIT = 50;
    public static final int HOOD_CURRENT_LIMIT = 30;

}