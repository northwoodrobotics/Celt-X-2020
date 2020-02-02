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
    public static final int LEFT_DRIVE_MOTOR_ONE =1; //SparkMax, NEO
    public static final int LEFT_DRIVE_MOTOR_TWO =2; //SparkMax, NEO
    public static final int RIGHT_DRIVE_MOTOR_ONE =3; //SparkMax, NEO
    public static final int RIGHT_DRIVE_MOTOR_TWO =4; //SparkMax, NEO
    public static final int INTAKE_ROLLER_MOTOR =5; //SparkMax, NEO550
    public static final int HOPPER_BRUSH_MOTOR =6; //SparkMax, NEO550
    public static final int THROAT_SERIALIZER_MOTOR =7; //TalonSRX, 2xBAG
    public static final int FEEDER_MOTOR =8; //TalonSRX, 2xBAG
    public static final int TURRET_AZIMUTH_MOTOR =9; //SparkMax, NEO550
    public static final int HOOD_MOTOR =10; //SparkMax, NEO550
    public static final int BOOSTER_ROLLER_MOTOR =11; //SparkMax, NEO
    public static final int SHOOTER_WHEEL_MOTOR_ONE =12; //SparkMax, NEO
    public static final int SHOOTER_WHEEL_MOTOR_TWO =13; //SparkMax, NEO
    public static final int LEFT_CLIMBER_MOTOR =14; //SparkMax, NEO
    public static final int RIGHT_CLIMBER_MOTOR =15; //SparkMax, NEO

    //PCM PORTS
    public static final int INTAKE_CYLINDER =1;
    public static final int POP_UP_CYLINDER =2;

    //Gear Ratios
    public static final double DRIVE_GEAR_RATIO =11;
    public static final double INTAKE_GEAR_RATIO =4;
    public static final double HOPPER_GEAR_RATIO =12;
    public static final double TURRET_GEAR_RATIO = 20.0*(210.0/22.0); //20:1 gearbox + 22 tooth pinion driving a 210 tooth ring
    public static final double HOOD_GEAR_RATIO =1.0; //fix - approximately 1 degree/revolution
    public static final double BOOSTER_GEAR_RATIO =1.0/2.0; //overdriven
    public static final double SHOOTER_GEAR_RATIO =3.0/4.0; //overdriven

    //Wheel Diameters
    public static final double DRIVE_WHEEL_DIAMETER =6.0; //inches

}
