// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.utilities.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // // DRIVEBASE \\ \\

    // Make sure to measure these with as much presicion as possible, as it will have great affect on path planner autos and teleop driving

    public static final double TRACK_WIDTH = Units.inchesToMeters(20.67); // Distance between centers of right and left wheels on robot

    public static final double WHEEL_BASE = Units.inchesToMeters(20.75); // Distance between front and back wheels on robot

    public static final double MAX_METERS_PER_SECOND = Units.feetToMeters(12.0); // Find this on sds website

    public static final double DRIVE_GEAR_RATIO = 8.14; // MK4i L1 Neo, find on sds website

    public static final double TURN_GEAR_RATIO = 150.0 / 7.0; // MK4i turning ratio MK4i Neo, find on sds website

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.79); // Wheel diameter


    public static final double REGULAR_SPEED = 1; // Regular speed multiplier of robot

    public static final double SLOW_SPEED = 0.4; // Slow speed multiplier of robot


  
    // FRONT LEFT MODULE \\
  
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 12; 
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 11; 
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 1; 
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 52.03; // 52.03: Grimes House
    public static final SwerveModuleConstants FRONT_LEFT_MODULE = new SwerveModuleConstants(FRONT_LEFT_MODULE_DRIVE_MOTOR, FRONT_LEFT_MODULE_STEER_MOTOR, FRONT_LEFT_MODULE_STEER_ENCODER, FRONT_LEFT_MODULE_STEER_OFFSET);

  
    // FRONT RIGHT MODULE \\
  
    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 18;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 17; 
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 7;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 144.22; // 144.22: Grimes House
    public static final SwerveModuleConstants FRONT_RIGHT_MODULE =  new SwerveModuleConstants(FRONT_RIGHT_MODULE_DRIVE_MOTOR, FRONT_RIGHT_MODULE_STEER_MOTOR, FRONT_RIGHT_MODULE_STEER_ENCODER, FRONT_RIGHT_MODULE_STEER_OFFSET);

  
    // BACK LEFT MODULE \\
  
    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 14; 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 13;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 3;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = 45.25; // 45.25: Grimes House
    public static final SwerveModuleConstants BACK_LEFT_MODULE = new SwerveModuleConstants(BACK_LEFT_MODULE_DRIVE_MOTOR, BACK_LEFT_MODULE_STEER_MOTOR, BACK_LEFT_MODULE_STEER_ENCODER, BACK_LEFT_MODULE_STEER_OFFSET);

  
    // BACK RIGHT MODULE \\
    
    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 16; 
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 15; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 5; 
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 231.1; // 231.1: Grimes House
    public static final SwerveModuleConstants BACK_RIGHT_MODULE = new SwerveModuleConstants(BACK_RIGHT_MODULE_DRIVE_MOTOR, BACK_RIGHT_MODULE_STEER_MOTOR, BACK_RIGHT_MODULE_STEER_ENCODER, BACK_RIGHT_MODULE_STEER_OFFSET);
    
}
