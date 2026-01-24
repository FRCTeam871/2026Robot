// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final boolean shouldReplay = true;

//-------------------------------------------------------old

    public static final double DRIVE_SPEED = .5;
    public static final double DRIVE_ROTATION = 1;
    public static final double ARM_SPEED = 1.0;
    public static final double SHOOT_SPEED = .5;

    public static final double SWERVE_STEER_KP = 2 / 360.0;
    public static final double SWERVE_STEER_KI = 0.000;
    public static final double SWERVE_STEER_KD = 0;

    public static final double MAX_VELOCITY = 36000;
    public static final double MAX_ACCELERATION = 100000;

    public static final double RADIUS_IN_METERS = Units.inchesToMeters(2);
    public static final double SWERVE_DRIVE_RATIO = 1 / 6.75;
    public static final double SWERVE_POSITION_FACTOR = RADIUS_IN_METERS * 2 * Math.PI * SWERVE_DRIVE_RATIO;
    public static final double NEO_MAX_RPM = 5676;
    public static final double MAX_SPEED_MPS = (NEO_MAX_RPM/ 60.0d) * SWERVE_POSITION_FACTOR;

    public static final int BACK_LEFT_DRIVE_ID = 4;
    public static final int BACK_RIGHT_DRIVE_ID = 3;
    public static final int FRONT_RIGHT_DRIVE_ID = 10;
    public static final int FRONT_LEFT_DRIVE_ID = 9;

    public static final int BACK_LEFT_STEER_ID = 6;
    public static final int BACK_RIGHT_STEER_ID = 1;
    public static final int FRONT_RIGHT_STEER_ID = 12;
    public static final int FRONT_LEFT_STEER_ID = 7;

    public static final int BACK_LEFT_SENSOR_ID = 5;
    public static final int BACK_RIGHT_SENSOR_ID = 2;
    public static final int FRONT_RIGHT_SENSOR_ID = 11;
    public static final int FRONT_LEFT_SENSOR_ID = 8;
    
    public static final boolean DRIVE_INVERTED_BACK_LEFT = true;
    public static final boolean DRIVE_INVERTED_BACK_RIGHT = false;
    public static final boolean DRIVE_INVERTED_FRONT_RIGHT = true; 
    public static final boolean DRIVE_INVERTED_FRONT_LEFT = true; 

    public static final double MAGNET_OFFSET_BACK_LEFT =-0.060059;
    public static final double MAGNET_OFFSET_BACK_RIGHT =-0.018555;
    public static final double MAGNET_OFFSET_FRONT_RIGHT = -0.043457 ;
    public static final double MAGNET_OFFSET_FRONT_LEFT =-0.019287109375;

    public static final boolean STEER_INVERTED_BACK_LEFT = true;
    public static final boolean STEER_INVERTED_BACK_RIGHT = true;
    public static final boolean STEER_INVERTED_FRONT_RIGHT = true;
    public static final boolean STEER_INVERTED_FRONT_LEFT = true;

    public static final SensorDirectionValue STEER_ENCODER_DIRECTION_BACK_LEFT = SensorDirectionValue.CounterClockwise_Positive;
    public static final SensorDirectionValue STEER_ENCODER_DIRECTION_BACK_RIGHT = SensorDirectionValue.CounterClockwise_Positive;
    public static final SensorDirectionValue STEER_ENCODER_DIRECTION_FRONT_RIGHT = SensorDirectionValue.CounterClockwise_Positive;
    public static final SensorDirectionValue STEER_ENCODER_DIRECTION_FRONT_LEFT = SensorDirectionValue.CounterClockwise_Positive;
    
    public static final double LEVER_ARM_VAL = (Constants.DISTANCE_BETWEEN_WHEELS / 39.37) / 2;
    
    public static final int AUTON_STAGES = 7;
    /**
     * Specifically arm
     * 
     * 
     */
    public static final double NINETY_DEGREE_VOLTAGE_VALUE = 1.18;
    public static final double ZERO_DEGREE_VOLTAGE_VALUE = 2.00;

    public static final double ARM_PROPORTIONAL = .0175;
    public static final double ARM_INTEGRAL = 0;
    public static final double ARM_DERIVATIVE = 0;

    public static final double ARM_MAX_ACCELERATION = 720;
    public static final double ARM_MAX_VELOCITY = 360;

    /**
     * Arm Setpoints
     * 
     * 
     */
    public static final double ARM_SETPOINT_HORIZONTAL = 12 + 8;
    public static final double ARM_SETPOINT_AMP = 104;
    public static final double ARM_SETPOINT_SPEAKER = 65.5;
    public static final double ARM_SETPOINT_PICKUP = -7;
    public static final double ARM_SETPOINT_AUTONSTART = 8;

    /**
     * 
     * In Inches
     */
    public static final double DISTANCE_BETWEEN_WHEELS = 22.75;

    public static double deadband(double raw, double threshold) {
        
        if (Math.abs(raw) < threshold) {
            return 0;
        } else {
            if (raw > 0) {
                return (raw - threshold) / (1 - threshold);
            } else {
                return (raw + threshold) / (1 - threshold);
            }
        }

    }

    public static double deadbandAndExponential(double raw) {
        return exponentialDrive(deadband(raw, 0.1));
    }

    public static double exponentialDrive(double controllerOutput) {
        double contollerOutputA = 15;
        double controllerOutputB = 0.015;
        double controllerOutputC = (1 - controllerOutputB) / (contollerOutputA - 1);
        double wrappedControllerOutput = controllerOutputC * Math.pow(contollerOutputA, Math.abs(controllerOutput))
                + controllerOutputB * Math.abs(controllerOutput)
                - controllerOutputC;
        if (controllerOutput >= 0) {
            return wrappedControllerOutput;
        } else {
            return -wrappedControllerOutput;
        }
    }
   

}