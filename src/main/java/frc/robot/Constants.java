// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.subsystems.swerveModule.SwerveModuleIO;
import frc.robot.subsystems.swerveModule.SwerveModuleIOSparkFlex;

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
public static final double SWERVE_STEER_KP = 2 / 360.0;
    public static final double SWERVE_STEER_KI = 0.000;
    public static final double SWERVE_STEER_KD = 0;

    public static final double MAX_STEER_VELOCITY = 36000;
    public static final double MAX_STEER_ACCELERATION = 100000;

    public static final double DRIVE_SPEED_MULTIPLIER = 1.0;

    public static final double RADIUS_IN_METERS = edu.wpi.first.math.util.Units.inchesToMeters(1.85);
    public static final double SWERVE_DRIVE_RATIO = 1 / 6.75;
    public static final double SWERVE_POSITION_FACTOR = RADIUS_IN_METERS * 2 * Math.PI * SWERVE_DRIVE_RATIO;
    public static final double NEO_MAX_RPM = 5676;
    public static final double MAX_SPEED_MPS = (NEO_MAX_RPM / 60.0d) * SWERVE_POSITION_FACTOR;
    public static final double MAX_ROTATION_SPEED_RDPS = 2 * Math.PI * 2;
    public static final double LEVER_ARM_VAL = edu.wpi.first.math.util.Units
            .inchesToMeters(Constants.DISTANCE_BETWEEN_WHEELS) / 2;

    public static final Distance ELEVATOR_TOLERANCE = Units.Inches.of(3);

    public static final double DISTANCE_BETWEEN_WHEELS = 22.75; // inches
    // INTAKE CONSTANTS
    public static final Time PISTON_THRESHOLD = Units.Second.of(1.0);
    public static final Time TARGET_DROP_THRESHOLD = Units.Second.of(0.5);
    public static final Time PISTON_OUT_TIME = Units.Seconds.of(0.25);
    public static final Pose3d[] EMPTY_POSE_ARRAY = {};
    public static final int[] EMPTY_INT_ARRAY = {};

    public static final int AUTON_STAGES = 7;

    public record ModuleConstants(
            String label,
            int driveId,
            boolean driveInverted,
            int steerId,
            boolean steerInverted,
            int encoderId,
            double encoderOffset,
            SensorDirectionValue encoderDirection,
            int dashBoardX,
            int dashBoardY,
            Translation2d leverArm) {
    }

    // positive x is foward and positive y is left
    public static final ModuleConstants[] MODULE_CONSTANTS = new ModuleConstants[] {
            new ModuleConstants(
                    "FL",
                    9,
                    false,
                    7,
                    true,
                    8,
                    -0.360840,
                    SensorDirectionValue.CounterClockwise_Positive,
                    3,
                    0,
                    new Translation2d(Constants.LEVER_ARM_VAL, Constants.LEVER_ARM_VAL)),
            new ModuleConstants(
                    "FR",
                    12,
                    false,
                    10,
                    true,
                    11,
                    -0.555176,
                    SensorDirectionValue.CounterClockwise_Positive,
                    0,
                    0,
                    new Translation2d(Constants.LEVER_ARM_VAL, -Constants.LEVER_ARM_VAL)),
            new ModuleConstants(
                    "BL",
                    6,
                    false,
                    4,
                    true,
                    5,
                    0.613037,
                    SensorDirectionValue.CounterClockwise_Positive,
                    0,
                    0,
                    new Translation2d(-Constants.LEVER_ARM_VAL, Constants.LEVER_ARM_VAL)),
            new ModuleConstants(
                    "BR",
                    3,
                    false,
                    1,
                    true,
                    2,
                    -0.481445,
                    SensorDirectionValue.CounterClockwise_Positive,
                    3,
                    5,
                    new Translation2d(-Constants.LEVER_ARM_VAL, -Constants.LEVER_ARM_VAL))
    };

    // positive x is foward and positive y is left
    public static final ModuleConstants[] MODULE_CONSTANTS_SYMPHONY = new ModuleConstants[] {
            new ModuleConstants(
                    "FL",
                    3,
                    false,
                    1,
                    true,
                    2,
                    -0.0185555,
                    SensorDirectionValue.CounterClockwise_Positive,
                    3,
                    0,
                    new Translation2d(Constants.LEVER_ARM_VAL, Constants.LEVER_ARM_VAL)),
            new ModuleConstants(
                    "FR",
                    4,
                    true,
                    6,
                    true,
                    5,
                    -0.060059,
                    SensorDirectionValue.CounterClockwise_Positive,
                    0,
                    0,
                    new Translation2d(Constants.LEVER_ARM_VAL, -Constants.LEVER_ARM_VAL)),
            new ModuleConstants(
                    "BL",
                    10,
                    true,
                    12,
                    true,
                    11,
                    -0.0434570,
                    SensorDirectionValue.CounterClockwise_Positive,
                    0,
                    0,
                    new Translation2d(-Constants.LEVER_ARM_VAL, Constants.LEVER_ARM_VAL)),
            new ModuleConstants(
                    "BR",
                    9,
                    true,
                    7,
                    true,
                    8,
                    -0.019287109375,
                    SensorDirectionValue.CounterClockwise_Positive,
                    3,
                    5,
                    new Translation2d(-Constants.LEVER_ARM_VAL, -Constants.LEVER_ARM_VAL))
    };

    public static SwerveModuleIO getRealSwerveModuleIO(ModuleConstants moduleConstants) {
       return new SwerveModuleIOSparkFlex(moduleConstants);
    }

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
