package frc.robot.subsystem;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.Constants;

public class SwerveModule {
    private final SparkMax drive;
    private final SparkMax steer;
    private final CANcoder steeringEncoder;
    private final RelativeEncoder distanceEncoder;

    private final ProfiledPIDController pidController;
    private final Translation2d leverArm;
    private SwerveModuleState state = new SwerveModuleState(0, new Rotation2d(0));

    private final GenericEntry steerOutputEntry;
    private final GenericEntry driveOutputEntry;
    // private final GenericEntry targetAngleEntry;


    public SwerveModule(final int driveId,
                        final boolean driveInverted,
                        final int steerId,
                        final boolean steerInverted,
                        final int encoderId,
                        final double encoderOffset,
                        final SensorDirectionValue encoderDirection,
                        final Translation2d leverArm,
                        ShuffleboardLayout layout) {
        this.leverArm = leverArm;

        this.drive = new SparkMax(driveId, MotorType.kBrushless);
        SparkBaseConfig driveConfig = new SparkMaxConfig().inverted(driveInverted).idleMode(IdleMode.kBrake);
        this.drive.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 
        driveConfig.encoder.positionConversionFactor(Constants.SWERVE_POSITION_FACTOR);
        driveConfig.encoder.velocityConversionFactor(Constants.SWERVE_POSITION_FACTOR / 60);
        this.distanceEncoder = drive.getEncoder();

        this.steer = new SparkMax(steerId, MotorType.kBrushless);
        this.steer.configure(new SparkMaxConfig().inverted(steerInverted).idleMode(IdleMode.kBrake), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 

        final CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = encoderOffset;
        config.MagnetSensor.SensorDirection = encoderDirection;
        this.steeringEncoder = new CANcoder(encoderId);
        this.steeringEncoder.getConfigurator().apply(config);

        this.pidController = new ProfiledPIDController(Constants.SWERVE_STEER_KP,
                Constants.SWERVE_STEER_KI,
                Constants.SWERVE_STEER_KD,
                new TrapezoidProfile.Constraints(Constants.MAX_VELOCITY, Constants.MAX_ACCELERATION));
        this.pidController.enableContinuousInput(0, 360);

        // Add stuff to the layout
        this.steerOutputEntry = layout.add("Steer Power", 0)
                .withPosition(0, 0).getEntry();
        layout.addDouble("Current Angle", this::getDirectionDegrees)
                .withPosition(0, 2)
                .withWidget(BuiltInWidgets.kGyro);
        this.driveOutputEntry = layout.add("Drive Power", 0)
                .withPosition(0, 1)
                .getEntry();
        // this.targetAngleEntry = layout.add("Target Angle", 0.0d)
        //         .withPosition(1, 1)
        //         .withWidget(BuiltInWidgets.kGyro)
        //         .getEntry();
    }

    public void setState(final SwerveModuleState state) {
        this.state = state;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(distanceEncoder.getVelocity(), Rotation2d.fromDegrees(getDirectionDegrees()));
    }

    public void periodic() {
        final double currentSteeringAngle = getDirectionDegrees();
        final SwerveModuleState optimizedState = SwerveModuleState.optimize(state,
                Rotation2d.fromDegrees(currentSteeringAngle));
        final double desiredSteeringAngle = optimizedState.angle.getDegrees();
        final double outputSteer = pidController.calculate(currentSteeringAngle, desiredSteeringAngle);

        drive.set(optimizedState.speedMetersPerSecond/Constants.MAX_SPEED_MPS );
        steer.set(outputSteer);

        // targetAngleEntry.setDouble(desiredSteeringAngle);
        driveOutputEntry.setDouble(optimizedState.speedMetersPerSecond/Constants.MAX_SPEED_MPS);
        steerOutputEntry.setDouble(outputSteer);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(distanceEncoder.getPosition(),
                Rotation2d.fromDegrees(getDirectionDegrees()));
    }

    private double getDirectionDegrees() {
        return steeringEncoder.getAbsolutePosition().getValue().in(Units.Degrees);
    }

    public Translation2d getLeverArm() {
        return leverArm;
    }
}

