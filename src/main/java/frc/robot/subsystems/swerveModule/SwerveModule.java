package frc.robot.subsystems.swerveModule;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
    private final ProfiledPIDController pidController;
    private final Translation2d leverArm;
    private SwerveModuleState state = new SwerveModuleState(0, new Rotation2d(0));
    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
    private final String label;

    public SwerveModule(final Translation2d leverArm, final SwerveModuleIO io, final String label) {
        this.leverArm = leverArm;
        this.io = io;
        this.label = label;

        this.pidController = new ProfiledPIDController(
                Constants.SWERVE_STEER_KP,
                Constants.SWERVE_STEER_KI,
                Constants.SWERVE_STEER_KD,
                new TrapezoidProfile.Constraints(Constants.MAX_STEER_VELOCITY, Constants.MAX_STEER_ACCELERATION));
        this.pidController.enableContinuousInput(0, 360);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + label, inputs);

        final double currentSteeringAngle = getDirectionDegrees();
        state.optimize(Rotation2d.fromDegrees(currentSteeringAngle));

        final double desiredSteeringAngle = state.angle.getDegrees();
        final double outputSteer = pidController.calculate(currentSteeringAngle, desiredSteeringAngle);

        io.setDriveSpeed(state.speedMetersPerSecond / Constants.MAX_SPEED_MPS);
        // Logger.recordOutput(
        //         "Drive/Module" + label + "/DriveSpeed", state.speedMetersPerSecond / Constants.MAX_SPEED_MPS);
        io.setSteerSpeed(outputSteer);
        // Logger.recordOutput("Steer/Module" + label + "/SteerSpeed", outputSteer);
    }

    public void setState(final SwerveModuleState state) {
        this.state = state;
    }

    // @AutoLogOutput(key = "Drive/Module{label}/State")
    public SwerveModuleState getState() {
        return new SwerveModuleState(inputs.driveVelocity, Rotation2d.fromDegrees(getDirectionDegrees()));
    }

    // @AutoLogOutput(key = "Drive/Module{label}/Position")
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(inputs.drivePosition, Rotation2d.fromDegrees(getDirectionDegrees()));
    }

    private double getDirectionDegrees() {
        return inputs.steeringAngleDegrees;
    }

    public Translation2d getLeverArm() {
        return leverArm;
    }
}
