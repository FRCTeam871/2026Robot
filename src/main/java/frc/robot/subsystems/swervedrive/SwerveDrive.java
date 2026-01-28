package frc.robot.subsystems.swervedrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ChangableSlewRateLimiter;
import frc.robot.Constants;
import frc.robot.subsystems.swerveModule.SwerveModule;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends SubsystemBase {
    @AutoLogOutput(key = "Drive/fieldOrientation")
    private boolean fieldOrientation;

    private final SwerveModule[] swerveModules;
    private final SwerveDriveKinematics swerveDriveKinematics;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveDriveIO io;
    private final SwerveDriveIOInputsAutoLogged inputs = new SwerveDriveIOInputsAutoLogged();
    private RobotConfig config;
    private ChangableSlewRateLimiter forwardRateLimiter;
    private ChangableSlewRateLimiter sideRateLimiter;
    private ChangableSlewRateLimiter rotationRateLimiter;

    private boolean headingHoldEnabled;
    private Rotation2d headingHold;
    private ProfiledPIDController yawPidController =
            new ProfiledPIDController(0.08, 0.001, 0, new Constraints(1000, 1000));

    private boolean poseHoldEnabled;
    private Pose2d poseHold;
    private ProfiledPIDController sidePidController =
            new ProfiledPIDController(4, 0.001, .1, new Constraints(1000, 1000));
    private ProfiledPIDController forwardPidController =
            new ProfiledPIDController(2.5, 0.001, .1, new Constraints(1000, 1000));

    public SwerveDrive(final SwerveDriveIO io, final SwerveModule... swerveModules) {
        yawPidController.enableContinuousInput(0, 360);
        this.swerveModules = swerveModules;
        forwardRateLimiter = new ChangableSlewRateLimiter(Constants.MAX_SPEED_MPS);
        sideRateLimiter = new ChangableSlewRateLimiter(Constants.MAX_SPEED_MPS);
        rotationRateLimiter = new ChangableSlewRateLimiter(Constants.MAX_ROTATION_SPEED_RDPS);
        this.io = io;
        final Translation2d[] leverArmArray =
                Arrays.stream(swerveModules).map(SwerveModule::getLeverArm).toArray(Translation2d[]::new);

        this.swerveDriveKinematics = new SwerveDriveKinematics(leverArmArray);
        this.poseEstimator = new SwerveDrivePoseEstimator(
                swerveDriveKinematics, getGyroRotation(), getModulePositions(), new Pose2d());

        try {
            this.config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    poseEstimator::getEstimatedPosition,
                    this::resetOdometry,
                    this::getChassisSpeeds,
                    this::updateSpeed,
                    new PPHolonomicDriveController(new PIDConstants(1, 0, 0), new PIDConstants(1, 0, 0), 4.5),
                    config,
                    () -> false,
                    this);
        } catch (Exception e) {
            DriverStation.reportError("AutoBuilder.configure failed", e.getStackTrace());
        }
    }

    public RobotConfig getConfig() {
        return config;
    }

    public Command manualDrive(final DoubleSupplier vx, final DoubleSupplier vy, final DoubleSupplier omegarad) {
        return run(() -> {
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                    vy.getAsDouble() * Constants.MAX_SPEED_MPS,
                    vx.getAsDouble() * Constants.MAX_SPEED_MPS,
                    omegarad.getAsDouble() * Constants.MAX_ROTATION_SPEED_RDPS);
            if (fieldOrientation) {
                Rotation2d rotation = getEstimatedPose().getRotation();
                if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                    rotation = rotation.plus(Rotation2d.fromDegrees(180));
                }
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, rotation);
            }
            // might not want to make this zero
            if (headingHoldEnabled && chassisSpeeds.omegaRadiansPerSecond == 0) {
                final double yawout = yawPidController.calculate(
                        getEstimatedPose().getRotation().getDegrees(), headingHold.getDegrees());
                chassisSpeeds.omegaRadiansPerSecond = yawout * .5;
            }

            if (poseHoldEnabled && chassisSpeeds.vxMetersPerSecond == 0 && chassisSpeeds.vyMetersPerSecond == 0) {
                // Logger.recordOutput("Drive/HoldPose", poseHold);

                final Pose2d relTgtPose = poseHold.relativeTo(getEstimatedPose());

                final double yout = sidePidController.calculate(0, relTgtPose.getY());
                final double xout = forwardPidController.calculate(0, relTgtPose.getX());
                // Logger.recordOutput("Drive/tyPID", yout);
                // Logger.recordOutput("Drive/txPID", xout);

                chassisSpeeds.vxMetersPerSecond = xout;
                chassisSpeeds.vyMetersPerSecond = yout;
            }

            updateSpeed(chassisSpeeds);
        });
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        // Logger.processInputs("Drive", inputs);

        poseEstimator.update(getGyroRotation(), getModulePositions());
        for (final SwerveModule bob : swerveModules) {
            bob.periodic();
        }
    }

    // @AutoLogOutput(key = "Drive/ChassisSpeeds")
    public ChassisSpeeds getChassisSpeeds() {
        return swerveDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    // @AutoLogOutput(key = "Drive/ModuleStates")
    public SwerveModuleState[] getModuleStates() {
        return Arrays.stream(swerveModules).map(SwerveModule::getState).toArray(SwerveModuleState[]::new);
    }

    // @AutoLogOutput(key = "Drive/ModulePositions")
    public SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(swerveModules).map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new);
    }

    private void setStates(final SwerveModuleState[] states) {
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setState(states[i]);
        }
    }

    /**
     * +x is fowrard
     * +y is left
     * + is ccw
     */
    public void updateSpeed(final ChassisSpeeds speeds) {
        double multiplerRateLimit = Constants.MAX_SPEED_MPS;
        double rotationRateLimit =  Constants.MAX_ROTATION_SPEED_RDPS;
        // Logger.recordOutput("Drive/RateMultiplier", multiplerRateLimit);
        // Logger.recordOutput("Drive/InputSpeed", speeds);

        forwardRateLimiter.setRate(multiplerRateLimit);
        sideRateLimiter.setRate(multiplerRateLimit);
        rotationRateLimiter.setRate(rotationRateLimit);

        speeds.vxMetersPerSecond = forwardRateLimiter.calculate(speeds.vxMetersPerSecond);
        speeds.vyMetersPerSecond = sideRateLimiter.calculate(speeds.vyMetersPerSecond);
        speeds.omegaRadiansPerSecond = rotationRateLimiter.calculate(speeds.omegaRadiansPerSecond);

        // strat 1
        double speedMultiplier =  1;
        speeds.vxMetersPerSecond = speeds.vxMetersPerSecond * speedMultiplier;
        speeds.vyMetersPerSecond = speeds.vyMetersPerSecond * speedMultiplier;
        speeds.omegaRadiansPerSecond = speeds.omegaRadiansPerSecond * speedMultiplier;

        // //strat 2
        // double speedMultiplier2 = -.8*elevator.getCurrentHeightNormalized()+1;
        // double maxSpeed = speedMultiplier2*Constants.MAX_SPEED_MPS;
        // speeds.vxMetersPerSecond = MathUtil.clamp(speeds.vxMetersPerSecond, -maxSpeed , maxSpeed);
        // speeds.vyMetersPerSecond = MathUtil.clamp(speeds.vyMetersPerSecond, -maxSpeed , maxSpeed);

        // Logger.recordOutput("Drive/SpeedFiltered", speeds);
        setStates(swerveDriveKinematics.toSwerveModuleStates(speeds));
    }

    public Command resetOdometryCommand() {
        final Command robert = runOnce(this::resetOdometry).ignoringDisable(true);
        robert.setName("Reset Odometry");
        return robert;
    }

    public Command resetOdometryCommand(final Pose2d trajectory) {
        return runOnce(() -> resetOdometry(trajectory.getTranslation(), trajectory.getRotation()))
                .ignoringDisable(true);
    }

    public void resetOdometry() {
        resetOdometry(new Translation2d(0, 0), Rotation2d.fromDegrees(0));
    }

    public void resetOdometry(final Pose2d pose) {
        resetOdometry(pose.getTranslation(), pose.getRotation());
    }

    public void resetOdometry(final Translation2d position, final Rotation2d direction) {
        poseEstimator.resetPose(new Pose2d(position, direction));
        // swerveDriveOdometry.resetPosition(getRotation(), getModulePositions(), new Pose2d(position, direction));
    }

    @AutoLogOutput(key = "Drive/Rotation")
    public Rotation2d getGyroRotation() {
        return inputs.gyroRotation;
    }

    @AutoLogOutput(key = "Drive/EstimatedPose")
    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public double getYawRate() {
        return inputs.gyroRate;
    }

    public void addVisionMeasurement(
            final Pose2d visionRobotPoseMeters,
            final double timestampSeconds,
            final Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    public boolean isFieldOrientation() {
        return fieldOrientation;
    }

    public void setFieldOrientation(boolean fieldOrientation) {
        this.fieldOrientation = fieldOrientation;
    }

    public void setCurrentAngle(double angle) {
        io.setCurrentAngle(angle);
    }

    public Command doHeadingHoldBlueRelative(Rotation2d rotation) {
        return Commands.runOnce(() -> {
                    Rotation2d rotation2 = rotation;
                    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                        rotation2 = FlippingUtil.flipFieldRotation(rotation2);
                    }
                    headingHoldEnabled = true;
                    headingHold = rotation2;
                })
                .andThen(Commands.run(() -> {}))
                .finallyDo(() -> {
                    headingHoldEnabled = false;
                });
    }

    public Command doPoseHoldBlueRelative(Pose2d pose) {
        return Commands.runOnce(() -> {
                    Pose2d pose2 = pose;
                    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                        pose2 = FlippingUtil.flipFieldPose(pose2);
                    }
                    poseHoldEnabled = true;
                    poseHold = pose2;
                })
                .andThen(Commands.run(() -> {}))
                .finallyDo(() -> {
                    poseHoldEnabled = false;
                });
    }
}
