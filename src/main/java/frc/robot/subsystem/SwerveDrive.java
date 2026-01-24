package frc.robot.subsystem;

import java.util.Arrays;
import java.util.Map;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDrive extends SubsystemBase {
    private final SwerveModule[] swerveModules;
    private final SwerveDriveKinematics swerveDriveKinematics;
    private final SwerveDriveOdometry swerveDriveOdometry;
    private final AHRS gyro;
    //private final GenericEntry speedX;
    //private final GenericEntry speedY;
    //private final GenericEntry speedRot;

    public SwerveDrive(final AHRS gyro, final ShuffleboardLayout layout, final SwerveModule... swerveModules) {
        this.gyro = gyro;
        this.swerveModules = swerveModules;

        final Translation2d[] leverArmArray = Arrays.stream(swerveModules)
                .map(SwerveModule::getLeverArm)
                .toArray(Translation2d[]::new);

        this.swerveDriveKinematics = new SwerveDriveKinematics(leverArmArray);
        swerveDriveOdometry = new SwerveDriveOdometry(swerveDriveKinematics, gyro.getRotation2d(),
                getModulePositions());

        //speedX = layout.add("chassisSpeeds x", 0.0).getEntry();
        //speedY = layout.add("chassisSpeeds y", 0.0).getEntry();
        //speedRot = layout.add("chassisSpeeds rot", 0.0).getEntry();

        try {
            AutoBuilder.configure(
                swerveDriveOdometry::getPoseMeters,
                this::resetOdometry,
                this::getChassisSpeeds,
                this::updateSpeed,
                new PPHolonomicDriveController(
                        new PIDConstants(1, 0, 0),
                        new PIDConstants(1, 0, 0),
                        4.5
                ),
                RobotConfig.fromGUISettings(),
                () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red,
                this
            );
        } catch (Exception e) {
            DriverStation.reportError("AutoBuilder.configure failed", e.getStackTrace());
        }

        layout.addDouble("x", () -> swerveDriveOdometry.getPoseMeters().getX())
                .withPosition(0, 0);
        layout.addDouble("y", () -> swerveDriveOdometry.getPoseMeters().getY())
                .withPosition(1, 0);
        layout.addDouble("Gyro Angle", () -> gyro.getRotation2d().getDegrees())
                .withWidget(BuiltInWidgets.kGyro)
                .withProperties(Map.of("Counter clockwise", true))
                .withPosition(0, 1);
        // layout.addFloat("GyroFusedHeading", () -> -gyro.getFusedHeading())
        //         .withWidget(BuiltInWidgets.kGyro)
        //         .withProperties(Map.of("Counter clockwise", true))
        //         .withPosition(2, 1);
        layout.addDouble("Pose Angle", () -> swerveDriveOdometry.getPoseMeters().getRotation().getDegrees())
                .withWidget(BuiltInWidgets.kGyro)
                .withProperties(Map.of("Counter clockwise", true))
                .withPosition(1, 1);
        layout.add(resetOdometryCommand()).withPosition(0, 2);
        // layout.add("Follow Angle", 0.0)
        //         .withWidget(BuiltInWidgets.kGyro)
        //         .withPosition(1, 2)
        //         .withProperties(Map.of("Counter clockwise", true))
        //         .getEntry();
        layout.addBoolean("IsGyroCalibrating", gyro::isCalibrating)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(2, 2);
    }

    public Command manualDrive(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier omegarad) {
        return run(() -> {
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(vy.getAsDouble()*Constants.MAX_SPEED_MPS*Constants.DRIVE_SPEED
            , vx.getAsDouble()*Constants.MAX_SPEED_MPS*Constants.DRIVE_SPEED
            , omegarad.getAsDouble()*Constants.MAX_SPEED_MPS*Constants.DRIVE_ROTATION);
            updateSpeed(chassisSpeeds);
        });
    }

    @Override
    public void periodic() {
        swerveDriveOdometry.update(getRotation(), getModulePositions());
        for (SwerveModule bob : swerveModules) {
            bob.periodic();
        }
    }

    public ChassisSpeeds getChassisSpeeds() {
        return swerveDriveKinematics.toChassisSpeeds(
                Arrays.stream(swerveModules)
                        .map(SwerveModule::getState)
                        .toArray(SwerveModuleState[]::new));
    }

    public SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(swerveModules)
                .map(SwerveModule::getPosition)
                .toArray(SwerveModulePosition[]::new);
    }

    private void setStates(SwerveModuleState[] states) {
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setState(states[i]);
        }
    }

    public void updateSpeed(ChassisSpeeds speeds) {
        //speedX.setDouble(speeds.vxMetersPerSecond);
        //speedY.setDouble(speeds.vyMetersPerSecond);
        //speedRot.setDouble(speeds.omegaRadiansPerSecond);

        SwerveModuleState[] states = swerveDriveKinematics.toSwerveModuleStates(speeds);
        setStates(states);
    }

    public Command resetOdometryCommand() {
        final Command robert = runOnce(this::resetOdometry).ignoringDisable(true);
        robert.setName("Reset Odometry");
        return robert;
    }

    public Command resetOdometryCommand(Pose2d trajectory) {
        return runOnce(() -> resetOdometry(trajectory.getTranslation(), trajectory.getRotation()))
                .ignoringDisable(true);
    }

    public void resetOdometry() {
        resetOdometry(new Translation2d(0, 0), Rotation2d.fromDegrees(0));
    }

    public void resetOdometry(Pose2d pose) {
        resetOdometry(pose.getTranslation(), pose.getRotation());
    }

    public void resetOdometry(Translation2d position, Rotation2d direction) {
        swerveDriveOdometry.resetPosition(getRotation(), getModulePositions(), new Pose2d(position, direction));
    }

    public Rotation2d getRotation() {
        return gyro.getRotation2d();
    }
}
