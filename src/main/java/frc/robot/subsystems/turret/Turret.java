package frc.robot.subsystems.turret;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import frc.robot.subsystems.turret.TurretIO.TurretIOInputs;

public class Turret extends SubsystemBase {
    private final TurretIO io;
    private final SwerveDrive swerveDrive;
    private final TurretIOInputs inputs = new TurretIOInputsAutoLogged();

    // private SparkFlex turretMotor;

    public Turret(TurretIO io, SwerveDrive swerveDrive) {
        // Motor TBD
        // turretMotor = new SparkFlex(2, MotorType.kBrushless);
        this.io = io;
        this.swerveDrive = swerveDrive;

    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public Pose3d currentPoseOfFuelRelease() {
        Transform3d fuelReleasePoseRelative = new Transform3d(Units.Inches.of(7), Units.Inches.of(0),
                Units.Inches.of(6), new Rotation3d(Units.Degrees.of(0), Units.Degrees.of(-77), Units.Degrees.of(0)));
        return currentTurretPose().plus(fuelReleasePoseRelative);
    }

    public Pose3d targetPoseOfFuelRelease() {
        Transform3d fuelReleasePoseRelative = new Transform3d(Units.Inches.of(7), Units.Inches.of(0),
                Units.Inches.of(6), new Rotation3d(Units.Degrees.of(0), Units.Degrees.of(-77), Units.Degrees.of(0)));
        return targetTurretPose().plus(fuelReleasePoseRelative);
    }

    public Pose3d currentTurretPose() {
        Pose3d currentRobotPose = new Pose3d(swerveDrive.getEstimatedPose());
        // Pose3d currentRobotPose = new Pose3d(Math.cos(5),Math.cos(2), 0,new
        // Rotation3d(0,0,0));
        Transform3d turretPoseRelative = new Transform3d(Units.Inches.of(-11), Units.Inches.of(0), Units.Inches.of(20),
                new Rotation3d(Units.Degrees.of(0), Units.Degrees.of(0), inputs.turretAngle));
        return currentRobotPose.plus(turretPoseRelative);
    }

    public Pose3d targetTurretPose() {
        Pose3d currentRobotPose = new Pose3d(swerveDrive.getEstimatedPose());
        // Pose3d currentRobotPose = new Pose3d(Math.cos(5),Math.cos(2), 0,new
        // Rotation3d(0,0,0));
        Transform3d turretPoseRelative = new Transform3d(Units.Inches.of(-11), Units.Inches.of(0), Units.Inches.of(20),
                new Rotation3d(Units.Degrees.of(0), Units.Degrees.of(0), inputs.setpointAngle));
        return currentRobotPose.plus(turretPoseRelative);
    }

    public Command runTurretMotor(DoubleSupplier speed) {
        return run(() -> {
            io.runTurretMotor(speed.getAsDouble());
        }).finallyDo(() -> {
            io.runTurretMotor(0);
        });
    }

    public void setYawSetpoint(Angle angle) {
        io.setTarget(angle);
    }
}
