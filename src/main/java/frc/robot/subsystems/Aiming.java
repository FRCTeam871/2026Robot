package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.lang.StackWalker.Option;
import java.util.ArrayList;
import java.util.EmptyStackException;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.databind.introspect.TypeResolutionContext.Empty;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.fieldtracking.FieldTracking;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import frc.robot.subsystems.turret.Turret;

public class Aiming extends SubsystemBase {
    Turret turret;
    Shooter shooter;
    FieldTracking fieldtracking;
    SwerveDrive swerveDrive;
    Sequencing sequencing;
    Optional<LinearVelocity> desiredShootSpeed = Optional.empty();
    boolean wantToShoot;
    Translation3d hubBlue;
    Translation3d leftSideShootBlue;
    Translation3d rightSideShootBlue;
    boolean manualAim = false;

    public Aiming(Turret turret, Shooter shooter, FieldTracking fieldtracking, SwerveDrive swerveDrive,
            Sequencing sequencing) {
        this.turret = turret;
        this.shooter = shooter;
        this.fieldtracking = fieldtracking;
        this.swerveDrive = swerveDrive;
        this.sequencing = sequencing;
        wantToShoot = false;

        hubBlue = new Translation3d(Constants.HUB_POSITION.getMeasureX(), Constants.HUB_POSITION.getMeasureY(),
                Units.Feet.of(6));
        leftSideShootBlue = new Translation3d(Units.Inches.of(79.3), Units.Inches.of(238.275), Units.Inches.of(0.0));
        rightSideShootBlue = new Translation3d(Units.Inches.of(79.3), Units.Inches.of(79.425), Units.Inches.of(0.0));
        setDefaultCommand(findSpeed().alongWith(fire()).ignoringDisable(true));
        }

    @Override
    public void periodic() {
        Logger.recordOutput("Aiming/wantToShoot", wantToShoot);
    }

    public Command findSpeed() {
        return run(() -> {
            Logger.recordOutput("Aiming/RobotPose", swerveDrive.getEstimatedPose());
            // Logger.recordOutput("Aiming/TurretPose", turret.currentTurretPose());
            // Logger.recordOutput("Aiming/CurrentFuelReleasePose", turret.currentPoseOfFuelRelease());
            // Logger.recordOutput("Aiming/TargetTurretPose", turret.targetTurretPose());
            // Logger.recordOutput("Aiming/TargetPoseOfFuelRelease", turret.targetPoseOfFuelRelease());

            Translation3d target = hubBlue;
            Translation3d turretTranslation = turret.currentTurretPose().getTranslation();
            if (turretTranslation.getMeasureX().minus(Constants.centerOfField.getMeasureX()).abs(Inches) < (651.2 / 2)
                    - 158.6) {
                if (turretTranslation.getY() > 8.06958 / 2
                        ^ DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                    target = leftSideShootBlue;
                } else {
                    target = rightSideShootBlue;
                }
            }
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                Translation2d flipped = FlippingUtil.flipFieldPosition(target.toTranslation2d());
                target = new Translation3d(flipped.getX(), flipped.getY(), target.getZ());
            }
            Logger.recordOutput("Aiming/Target", target);
            if(manualAim){
                turret.setYawSetpoint(Units.Degrees.of(0));
            } else{
            turret.setYawSetpoint(calculateAngle(turretTranslation, target)
                    .minus(swerveDrive.getEstimatedPose().getRotation().getMeasure()));
            }

            desiredShootSpeed = calculateLaunchSpeed(turret.targetPoseOfFuelRelease(), target);

            // Logger.recordOutput("Aiming/desiredShootSpeed", desiredShootSpeed.orElse(Units.MetersPerSecond.of(0)));
            // Logger.recordOutput("Aiming/realDesiredShootSpeed", desiredShootSpeed.orElse(Units.MetersPerSecond.of(0)));

            boolean isTrajectoryInvalid = desiredShootSpeed.isEmpty()
                    || desiredShootSpeed.get().gt(Units.MetersPerSecond.of(20))
                    || desiredShootSpeed.get().lt(Units.MetersPerSecond.of(1));

            makeFuelTrajectoryArray(
                    shooter.getShooterSpeed(),
                    turret.currentPoseOfFuelRelease(),
                    true);

            if (isTrajectoryInvalid) {
                Translation3d[] fuelTrajectory;
                fuelTrajectory = new Translation3d[0];
                Logger.recordOutput("Aiming/TrajectoryPath", fuelTrajectory);
                // Logger.recordOutput("Aiming/InitialPose", turret.targetPoseOfFuelRelease());
                desiredShootSpeed = Optional.empty();
                // Logger.recordOutput("Aiming/realDesiredShootSpeed",
                        // desiredShootSpeed.orElse(Units.MetersPerSecond.of(0)));
                return;
            }

            makeFuelTrajectoryArray(
                    desiredShootSpeed.get(),
                    turret.targetPoseOfFuelRelease(),
                    false);

            if (!wantToShoot) {
                desiredShootSpeed = Optional.empty();
                // Logger.recordOutput("Aiming/realDesiredShootSpeed",
                //         desiredShootSpeed.orElse(Units.MetersPerSecond.of(0)));
                return;
            }
            
        });
    }

    private Command fire() {
        Command doShoot = sequencing.shooterCommand(() -> 
            shooter.convertShootSpeedToRPM(desiredShootSpeed.orElse(Units.MetersPerSecond.of(0))))
            .until(()-> desiredShootSpeed.isEmpty());  

        return Commands.run(() -> {
            // Logger.recordOutput("Aiming/doShoot", doShoot.isScheduled());
            if (desiredShootSpeed.isPresent()) {
                // start command if not running
                if (!doShoot.isScheduled()) {
                    CommandScheduler.getInstance().schedule(doShoot);
                }
            } 
        });
    }

    public Command shootTrue() {
        return Commands.run(() -> {
            wantToShoot = true;
        }).finallyDo(() -> {
            wantToShoot = false;
        });
    }

    public Angle calculateAngle(Translation3d start, Translation3d end) {

        double yDisp = (end.getY() - start.getY());
        double xDisp = end.getX() - start.getX();
        Angle desiredYaw = Units.Radians.of(Math.atan2(yDisp, xDisp));
        return desiredYaw;
    }

    public Optional<LinearVelocity> calculateLaunchSpeed(Pose3d start, Translation3d end) {
        double yDisp = end.getY() - start.getY();
        double xDisp = end.getX() - start.getX();
        Distance displacementH = Units.Meters.of(Math.sqrt((xDisp * xDisp) + (yDisp * yDisp)));
        Distance displacementV = end.getMeasureZ().minus(start.getMeasureZ());
        Angle pitchAngle = start.getRotation().getMeasureY().unaryMinus();
        double actuallydescriptive = ((2 / Constants.GRAVITY.in(MetersPerSecondPerSecond)) *
                (displacementV.minus(displacementH.times(Math.tan(pitchAngle.in(Radians))))).in(Meters));
        if (actuallydescriptive < 0) {
            return Optional.empty();
        }
        LinearVelocity velocityInitial = Units.MetersPerSecond.of(displacementH.in(Meters) /
                (Math.cos(pitchAngle.in(Radians)) * Math.sqrt(actuallydescriptive)));
        return Optional.of(velocityInitial);
    }

    public void makeFuelTrajectoryArray(LinearVelocity desiredShootSpeed, Pose3d initialPose,
            boolean CurrentTrajectory) {
        List<Translation3d> fuelPositions = new ArrayList<>();
        Distance ground = Units.Inches.of(0);
        Translation3d fuelPosition = initialPose.getTranslation();
        for (Time t = Units.Seconds.of(0); t.lt(Units.Seconds.of(15))
                && fuelPosition.getMeasureZ().gte(ground); t = t.plus(Units.Seconds.of(.1))) {
            fuelPosition = findBallPosition(t, initialPose, desiredShootSpeed);
            fuelPositions.add(fuelPosition);
            //
        }
        Translation3d[] fuelTrajectory = fuelPositions.toArray(size -> new Translation3d[size]);
        if (CurrentTrajectory) {
            Logger.recordOutput("Aiming/TrajectoryPathCurrent", fuelTrajectory);
        } else {
            Logger.recordOutput("Aiming/TrajectoryPath", fuelTrajectory);
        }
    }

    public Translation3d findBallPosition(Time time, Pose3d initialPose, LinearVelocity launchSpeed) {

        double angle = -initialPose.getRotation().getY(); // launch angle
        LinearVelocity initialVelocityY = launchSpeed.times(Math.sin(angle));
        LinearVelocity initialVelocityX = launchSpeed.times(Math.cos(angle));
        Distance displacementX = initialVelocityX.times(time);
        Distance displacementY = initialVelocityY.times(time)
                .plus(Constants.GRAVITY.times(0.5).times(time).times(time)); // time
        Translation3d xDirection = new Translation3d(1, 0, 0).rotateBy(initialPose.getRotation());
        xDirection = new Translation3d(xDirection.getX(), xDirection.getY(), 0);
        xDirection = xDirection.div(xDirection.getNorm());
        xDirection = new Translation3d(
                displacementX.times(xDirection.getX()),
                displacementX.times(xDirection.getY()),
                displacementX.times(xDirection.getZ()));

        Translation3d yDirection = new Translation3d(
                Units.Meters.of(0),
                Units.Meters.of(0),
                displacementY);

        return initialPose.getTranslation().plus(xDirection.plus(yDirection));
    }
      public Command manualTurret(){
        return Commands.run(()-> manualAim = true).finallyDo(()-> manualAim = false);
    }

}

