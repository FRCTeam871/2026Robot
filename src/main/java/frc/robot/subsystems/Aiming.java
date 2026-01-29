package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.fieldtracking.FieldTracking;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;

public class Aiming extends SubsystemBase {
    Turret turret;
    Shooter shooter;
    FieldTracking fieldtracking;

    public Aiming(Turret turret, Shooter shooter, FieldTracking fieldtracking) {
        this.turret = turret;
        this.shooter = shooter;
        this.fieldtracking = fieldtracking;
    }

    public Command aim() {
        return run(() -> {
            // TODO: do real math and real code
            setDesiredYaw();
            boolean bob = false;
            if(bob){
                setDesiredRPM();
            }
        });
    }

    public void setDesiredYaw() {
        Translation3d turretPose = turret.findTurretPose().getTranslation();
        Translation3d hub = Translation3d.kZero;



        Angle desiredYaw = null;
        turret.setYawSetpoint(desiredYaw);
    }

    public void setDesiredRPM() {
        Pose3d targetPoseOfFuelRelease = turret.targetPoseOfFuelRelease();
        Distance height = null;
        Distance distance = null;
        Angle pitchAngle = null;



        LinearVelocity desiredShootSpeed = null;
        makeFuelTrajectoryArray(desiredShootSpeed, turret.targetPoseOfFuelRelease());
        shooter.setShooterSpeed(desiredShootSpeed);
    }

    public void makeFuelTrajectoryArray(LinearVelocity desiredShootSpeed, Pose3d initialPose){
        List<Translation3d> fuelPositions = new ArrayList<>();
        Distance ground = Units.Inches.of(0);
        Translation3d fuelPosition = initialPose.getTranslation();
        for(Time t = Units.Seconds.of(0); fuelPosition.getMeasureY().gte(ground) ; t = t.plus(Units.Seconds.of(.1))){
            fuelPosition = findBallPosition(t, initialPose, desiredShootSpeed);
            fuelPositions.add(fuelPosition); 
        }
        Translation3d[] fuelTrajectory = fuelPositions.toArray(size -> new Translation3d[size]);
        Logger.recordOutput("Aiming/TrajectoryPath", fuelTrajectory);
    }

    public Translation3d findBallPosition(Time time, Pose3d initialPose, LinearVelocity launchSpeed){
        double angle = initialPose.getRotation().getY();        // launch angle
        LinearVelocity initialVelocityY = launchSpeed.times(Math.sin(angle));
        LinearVelocity initialVelocityX = launchSpeed.times(Math.cos(angle));
        return null;
    }
}
