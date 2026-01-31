package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;

import java.text.Normalizer;
import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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

    @Override
    public void periodic() {
        makeFuelTrajectoryArray(Units.MetersPerSecond.of(9+Math.cos(Timer.getFPGATimestamp())*.5), new Pose3d(2, 6, .5, new Rotation3d(Math.toRadians(0), Math.toRadians(77+Math.sin(Timer.getFPGATimestamp())*5), Math.toRadians(-35))));
        
    }

    public Command aim() {
        return run(() -> {
            setDesiredYaw();
            boolean jaydenCalcanoTheHumanCalculator = false;
            if(jaydenCalcanoTheHumanCalculator){
                setDesiredRPM();
            }
        });
    }

    public void setDesiredYaw() {
        Translation3d turretPose = turret.findTurretPose().getTranslation();
        Translation3d hub = Translation3d.kZero;
        


        Angle desiredYaw = Units.Degrees.of(77);
        turret.setYawSetpoint(desiredYaw);
    }

    public void setDesiredRPM() {
        Pose3d targetPoseOfFuelRelease = turret.targetPoseOfFuelRelease();
        Distance height = Distance.ofBaseUnits(6, Feet);
        Distance distance = null;
        Angle pitchAngle = null;



        LinearVelocity desiredShootSpeed = null;
        makeFuelTrajectoryArray(desiredShootSpeed, turret.targetPoseOfFuelRelease());
        shooter.setShooterSpeed(desiredShootSpeed);
    }

    public void makeFuelTrajectoryArray(LinearVelocity desiredShootSpeed, Pose3d initialPose) {
        List<Translation3d> fuelPositions = new ArrayList<>();
        Distance ground = Units.Inches.of(0);
        Translation3d fuelPosition = initialPose.getTranslation();
        for(Time t = Units.Seconds.of(0); t.lt(Units.Seconds.of(15)) && fuelPosition.getMeasureZ().gte(ground); t = t.plus(Units.Seconds.of(.1))){
            fuelPosition = findBallPosition(t, initialPose, desiredShootSpeed);
            fuelPositions.add(fuelPosition); 
            // 
        }
        Translation3d[] fuelTrajectory = fuelPositions.toArray(size -> new Translation3d[size]);
        Logger.recordOutput("Aiming/TrajectoryPath", fuelTrajectory);
        Logger.recordOutput("Aiming/InitialPose", initialPose);
    }

    public Translation3d findBallPosition(Time time, Pose3d initialPose, LinearVelocity launchSpeed){
        
        
        LinearAcceleration gravity = Units.MetersPerSecondPerSecond.of(-9.807);
        double angle = initialPose.getRotation().getY();        // launch angle
        LinearVelocity initialVelocityY = launchSpeed.times(Math.sin(angle));
        LinearVelocity initialVelocityX = launchSpeed.times(Math.cos(angle));
        Distance displacementX = initialVelocityX.times(time);
        Distance displacementY = initialVelocityY.times(time).plus(gravity.times(0.5).times(time).times(time));     // time times time times time times

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
            displacementY
        );

        return initialPose.getTranslation().plus(xDirection.plus(yDirection));
    }
}
