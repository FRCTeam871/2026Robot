package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.lang.StackWalker.Option;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.databind.introspect.TypeResolutionContext.Empty;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.fieldtracking.FieldTracking;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;

public class Aiming extends SubsystemBase {
    Turret turret;
    Shooter shooter;
    FieldTracking fieldtracking;
    Translation3d hub;

    public Aiming(Turret turret, Shooter shooter, FieldTracking fieldtracking) {
        this.turret = turret;
        this.shooter = shooter;
        this.fieldtracking = fieldtracking;
        hub = new Translation3d(Units.Inches.of(158.6 + (47.0/2.0)),Units.Inches.of(317.7/2.0),Units.Feet.of(6.5));
    }

    @Override
    public void periodic() {
        Translation3d turretTranslation;
        turretTranslation = new Translation3d(4 + Math.sin(Timer.getFPGATimestamp() * 2) * 2, 4 + Math.cos(Timer.getFPGATimestamp()) * 2,.5);
        Pose3d initialPose = new Pose3d(
                        turretTranslation,
                        new Rotation3d(
                                Units.Degrees.of(0),
                                Units.Degrees.of(77),
                                calculateAngle(turretTranslation, hub)));
        Optional<LinearVelocity> launchVelocity = calculateLaunchSpeed(turretTranslation, hub);
        Logger.recordOutput("Aiming/launchVelocity", launchVelocity.orElse(Units.MetersPerSecond.of(0)));
        boolean isTrajectoryInvalid = launchVelocity.isEmpty() || launchVelocity.get().gt(Units.MetersPerSecond.of(10)) || launchVelocity.get().lt(Units.MetersPerSecond.of(5));
        if(isTrajectoryInvalid){
            Translation3d[] fuelTrajectory;
            fuelTrajectory = new Translation3d[0];
        Logger.recordOutput("Aiming/TrajectoryPath", fuelTrajectory);
        Logger.recordOutput("Aiming/InitialPose", initialPose);
        }else {
       
                makeFuelTrajectoryArray(
                        launchVelocity.get(),
                        initialPose);
    }
        }
       

    public Command aim() {
        return run(() -> {
            setDesiredYaw();
            boolean jaydenCalcanoTheHumanCalculator = false;
            if (jaydenCalcanoTheHumanCalculator) {
                setDesiredRPM();
            }
        });
    }

    public void setDesiredYaw() {
        Translation3d turretPose = turret.findTurretPose().getTranslation();

        turret.setYawSetpoint(calculateAngle(turretPose, hub));
    }
    public Angle calculateAngle(Translation3d start, Translation3d end) {
        
        double yDisp = end.getY()-start.getY();
        double xDisp = end.getX()-start.getX();
        Angle desiredYaw = Units.Radians.of(Math.atan2(yDisp, xDisp));
        return desiredYaw;  
    }

    public void setDesiredRPM() {
 

        LinearVelocity desiredShootSpeed = null;
        makeFuelTrajectoryArray(desiredShootSpeed, turret.targetPoseOfFuelRelease());
        shooter.setShooterSpeed(desiredShootSpeed);
    }
    public Optional<LinearVelocity> calculateLaunchSpeed(Translation3d start, Translation3d end){
        double yDisp = end.getY()-start.getY();
        double xDisp = end.getX()-start.getX();
        Distance displacementH = Units.Meters.of(Math.sqrt((xDisp*xDisp)+(yDisp*yDisp)));
        Distance displacementV = end.getMeasureZ().minus(start.getMeasureZ());
        Angle pitchAngle = Units.Degrees.of(77);
        double billy =  ((2/Constants.GRAVITY.in(MetersPerSecondPerSecond))*
        (displacementV.minus(displacementH.times(Math.tan(pitchAngle.in(Radians))))).in(Meters));
        if(billy < 0){
            return Optional.empty();
        } 
        LinearVelocity velocityInitial = 
        Units.MetersPerSecond.of(displacementH.in(Meters)/
        (Math.cos(pitchAngle.in(Radians))*Math.sqrt(billy)));
        return Optional.of(velocityInitial);
    }

    public void makeFuelTrajectoryArray(LinearVelocity desiredShootSpeed, Pose3d initialPose) {
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
        Logger.recordOutput("Aiming/TrajectoryPath", fuelTrajectory);
        Logger.recordOutput("Aiming/InitialPose", initialPose);
    }

    public Translation3d findBallPosition(Time time, Pose3d initialPose, LinearVelocity launchSpeed) {

        LinearAcceleration gravity = Units.MetersPerSecondPerSecond.of(-9.807);
        double angle = initialPose.getRotation().getY(); // launch angle
        LinearVelocity initialVelocityY = launchSpeed.times(Math.sin(angle));
        LinearVelocity initialVelocityX = launchSpeed.times(Math.cos(angle));
        Distance displacementX = initialVelocityX.times(time);
        Distance displacementY = initialVelocityY.times(time).plus(gravity.times(0.5).times(time).times(time)); // time
                                                                                                                // times
                                                                                                                // time
                                                                                                                // times
                                                                                                                // time
                                                                                                                // times

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
}
