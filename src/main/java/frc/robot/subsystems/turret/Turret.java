package frc.robot.subsystems.turret;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    private final TurretIO io;
    // private SparkFlex turretMotor;

    public Turret(TurretIO io){
        // Motor TBD
        // turretMotor = new SparkFlex(2, MotorType.kBrushless);
        this.io = io;
    }
    
    public Pose3d findPoseOfFuelRelease() {
        //TODO: find real number
        //uses current angle
        return Pose3d.kZero;
    }
    
    public Pose3d targetPoseOfFuelRelease(){
        //TODO: make real
        //uses setpoint angle
        return Pose3d.kZero;
    }

    public Pose3d findTurretPose(){
        //TODO: make real
        return Pose3d.kZero;
    }
    
    public Command runTurretMotor(DoubleSupplier speed){
        return run(()-> {
            io.runTurretMotor(speed.getAsDouble());
        }).finallyDo(()->{
            io.runTurretMotor(0);
        });
    }
    public void setYawSetpoint(Angle angle){
        // turretAngle.set(angle);
    }
}   

