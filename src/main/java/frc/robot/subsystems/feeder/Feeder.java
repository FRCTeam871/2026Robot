package frc.robot.subsystems.feeder;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
    private final FeederIO io;
    // private SparkFlex turretMotor;

    public Feeder(FeederIO io){
        // Motor TBD
        // turretMotor = new SparkFlex(2, MotorType.kBrushless);
        this.io = io;
    }
    
    

    public Command runFeederMotor(double speed){
        return run(()-> {
            io.runFeederMotor(speed);
        }).finallyDo(()->{
            io.runFeederMotor(0);
        });
    }
    public void setYawSetpoint(Angle angle){
        // turretAngle.set(angle);
    }
    public void runDumbFeederMotor(double speed){
        io.runFeederMotor(speed);
    }
}   

