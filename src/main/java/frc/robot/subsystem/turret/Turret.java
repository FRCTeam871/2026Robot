package frc.robot.subsystem.turret;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

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
        
    

    public Command runTurretMotor(DoubleSupplier speed){
        return run(()-> {
            io.runTurretMotor(speed.getAsDouble());
        }).finallyDo(()->{
            io.runTurretMotor(0);
        });
    }
}   

