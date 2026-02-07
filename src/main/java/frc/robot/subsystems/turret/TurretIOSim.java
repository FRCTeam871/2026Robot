package frc.robot.subsystems.turret;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

public class TurretIOSim implements TurretIO{
    Angle targetAngle;                                                                                                                                                                                                                                    
    public TurretIOSim() {
        targetAngle = Units.Degrees.of(0);
        
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {      // function that updates the input values located in inputs
        inputs.turretAngle = Units.Degrees.of(3);
        inputs.setpointAngle = targetAngle;
    }


    @Override
    public void runTurretMotor(final double speed) {
    }

    @Override
    public void setTarget(Angle angle) {
        targetAngle = angle;
    }
        
    

  
}
