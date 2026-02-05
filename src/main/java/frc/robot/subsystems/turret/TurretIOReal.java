package frc.robot.subsystems.turret;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

public class TurretIOReal implements TurretIO{
    private final SparkMax turretMotor;
                                                                                                                                                                                                                                    
    public TurretIOReal() {
        this.turretMotor = new SparkMax(16, MotorType.kBrushless);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {      // function that updates the input values located in inputs
    }


    @Override
    public void runTurretMotor(final double speed) {
        turretMotor.set(speed);
    }

  
}
