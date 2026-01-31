package frc.robot.subsystems.turret;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose3d;

public class TurretIOReal implements TurretIO{
    private final SparkFlex turretMotor;

    public TurretIOReal() {
        this.turretMotor = new SparkFlex(16, MotorType.kBrushless);
    }

    @Override
    public void runTurretMotor(final double speed) {
        turretMotor.set(speed);
    }

  
}
