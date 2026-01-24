package frc.robot.subsystem.turret;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class TurretIOReal implements TurretIO{
    private final SparkFlex turretMotor;

    public TurretIOReal() {
        // NEED ACTUAL IDS
        this.turretMotor = new SparkFlex(16, MotorType.kBrushless);
    }

    @Override
    public void runTurretMotor(final double speed) {
        turretMotor.set(speed);
    }
}
