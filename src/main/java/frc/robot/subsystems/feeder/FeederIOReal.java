package frc.robot.subsystems.feeder;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose3d;

public class FeederIOReal implements FeederIO{
    private final SparkFlex feederMotor;

    public FeederIOReal() {
        // NEED ACTUAL IDS
        this.feederMotor = new SparkFlex(17, MotorType.kBrushless);
    }

    @Override
    public void runFeederMotor(final double speed) {
        feederMotor.set(speed);
    }

  
}
