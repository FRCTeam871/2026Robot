package frc.robot.subsystems.indexer;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Velocity;

public class IndexerIOReal implements IndexerIO{
    private final SparkFlex indexMotor;

    public IndexerIOReal() {
        this.indexMotor =  new SparkFlex(14, MotorType.kBrushless);
    }

    @Override
    public void runIndexMotor(final double speed) {
        indexMotor.set(speed);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {/* blenk */}


}
