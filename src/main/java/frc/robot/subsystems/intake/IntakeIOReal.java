package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;;

// ---------------------------------------------------------------------------------------------------------------------------------

public class IntakeIOReal implements IntakeIO {
    private final SparkFlex intakeMotor;
    private final DoubleSolenoid intakePiston;
    public IntakeIOReal() {
        this.intakeMotor = new SparkFlex(15, MotorType.kBrushless);
        this.intakePiston = new DoubleSolenoid(0, null, 0, 0);
    }
    @Override
    public void setSpeed(double speed) {
        intakeMotor.set(speed);
    }
    @Override
    public void setIntakeOut(boolean extend){
        if (extend){
            intakePiston.set(Value.kForward);
        }
        else{
            intakePiston.set(Value.kReverse);
        }
    }
}
