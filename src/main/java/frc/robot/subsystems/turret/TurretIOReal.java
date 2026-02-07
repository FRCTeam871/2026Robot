package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

public class TurretIOReal implements TurretIO{
    private final SparkMax turretMotor;
    private final SparkMaxConfig config;
    private final RelativeEncoder m_Turret_Encoder;
    private SparkClosedLoopController m_TurretMotorController;
                                                                                                                                                                                                           
    public TurretIOReal() {
        this.turretMotor = new SparkMax(16, MotorType.kBrushless);
        m_Turret_Encoder = turretMotor.getEncoder();
        this.m_TurretMotorController = turretMotor.getClosedLoopController();
        
        this.config = new SparkMaxConfig();
        updatePIDConstants(0.005, 0, 0.0, 0, 0, 0);
        SmartDashboard.putData(applyPIDConstants());
        config.apply(new SoftLimitConfig().reverseSoftLimit(.2).forwardSoftLimit(-.2));

    }

    public void updatePIDConstants(double kP, double kI, double kD, double kS, double kV, double kA) {
        config.closedLoop
        .p(kP, ClosedLoopSlot.kSlot1)
        .i(kI, ClosedLoopSlot.kSlot1)
        .d(kD, ClosedLoopSlot.kSlot1)
        .outputRange(0, 0);
        config.closedLoop.feedForward.kS(kS, ClosedLoopSlot.kSlot1).kV(kV, ClosedLoopSlot.kSlot1).kA(kA, ClosedLoopSlot.kSlot1);
        turretMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        SmartDashboard.putNumber("Turret/PID/kP", kP);
        SmartDashboard.putNumber("Turret/PID/kI", kI);
        SmartDashboard.putNumber("Turret/PID/kD", kD);
        SmartDashboard.putNumber("Turret/PID/kS", kS);
        SmartDashboard.putNumber("Turret/PID/kV", kV);
        SmartDashboard.putNumber("Turret/PID/kA", kA);
        System.out.printf("%f %f %f %f %f %f \n", kP, kI, kD, kS, kV, kA);
    }

     public Command applyPIDConstants(){
        return Commands.runOnce(()->{
            double kP = SmartDashboard.getNumber("Turret/PID/kP", 0);
            double kI = SmartDashboard.getNumber("Turret/PID/kI", 0);
            double kD = SmartDashboard.getNumber("Turret/PID/kD", 0);
            double kS = SmartDashboard.getNumber("Turret/PID/kS", 0);
            double kV = SmartDashboard.getNumber("Turret/PID/kV", 0);
            double kA = SmartDashboard.getNumber("Turret/PID/kA", 0);
            updatePIDConstants(kP, kI, kD, kS, kV, kA);
        }).withName("apply1").ignoringDisable(true);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {      // function that updates the input values located in inputs
        inputs.setpointAngle = Units.Rotations.of(m_TurretMotorController.getSetpoint());
        inputs.turretAngle = Units.Rotation.of(m_Turret_Encoder.getPosition());
    }


    @Override
    public void runTurretMotor(final double speed) {
        turretMotor.set(speed);
    }

    @Override
    public void setTarget(Angle angle) {
        m_TurretMotorController.setSetpoint(angle.in(Rotations), ControlType.kPosition, ClosedLoopSlot.kSlot1);
    }
}
