package frc.robot.subsystem.shooter;

import static edu.wpi.first.units.Units.RPM;

import java.nio.file.WatchEvent.Kind;
import java.util.function.DoubleSupplier;

import javax.xml.crypto.dsig.keyinfo.KeyValue;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ShooterIOReal implements ShooterIO {
    private final SparkFlex shooterMotor;
    private final SparkFlexConfig config;
    private final RelativeEncoder m_shooterEncoder;
    private SparkClosedLoopController m_shooterMotorController;
    PIDController shooterPIDController;
    SysIdRoutine routine;
    SimpleMotorFeedforward shooterMotorFeedForward;

    public ShooterIOReal() {
        this.shooterMotor = new SparkFlex(13, MotorType.kBrushless);
        m_shooterEncoder = shooterMotor.getEncoder();
        this.m_shooterMotorController = shooterMotor.getClosedLoopController();
        

        this.config = new SparkFlexConfig();
        updatePIDConstants(0.0035, 0, 0.000165, 0.0035, 0.001751, 0.00040043);
        SmartDashboard.putData(applyPIDConstants());

        shooterMotorFeedForward = new SimpleMotorFeedforward(0.0035, 0.001751, 0.00040043);  // PLACEHOLAR
        shooterPIDController = new PIDController(0.0035, 0, 0.000165);
        
    }

    public void updatePIDConstants(double kP, double kI, double kD, double kS, double kV, double kA) {
        config.closedLoop
        .p(kP, ClosedLoopSlot.kSlot1)
        .i(kI, ClosedLoopSlot.kSlot1)
        .d(kD, ClosedLoopSlot.kSlot1)
        .outputRange(0, 0);
        config.closedLoop.feedForward.kS(kS, ClosedLoopSlot.kSlot1).kV(kV, ClosedLoopSlot.kSlot1).kA(kA, ClosedLoopSlot.kSlot1);
        shooterMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        SmartDashboard.putNumber("Shooter/PID/kP", kP);
        SmartDashboard.putNumber("Shooter/PID/kI", kI);
        SmartDashboard.putNumber("Shooter/PID/kD", kD);
        SmartDashboard.putNumber("Shooter/PID/kS", kS);
        SmartDashboard.putNumber("Shooter/PID/kV", kV);
        SmartDashboard.putNumber("Shooter/PID/kA", kA);
    }
    
    @Override
    public void updateInputs(ShooterIOInputs inputs) {      // function that updates the input values located in inputs
        
        inputs.velocity = Units.RPM.of(m_shooterEncoder.getVelocity());
        inputs.motorVoltage = Units.Volts.of(shooterMotor.getAppliedOutput()*shooterMotor.getBusVoltage());
        inputs.position = Units.Rotations.of(m_shooterEncoder.getPosition());
    }

    @Override
    public void setMotorSetpoint(final AngularVelocity rpmSetpoint) {
        m_shooterMotorController.setSetpoint(rpmSetpoint.in(RPM), ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    }

    @Override
    public void runMotorSpeed(final double speed) {
        shooterMotor.set(speed);
    }
    @Override
    public void setVoltage(Voltage v) {
        shooterMotor.setVoltage(v);
    }
    
    public Command applyPIDConstants(){
        return Commands.runOnce(()->{
            double kP = SmartDashboard.getNumber("Shooter/PID/kP", 0);
            double kI = SmartDashboard.getNumber("Shooter/PID/kI", 0);
            double kD = SmartDashboard.getNumber("Shooter/PID/kD", 0);
            double kS = SmartDashboard.getNumber("Shooter/PID/kS", 0);
            double kV = SmartDashboard.getNumber("Shooter/PID/kV", 0);
            double kA = SmartDashboard.getNumber("Shooter/PID/kA", 0);
            updatePIDConstants(kP, kI, kD, kS, kV, kA);
        }).withName("apply");
    }

}
