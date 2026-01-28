package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage; 
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command; 
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.shooter.ShooterIOInputsAutoLogged;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

//https://www.reca.lc/flywheel?currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=50&flywheelMomentOfInertia=%7B%22s%22%3A0%2C%22u%22%3A%22in2%2Albs%22%7D&flywheelRadius=%7B%22s%22%3A0%2C%22u%22%3A%22in%22%7D&flywheelRatio=%7B%22magnitude%22%3A1%2C%22ratioType%22%3A%22Reduction%22%7D&flywheelWeight=%7B%22s%22%3A0%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22NEO%20Vortex%22%7D&motorRatio=%7B%22magnitude%22%3A1.33%2C%22ratioType%22%3A%22Reduction%22%7D&projectileRadius=%7B%22s%22%3A2%2C%22u%22%3A%22in%22%7D&projectileWeight=%7B%22s%22%3A0.5%2C%22u%22%3A%22lbs%22%7D&shooterMomentOfInertia=%7B%22s%22%3A5%2C%22u%22%3A%22in2%2Albs%22%7D&shooterRadius=%7B%22s%22%3A2%2C%22u%22%3A%22in%22%7D&shooterTargetSpeed=%7B%22s%22%3A5000%2C%22u%22%3A%22rpm%22%7D&shooterWeight=%7B%22s%22%3A2.5%2C%22u%22%3A%22lbs%22%7D&useCustomFlywheelMoi=0&useCustomShooterMoi=0
public class Shooter extends SubsystemBase {
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutVoltage m_appliedVoltage = Volts.mutable(0); 
    // Mutable holder for unit-safe linear distance values, persisted to avoid
    // reallocation.
    private final MutAngle m_angle = Radians.mutable(0); 
    // Mutable holder for unit-safe linear velocity values, persisted to avoid
    // reallocation.
    private final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);
    SysIdRoutine routine;
    ShooterIO io;
    private final ShooterIOInputs inputs = new ShooterIOInputsAutoLogged();
    
    public Shooter(ShooterIO io) {
        this.io = io;
        routine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(this::motorVoltageSetter, this::motorLogSetter, this));
        inputs.velocity.in(Units.RPM);
    }

    private void motorVoltageSetter(Voltage v) {
        io.setVoltage(v);
    }

    
    private AngularVelocity convertShootSpeedToRPM(LinearVelocity shootSpeed) {
         AngularVelocity desiredRPM = null;
         return desiredRPM;
    }

    public void setShooterSpeed(LinearVelocity speed){
        AngularVelocity RPM = convertShootSpeedToRPM(speed);
        io.setMotorSetpoint(RPM);
    }

    private void motorLogSetter(SysIdRoutineLog log) {
        log.motor("shooter-wheel")
            .voltage(
                m_appliedVoltage.mut_replace(
                    inputs.motorVoltage))
            .angularPosition(m_angle.mut_replace(inputs.position))
            .angularVelocity(m_velocity.mut_replace(inputs.velocity));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        // Logger.recordOutput("RPM", inputs.velocity);
    }

    public Command runMotorSpeed(double speed){
        return run(()-> {
            io.runMotorSpeed(speed);
        }).finallyDo(()->{
            io.runMotorSpeed(0);
        });
    }

    public Command holdMotorSetpoint(AngularVelocity rpmSetpoint){
        return run(()-> {
            Logger.recordOutput("rpmSetpoint", rpmSetpoint);
            io.setMotorSetpoint(rpmSetpoint);

        }).finallyDo(() -> {
            io.runMotorSpeed(0);
             Logger.recordOutput("rpmSetpoint", 0);
        });
    }

    public Command quasiStatic(Direction direction){
        return routine.quasistatic(direction).finallyDo(() -> {
            io.runMotorSpeed(0);
        });
    }
    public Command dynamic(Direction direction){
        return routine.dynamic(direction).finallyDo(() -> {
            io.runMotorSpeed(0);
        });
    }
    
}

