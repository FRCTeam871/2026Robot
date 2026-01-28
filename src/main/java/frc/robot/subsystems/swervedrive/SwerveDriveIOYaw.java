package frc.robot.subsystems.swervedrive;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;

public class SwerveDriveIOYaw implements SwerveDriveIO {
    private final AHRS gyro;

    public SwerveDriveIOYaw(final AHRS gyro) {
        this.gyro = gyro;
        SmartDashboard.putData("resetGyro", Commands.runOnce(gyro::zeroYaw).ignoringDisable(true));
        gyro.zeroYaw();
    }

    @Override
    public void updateInputs(final SwerveDriveIOInputs inputs) {
        inputs.gyroRotation = Rotation2d.fromDegrees(-gyro.getYaw());
        inputs.isCalibrating = gyro.isCalibrating();
        inputs.gyroRate = gyro.getRate();
    }

    @Override
    public void setCurrentAngle(double angle) {
        // TODO: make it work
    }
}
