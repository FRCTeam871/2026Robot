package frc.robot.subsystems.swervedrive;

import org.littletonrobotics.junction.Logger;
import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;

//https://pdocs.kauailabs.com/navx-mxp/installation/omnimount/

public class SwerveDriveIOYaw implements SwerveDriveIO {
    private final AHRS gyro;

    public SwerveDriveIOYaw(final AHRS gyro) {
        this.gyro = gyro;
        SmartDashboard.putData("resetGyro", Commands.runOnce(() -> setCurrentAngle(0)).ignoringDisable(true)); // blue
        SmartDashboard.putData(
                "resetGyro180", Commands.runOnce(() -> setCurrentAngle(180.0)).ignoringDisable(true));  // red
        gyro.zeroYaw();
    }

    @Override
    public void updateInputs(final SwerveDriveIOInputs inputs) {
        inputs.gyroRotation = Rotation2d.fromDegrees(-gyro.getAngle());
        inputs.isCalibrating = gyro.isCalibrating();
        inputs.gyroRate = gyro.getRate();
        Logger.recordOutput("Drive/isCalibrating", inputs.isCalibrating);
    }

    @Override
    public void setCurrentAngle(double angle) {
        Logger.recordOutput("Drive/gyroZero", angle);
        gyro.zeroYaw();
        gyro.setAngleAdjustment(angle);
    }
}
