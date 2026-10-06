package frc.robot.subsystems.drive;

public class SwerveModulePlacebo implements SwerveModuleIO {
    @Override
    public void setDriveEncoderPosition(double position) {
    }

    @Override
    public double getTurnEncoderPosition() {
        return 0.0;
    }

    @Override
    public void setDesiredDriveSpeedMPS(double speed) {
    }

    @Override
    public double getDriveEncoderSpeedMPS() {
        return 0.0;
    }

    @Override
    public void setDesiredTurnAngle(double angle) {
    }

    @Override
    public double getDriveEncoderPosition() {
        return 0.0;
    }

    @Override
    public double getChassisAngularOffset() {
        return 0.0;
    }

    @Override
    public void updateStates(SwerveModuleIOStates states) {

    }
}