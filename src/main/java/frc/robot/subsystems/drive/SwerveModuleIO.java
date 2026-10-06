package frc.robot.subsystems.drive;

public interface SwerveModuleIO {

    public static class SwerveModuleIOStates {

        public double driveVoltage = 0.0;
        public double turnVoltage = 0.0;
        public double driveVelocity = 0.0;
        public double driveDesiredVelocity = 0.0;
        public double turnAngle = 0.0;
        public double desiredAngle = 0.0;
        public double driveCurrent = 0.0;
        public double turnCurrent = 0.0;
        public double driveEncoderPos = 0.0;
        public double turningEncoderPos = 0.0;

    }

    public void setDriveEncoderPosition(double position);

    public void setDesiredDriveSpeedMPS(double speed);

    public double getDriveEncoderSpeedMPS();

    public void setDesiredTurnAngle(double angle);

    public double getTurnEncoderPosition();

    public double getDriveEncoderPosition();

    public void updateStates(SwerveModuleIOStates states);

    public double getChassisAngularOffset();

}