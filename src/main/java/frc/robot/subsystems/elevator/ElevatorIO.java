package frc.robot.subsystems.elevator;


import org.wpilib.units.measure.Distance;

public interface ElevatorIO {

    static class ElevatorIOInputs {
        public double position = 0.0;
        public double velocity = 0.0;
        public double appliedVoltageRight = 0.0;
        public double appliedVoltageLeft = 0.0;
        public double goalPosition = 0.0;
        public double intermediateSetpointPosition = 0.0;
        public double current = 0.0;
    }

    public void resetSetpointsToCurrentPosition();

    public void incrementGoalPosition(Distance newGoalPosition);

    public void setGoalPosition(Distance position);

    public void calculateNextIntermediateSetpoint();

    public void setIntermediateSetpoint(Distance position, double velocity);

    public double getEncoderVelocity();

    public double getEncoderPosition();

    public void setSpeedManualControl(double speed);

    public boolean isElevatorHeightAboveSpeedLimitingThreshold();

    public void updateStates(ElevatorIOInputs states);

}