package frc.robot.subsystems.coralWrist;

import frc.robot.CommandFactory.Setpoint;

public interface CoralWristIO {
    static class CoralWristIOStates {
        public double wristVelocity = 0.0;
        public double wristAppliedVoltage = 0.0;
        public double wristCurrent = 0.0;
        public double wristRelativeEncoderAngle = 0.0;
        public double wristAbsoluteAngle = 0.0;
        // public double trapezoidProfileGoalAngle = 0.0;
        public double goalAngle = 0.0;
    }

    public void resetRelativeToAbsolute();

    public void setGoalAngle(Setpoint setpoint);

    // taking out motion profiling to see if code works
    // public void setGoalStateTrapezoid(Angle angle);

    public void setWristSpeed(double speed);

    public void updateStates(CoralWristIOStates states);

    public boolean atGoal();

    public void periodic();
}
