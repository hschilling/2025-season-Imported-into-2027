package frc.robot.subsystems.coralIntake;

import frc.robot.CommandFactory.Setpoint;

public interface CoralIntakeIO {
    static class CoralIntakeIOStates {
        public double velocity = 0.0;
        public double goalVelocity = 0.0;
        public double leftCurrent = 0.0;
        public double rightCurrent = 0.0;
        public double leftAppliedVoltage = 0.0;
        public double rightAppliedVoltage = 0.0;
    }

    public void intake();

    public void setOuttakeSpeed(Setpoint setpoint);

    public void setZero();

    public boolean isStalling();

    public void passiveIntake();

    public void passiveIntakeIgnoringStall();

    public void updateStates(CoralIntakeIOStates states);
}