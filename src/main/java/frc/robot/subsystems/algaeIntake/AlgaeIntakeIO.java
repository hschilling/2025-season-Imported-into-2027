package frc.robot.subsystems.algaeIntake;

import org.wpilib.units.measure.AngularVelocity;

public interface AlgaeIntakeIO {

    static class AlgaeIntakeIOStates {
        public double intakeVelocity = 0.0;
        public double goalVelocity = 0.0;
        public double leftCurrent = 0.0;
        public double leftAppliedVoltage = 0.0;

    }

    public void setRollerSpeed(AngularVelocity speed);
    public void intake();
    public void outtake();
    public void passiveIntake();
    public void updateStates(AlgaeIntakeIOStates states);

    public boolean isStalling();
}
