package frc.robot.subsystems.coralWrist;

import frc.robot.CommandFactory.Setpoint;

public class CoralWristPlacebo implements CoralWristIO {

    @Override
    public void resetRelativeToAbsolute() {
    }

    @Override
    public void setGoalAngle(Setpoint setpoint) {
    }

    @Override
    public void setWristSpeed(double speed) {
    }

    @Override
    public void updateStates(CoralWristIOStates states) {
    }

    @Override
    public boolean atGoal() {
        return true;
    }

    @Override
    public void periodic() {
    }

    // @Override
    // public void setGoalStateTrapezoid(Angle angle) {
    // }
}
