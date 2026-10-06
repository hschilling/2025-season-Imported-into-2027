package frc.robot.subsystems.coralWrist;

import java.util.function.Supplier;

import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import frc.robot.CommandFactory.Setpoint;
import frc.robot.subsystems.coralWrist.CoralWristIO.CoralWristIOStates;

public class CoralWristSubsystem extends SubsystemBase {
    private final CoralWristIO io;
    private CoralWristIOStates states = new CoralWristIOStates();

    public CoralWristSubsystem(CoralWristIO io) {
        this.io = io;
    }

    public void resetWrist() {
       io.resetRelativeToAbsolute();
    }

    public Command goToSetpointCommand(Supplier<Setpoint> setpoint) {
        return this.runOnce(() -> {
            io.setGoalAngle(setpoint.get());
        });
    }

    public Command setWristSpeed(double speed) {
        return this.run(() -> {
            io.setWristSpeed(speed);
        });
    }

    public boolean atGoal() {
        return io.atGoal();
    }

    // taking out motion profiling to see if code works
    // public Command setGoalStateTrapezoidCommand(Angle angle) {
    // return this.run(() -> {
    // io.setGoalStateTrapezoid(angle);
    // });
    // }

    @Override
    public void periodic() {
        io.updateStates(states);
        SmartDashboard.putNumber("coralWrist/wristVelocity", states.wristVelocity);
        SmartDashboard.putNumber("coralWrist/wristAppliedVoltage",
                states.wristAppliedVoltage);
        SmartDashboard.putNumber("coralWrist/wristCurrent", states.wristCurrent);
        SmartDashboard.putNumber("coralWrist/wristRelativeEncoderAngleInDegrees",
                states.wristRelativeEncoderAngle * 180 / Math.PI);
        SmartDashboard.putNumber("coralWrist/wristAbsoluteEncoderAngleInDegrees",
                states.wristAbsoluteAngle * 180 / Math.PI);
        SmartDashboard.putNumber("coralWrist/goalAngle", states.goalAngle);
    }
}