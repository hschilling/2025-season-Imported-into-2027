package frc.robot.subsystems.algaeIntake;

import static org.wpilib.units.Units.RPM;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.networktables.NetworkTableEntry;
import org.wpilib.smartdashboard.SmartDashboard;

import frc.robot.CommandFactory;
import frc.robot.Constants;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeIO.AlgaeIntakeIOStates;

public class AlgaeIntakeSubsystem extends SubsystemBase {

    private final AlgaeIntakeIOStates states = new AlgaeIntakeIOStates();
    private AlgaeIntakeIO io;
    public boolean hasAlgae = false;
    private final NetworkTableEntry algaeEntry = CommandFactory.scoringStateTables.getEntry("hasAlgae");

    public AlgaeIntakeSubsystem(AlgaeIntakeIO io) {
        this.io = io;
    }

    public Command intake() {
        return this.run(() -> io.intake()).withName("run algae intake");
    }

    public Command outtake() {
        return this.run(() -> {
            io.outtake();
            if (hasAlgae) {
                setAlgaeEntry(false);
            }
        });
    }

    public Command defaultBehavior() {
        // return Commands.either(this.run(() -> io.passiveIntake()),
        //         this.run(() -> io.setRollerSpeed(RPM.of(0))).withName("Default Algae Intake"),
        //         () -> this.hasAlgae);
        return this.run(() -> {
            if (hasAlgae) {
                io.passiveIntake();
            } else {
                io.setRollerSpeed(RPM.of(0));
            }
        });
    }

    public Command intakeToStall() {
        return this.run(
                () -> {
                    if (io.isStalling() || hasAlgae) {
                        io.setRollerSpeed(RPM.of(0));
                        setAlgaeEntry(true);
                    } else {
                        io.setRollerSpeed(Constants.SpeedConstants.ALGAE_INTAKE_SPEED);

                    }
                });
    }

    public void setAlgaeEntry(Boolean algaeState) {
        algaeEntry.setBoolean(algaeState);
        hasAlgae = algaeState;
    }

    @Override
    public void periodic() {
        hasAlgae = algaeEntry.getBoolean(hasAlgae);
        io.updateStates(states);
        SmartDashboard.putNumber("algaeIntake/intakeVelocity", states.intakeVelocity);
        SmartDashboard.putNumber("algaeIntake/goalVelocity", states.goalVelocity);
        SmartDashboard.putNumber("algaeIntake/leftCurrent", states.leftCurrent);
        SmartDashboard.putNumber("algaeIntake/leftAppliedVoltage", states.leftAppliedVoltage);
        SmartDashboard.putBoolean("algaeIntake/hasAlgae", hasAlgae);
    }
}