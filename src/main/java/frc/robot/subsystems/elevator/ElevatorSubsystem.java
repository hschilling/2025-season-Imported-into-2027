package frc.robot.subsystems.elevator;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;

import java.util.function.Supplier;

import org.wpilib.units.measure.Distance;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import frc.robot.CommandFactory.GameMode;
import frc.robot.CommandFactory.Setpoint;
import frc.robot.Constants.SetpointConstants;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;;

public class ElevatorSubsystem extends SubsystemBase {

    private final ElevatorIO elevatorIO;
    private final ElevatorIOInputs states = new ElevatorIOInputs();
    public boolean profiledPIDEnabled = false;
    private double goalSetpoint;
    private final Distance HEIGHT_TOLERANCE = Inches.of(0.5);
    private double JOYSTICK_INPUT_TO_CHANGE_IN_POSITION_CONVERSION_FACTOR = 0.002;

    public static final class ElevatorConstants {
    }

    public ElevatorSubsystem(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
        elevatorIO.setIntermediateSetpoint(Meters.of(0), 0);
    }

    public Command goToGoalSetpointCmd(Supplier<Setpoint> setpoint, Supplier<GameMode> gameMode) {
        return this.runOnce(() -> {
            Distance elevatorSetpoint = getElevatorSetpoint(setpoint, gameMode);
            elevatorIO.setGoalPosition(elevatorSetpoint);
            profiledPIDEnabled = true;
            goalSetpoint = elevatorSetpoint.in(Meters);
        });
    };

    public Distance getElevatorSetpoint(Supplier<Setpoint> setpoint, Supplier<GameMode> gameMode) {
        Distance elevatorSetpoint = SetpointConstants.ELEVATOR_ALGAE_TURTLE_HEIGHT;
        if (gameMode.get() == GameMode.CORAL) {
            if (setpoint.get() == Setpoint.TURTLE) {
                elevatorSetpoint = SetpointConstants.ELEVATOR_CORAL_TURTLE_HEIGHT;
            } else if (setpoint.get() == Setpoint.L_ONE) {
                elevatorSetpoint = SetpointConstants.L_ONE_CORAL_HEIGHT;
            } else if (setpoint.get() == Setpoint.L_TWO) {
                elevatorSetpoint = SetpointConstants.L_TWO_CORAL_HEIGHT;
            } else if (setpoint.get() == Setpoint.L_THREE) {
                elevatorSetpoint = SetpointConstants.L_THREE_CORAL_HEIGHT;
            } else if (setpoint.get() == Setpoint.L_FOUR) {
                elevatorSetpoint = SetpointConstants.L_FOUR_CORAL_HEIGHT;
            } else if (setpoint.get() == Setpoint.AUTON) {
                elevatorSetpoint = SetpointConstants.AUTON_HEIGHT;
            }
        } else if (gameMode.get() == GameMode.ALGAE) {
            if (setpoint.get() == Setpoint.TURTLE) {
                elevatorSetpoint = SetpointConstants.ELEVATOR_ALGAE_TURTLE_HEIGHT;
            } else if (setpoint.get() == Setpoint.L_ONE) {
                elevatorSetpoint = SetpointConstants.L_ONE_ALGAE_HEIGHT;
            } else if (setpoint.get() == Setpoint.L_TWO) {
                elevatorSetpoint = SetpointConstants.L_TWO_ALGAE_HEIGHT;
            } else if (setpoint.get() == Setpoint.L_THREE) {
                elevatorSetpoint = SetpointConstants.L_THREE_ALGAE_HEIGHT;
            }
        }
        return elevatorSetpoint;
    }

    public boolean atGoal() {
        return (Math.abs(goalSetpoint - elevatorIO.getEncoderPosition()) <= HEIGHT_TOLERANCE.in(Meters));
    }

    public Command atGoalCommand() {
        return Commands.waitUntil(() -> atGoal());
    }
    
    public Command incrementGoalPosition(Distance changeInGoalPosition)
    {
        return this.run(()-> {
            profiledPIDEnabled = true;
            elevatorIO.incrementGoalPosition(changeInGoalPosition);
        });
    }

    public double getCurrentPosition() {
        return elevatorIO.getEncoderPosition();
    }

    public Command setSpeedManualControl(double speed) {
        return this.run(() -> elevatorIO.setSpeedManualControl(speed));
    }

    public boolean isElevatorHeightAboveSpeedLimitingThreshold()
    {
        return elevatorIO.isElevatorHeightAboveSpeedLimitingThreshold();
    }

    
    @Override
    public void periodic() {
        if (!profiledPIDEnabled) {
            elevatorIO.resetSetpointsToCurrentPosition();
        }
        elevatorIO.calculateNextIntermediateSetpoint();

        elevatorIO.updateStates(states);
        SmartDashboard.putNumber("Elevator/position", states.position);
        SmartDashboard.putNumber("Elevator/velocity", states.velocity);
        SmartDashboard.putNumber("Elevator/appliedVoltageRight", states.appliedVoltageRight);
        SmartDashboard.putNumber("Elevator/appliedVoltageLeft", states.appliedVoltageLeft);
        SmartDashboard.putNumber("Elevator/goalStatePosition", states.goalPosition);
        SmartDashboard.putNumber("Elevator/output current", states.current);
        SmartDashboard.putNumber("Elevator/intermediate setpoint position", states.intermediateSetpointPosition);
        SmartDashboard.putBoolean("Elevator/profiled PID enabled", profiledPIDEnabled);
        SmartDashboard.putBoolean("Elevator/elevator height above speed limiting threshold", isElevatorHeightAboveSpeedLimitingThreshold());
    }
}
