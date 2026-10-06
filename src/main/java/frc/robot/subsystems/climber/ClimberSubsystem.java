package frc.robot.subsystems.climber;

import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
// import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;

public class ClimberSubsystem extends SubsystemBase {
    
    private ClimberIO climberIO;
    private final ClimberIOInputs inputs = new ClimberIOInputs();
    
    public ClimberSubsystem(ClimberIO climberIO)
    {
            this.climberIO = climberIO; 
            //add a line setting the climber to its initial position 
    }

    public Command setSpeed(LinearVelocity speed)
    {
        return this.run(() -> climberIO.setSpeed(speed));
    }

    @Override
    public void periodic() {
        climberIO.updateStates(inputs);
        SmartDashboard.putNumber("climber/climberAngle", inputs.climberAngle);
        SmartDashboard.putNumber("climber/climberVelocity", inputs.climberVelocity);
        //TODO: we can likely take this out - climber is manual only!
        SmartDashboard.putNumber("climber/goalAngle", inputs.climberGoalAngle);
        SmartDashboard.putNumber("climber/servoAngle", inputs.servoAngle);
        //TODO: is this one really necessary?
        SmartDashboard.putNumber("climber/servoVelocity", inputs.servoVelocity);
        SmartDashboard.putNumber("climber/servoGoalAngle", inputs.servoGoalAngle);
    }
}
