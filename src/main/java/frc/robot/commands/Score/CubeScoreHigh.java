package frc.robot.commands.Score;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Wrist;

public class CubeScoreHigh extends CommandBase {
  
  Collector collector;
  Wrist wrist;

  public CubeScoreHigh(Collector collector, Wrist wrist) {
    this.collector = collector;
    this.wrist = wrist;

    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
