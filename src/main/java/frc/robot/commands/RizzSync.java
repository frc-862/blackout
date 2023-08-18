package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Wrist;

public class RizzSync extends CommandBase {
    Wrist wrist;
    double lastTime = 0;
    boolean doSync = true;
    public RizzSync(Wrist wrist) {
        this.wrist = wrist;

        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        wrist.setAngle(wrist.getAbsoluteAngle());
        lastTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if(lastTime - Timer.getFPGATimestamp() > 5 && Math.abs(wrist.getAngle().getDegrees() - wrist.getAbsoluteAngle()) < 7 && doSync) {
            wrist.setAngle(wrist.getAbsoluteAngle());
            lastTime = Timer.getFPGATimestamp();
        }

        if(wrist.getLimitSwitch() && Math.abs(WristConstants.MAX_ANGLE - wrist.getAbsoluteAngle()) > 7) {
            doSync = false;
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
