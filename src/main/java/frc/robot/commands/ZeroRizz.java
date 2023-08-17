package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Wrist;

public class ZeroRizz extends CommandBase {
    Wrist wrist;

    public ZeroRizz(Wrist wrist) {
        this.wrist = wrist;

        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        wrist.disableWrist();
        wrist.setPower(WristConstants.ZERO_SPEED);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        wrist.stop();
        wrist.enableWrist();
        wrist.setTargetAngle(WristConstants.MAX_ANGLE);
    }

    @Override
    public boolean isFinished() {
        return wrist.getLimitSwitch();
    }
}
