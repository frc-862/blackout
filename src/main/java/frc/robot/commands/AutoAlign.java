package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class AutoAlign extends CommandBase {
	
	private Drivetrain drivetrain;
	private Limelight limelight;

	PIDController RController = new PIDController(AutoAlignConstants.RkP, AutoAlignConstants.RkI, AutoAlignConstants.RkD);
	PIDController YController = new PIDController(AutoAlignConstants.YkP, AutoAlignConstants.YkI, AutoAlignConstants.YkD);

	private double YOutput; 
	private double ROutput;

	public AutoAlign(Drivetrain drivetrain, Limelight limelight) {
		this.drivetrain = drivetrain;
		this.limelight = limelight;

		addRequirements(drivetrain);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		YController.setTolerance(AutoAlignConstants.TOLERANCE);
		
		YController.setSetpoint(0d);

	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if(limelight.hasVision() && limelight.getPipelineNum() == 1) {
			YOutput = YController.calculate(limelight.getHorizontalOffset());
		} else {
			YOutput = 0d;
		}

		drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(drivetrain.percentOutputToMetersPerSecond(0d), 
        drivetrain.percentOutputToMetersPerSecond(YOutput),
        drivetrain.percentOutputToRadiansPerSecond(ROutput), 
        drivetrain.getYaw2d()));
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		drivetrain.stop();
		limelight.setPipelineNum(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
