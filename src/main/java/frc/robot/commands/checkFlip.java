// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class checkFlip extends CommandBase {
	Drivetrain drivetrain;
	XboxController driver;
	Debouncer deb = new Debouncer(1, DebounceType.kFalling);

	public checkFlip(Drivetrain drivetrain, XboxController driver) {
		this.driver = driver;
		this.drivetrain = drivetrain;
	}
	

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		switch (driver.getPOV()) {
			case 0:
				if (deb.calculate(driver.getPOV() == 0)) {
					drivetrain.flipFR();
				}
				break;
			case 90:
				if (deb.calculate(driver.getPOV() == 90)) {
					drivetrain.flipBR();
				}
				break;
			case 180:
				if (deb.calculate(driver.getPOV() == 180)) {
					drivetrain.flipBL();
				}
				break;
			case 270:
				if (deb.calculate(driver.getPOV() == 270)) {
					drivetrain.flipFL();
				}
				break;
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
