package frc.robot;

import frc.robot.subsystems.Collector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.Collect;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.ZeroRizz;
import frc.robot.commands.Tests.Collector.CollectorSystemTest;
import frc.robot.commands.Tests.Drive.DriveTrainSystemTest;
import frc.robot.commands.HoldPower;
import frc.robot.commands.setPoints;
import frc.robot.subsystems.Drivetrain;
import frc.thunder.LightningContainer;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.WristAngles.wristStates;
import frc.thunder.auto.Autonomous;
import frc.thunder.auto.AutonomousCommandFactory;
import frc.thunder.pathplanner.com.pathplanner.lib.PathConstraints;
import frc.thunder.testing.SystemTest;

public class RobotContainer extends LightningContainer {

        private static final Limelight limelight = new Limelight(LimelightConstants.FRONT_NAME,
                        LimelightConstants.FRONT_POSE);

        // Creating our main subsystems
        private static final Drivetrain drivetrain = new Drivetrain(limelight);
        private static final Wrist wrist = new Wrist();
        private static final Collector collector = new Collector();

        // Creates our controllers and deadzones
        private static final XboxController driver = new XboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
        private static final XboxController copilot = new XboxController(ControllerConstants.COPILOT_CONTROLLER_PORT);
        private Debouncer deb = new Debouncer(1, DebounceType.kFalling);

        // creates Autonomous Command
        private static final AutonomousCommandFactory autoFactory = new AutonomousCommandFactory(drivetrain::getPose,
                        drivetrain::resetOdometry, drivetrain.getDriveKinematics(),
                        AutonomousConstants.DRIVE_PID_CONSTANTS,
                        AutonomousConstants.THETA_PID_CONSTANTS, AutonomousConstants.POSE_PID_CONSTANTS,
                        drivetrain::setStates, drivetrain::resetNeoAngle, drivetrain);

        @Override
        protected void configureButtonBindings() {

                new Trigger(DriverStation::isTeleopEnabled).onTrue(new ZeroRizz(wrist));

                /* driver Controls */
                // RESETS
                new Trigger(() -> (driver.getBackButton() && driver.getStartButton()))
                                .onTrue(new InstantCommand(drivetrain::zeroHeading, drivetrain)); // Resets Forward to
                                                                                                  // be the direction
                                                                                                  // the robot is facing

                new Trigger(driver::getAButton).onTrue(new InstantCommand(drivetrain::resetNeoAngle)); // REsyncs the NEOs relative encoder to the absolute encoders on the swerve modules

                // Flips modules 180 degrees to fix a module that is facing the wrong way on
                // startup
        
                new Trigger(() -> driver.getPOV() == 0).onTrue(new InstantCommand(drivetrain::flipFR, drivetrain));
                new Trigger(() -> driver.getPOV() == 180).onTrue(new InstantCommand(drivetrain::flipBL, drivetrain));
                new Trigger(() -> driver.getPOV() == 90).onTrue(new InstantCommand(drivetrain::flipBR, drivetrain));
                new Trigger(() -> driver.getPOV() == 270).onTrue(new InstantCommand(drivetrain::flipFL, drivetrain));

                // SET DRIVE PODS TO 45
                new Trigger(driver::getXButton).whileTrue(new RunCommand(() -> drivetrain.stop(), drivetrain)); // Locks wheels to prevent sliding especially once balanced

                // AUTOBALANCE
                // new Trigger(driver::getBButton).whileTrue(new AutoBalance(drivetrain)); //
                // FOR TESTING

                // LINE UP
                new Trigger(driver::getBButton).whileTrue(new AutoAlign(drivetrain, limelight));

                // AUTO deploy on collect TODO test
                // Retract on stoping collect TODO test

                // Auto shoot when button

                /* copilot controls */

                // SETPOINTS
                new Trigger(copilot::getAButton).whileTrue(new setPoints(wrist, () -> wristStates.Ground));
                new Trigger(copilot::getBButton).whileTrue(new setPoints(wrist, () -> wristStates.Stow));
                new Trigger(copilot::getXButton).whileTrue(new setPoints(wrist, () -> wristStates.MidCube));
                new Trigger(copilot::getYButton).whileTrue(new setPoints(wrist, () -> wristStates.HighCube));

                // new Trigger(copilot::getRightBumper).whileTrue(new InstantCommand(() ->
                // collector.setPercentPower(copilot.getRightTriggerAxis()))).onFalse(new
                // InstantCommand(() -> collector.setPercentPower(0d)));

                new Trigger(copilot::getRightBumper).whileTrue(new ZeroRizz(wrist));

                // FLICK TODO FIX
                // new Trigger(() -> -copilot.getLeftY() < -0.25).onTrue(new InstantCommand(()
                // -> wrist.setGoalState(wristStates.Ground)));

                // DISABLE LIFT
                new Trigger(() -> copilot.getStartButton() && copilot.getBackButton())
                                .onTrue(new InstantCommand(wrist::disableWrist));

                // BIAS
                new Trigger(() -> copilot.getPOV() == 0).onTrue(new InstantCommand(() -> wrist.adjustWrist(10d)));
                new Trigger(() -> copilot.getPOV() == 180).onTrue(new InstantCommand(() -> wrist.adjustWrist(-10d)));
        }

        // Creates the autonomous commands
        @Override
        protected void configureAutonomousCommands() {
                // EXAMPLE
                // autoFactory.makeTrajectory("NAME", Maps.getPathMap(drivetrain, servoturn,
                // lift, collector, leds, arm),
                // new PathConstraints(AutonomousConstants.MAX_VELOCITY,
                // AutonomousConstants.MAX_ACCELERATION));

                // A PATHS
                autoFactory.makeTrajectory("A1[3]-M-LOW", Maps.getPathMap(drivetrain, collector, limelight, wrist),
                        new PathConstraints(3.5, 3));

                // B PATHS
                autoFactory.makeTrajectory("B2[1]-C-LOW", Maps.getPathMap(drivetrain, collector, limelight, wrist),
                        new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
                autoFactory.makeTrajectory("B2[1]-M-C-LOW", Maps.getPathMap(drivetrain, collector, limelight, wrist),
                        new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
                autoFactory.makeTrajectory("B2[1]-M-C-LOW-SLAM", Maps.getPathMap(drivetrain, collector, limelight, wrist),
                        new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
                // C PATHS
                autoFactory.makeTrajectory("C3[3]-M-LOW", Maps.getPathMap(drivetrain, collector, limelight, wrist),
                        new PathConstraints(3, 2.5));

                // ANYWHERE
                Autonomous.register("ruh roh flick auto", new InstantCommand()); // Emergency Auton that doesn't drive
        }

        @Override
        protected void configureDefaultCommands() {

                /*
                 * Set up the default command for the drivetrain. The controls are for
                 * field-oriented
                 * driving: Left stick Y axis -> forward and backwards movement Left stick X
                 * axis -> left
                 * and right movement Right stick X axis -> rotation
                 */
                drivetrain.setDefaultCommand(new SwerveDrive(drivetrain,
                                () -> MathUtil.applyDeadband(driver.getLeftX(), ControllerConstants.DEADBAND),
                                () -> MathUtil.applyDeadband(driver.getLeftY(), ControllerConstants.DEADBAND),
                                () -> MathUtil.applyDeadband(-driver.getRightX(), ControllerConstants.DEADBAND),
                                () -> driver.getRightTriggerAxis() > 0.25,
                                () -> driver.getLeftTriggerAxis() > 0.25));

                collector.setDefaultCommand(new HoldPower(collector,
                                () -> MathUtil.applyDeadband(copilot.getRightTriggerAxis(),
                                ControllerConstants.DEADBAND) - MathUtil.applyDeadband(copilot.getLeftTriggerAxis(),
                                ControllerConstants.DEADBAND), driver, copilot));

                // wrist.setDefaultCommand(new ZeroWrist(wrist)); //TOOD: test```

                // collector.setDefaultCommand(new Collect(collector, () ->
                // MathUtil.applyDeadband(copilot.getRightTriggerAxis(),
                // ControllerConstants.DEADBAND) -
                // MathUtil.applyDeadband(copilot.getLeftTriggerAxis(),
                // ControllerConstants.DEADBAND)));

        }

        @Override
        protected void configureSystemTests() {
                SystemTest.registerTest("fl drive test",
                                new DriveTrainSystemTest(drivetrain, drivetrain.getFrontLeftModule(), 0.25));
                SystemTest.registerTest("fr drive test",
                                new DriveTrainSystemTest(drivetrain, drivetrain.getFrontRightModule(), 0.25));
                SystemTest.registerTest("bl drive test",
                                new DriveTrainSystemTest(drivetrain, drivetrain.getBackLeftModule(), 0.25));
                SystemTest.registerTest("br drive test",
                                new DriveTrainSystemTest(drivetrain, drivetrain.getBackRightModule(), 0.25));

                SystemTest.registerTest("Collector test", new CollectorSystemTest(collector, 1d, 5500));
        }

        @Override
        protected void releaseDefaultCommands() {
        }

        @Override
        protected void initializeDashboardCommands() {
        }

        @Override
        protected void configureFaultCodes() {
        }

        @Override
        protected void configureFaultMonitors() {
        }

        @Override
        protected AutonomousCommandFactory getCommandFactory() {
                return autoFactory;
        }

        /*
         * Hello this is your favorite programing monster
         * 
         * I haunt your code and your dreams, you wont sleep at night while thinking
         * about the code
         * issues you've been having.
         * 
         * have fun
         * 
         * bu bye >:)
         */

}
