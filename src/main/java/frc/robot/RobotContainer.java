package frc.robot;

import frc.robot.subsystems.Collector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LimelightFront;
import frc.robot.subsystems.Music;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.ServoTurn;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.GamePiece;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.Collect;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.Tests.Collector.CollectorTest;
import frc.robot.commands.Tests.Drive.DriveTrainSystemTest;
import frc.robot.commands.HoldPower;
import frc.robot.commands.SafeToScoreLED;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDs;
import frc.thunder.LightningContainer;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.WristAngles.wristStates;
import frc.thunder.auto.Autonomous;
import frc.thunder.auto.AutonomousCommandFactory;
import frc.thunder.testing.SystemTest;

public class RobotContainer extends LightningContainer {

    private static final LimelightFront frontLimelight = new LimelightFront(LimelightConstants.FRONT_NAME, LimelightConstants.FRONT_POSE);
    // private static final LimelightBack backLimelight = new LimelightBack(LimelightConstants.BACK_NAME, LimelightConstants.BACK_POSE);

    // Creating our main subsystems
    private static final Drivetrain drivetrain = new Drivetrain(frontLimelight);
    // private static final Wrist wrist = new Wrist();
    // private static final Collector collector = new Collector();
    // private static final LEDs leds = new LEDs(collector);
    private static final Music music = new Music();

    // Creates our controllers and deadzones
    private static final XboxController driver = new XboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
    private static final XboxController copilot = new XboxController(ControllerConstants.COPILOT_CONTROLLER_PORT);
    private static final Joystick buttonPad = new Joystick(ControllerConstants.BUTTON_PAD_CONTROLLER_PORT);

    // creates Autonomous Command
    private static final AutonomousCommandFactory autoFactory = new AutonomousCommandFactory(drivetrain::getPose, drivetrain::resetOdometry, drivetrain.getDriveKinematics(),
            AutonomousConstants.DRIVE_PID_CONSTANTS, AutonomousConstants.THETA_PID_CONSTANTS, AutonomousConstants.POSE_PID_CONSTANTS, drivetrain::setStates, drivetrain::resetNeoAngle, drivetrain);

    @Override
    protected void configureButtonBindings() {
        /* driver Controls */
        // RESETS
        new Trigger(() -> (driver.getBackButton() && driver.getStartButton())).onTrue(new InstantCommand(drivetrain::zeroHeading, drivetrain)); // Resets Forward to be the direction the robot is facing

        new Trigger(driver::getAButton).onTrue(new InstantCommand(drivetrain::resetNeoAngle)); // REsyncs the NEOs relative encoder to the absolute encoders on the swerve modules

        // Flips modules 180 degrees to fix a module that is facing the wrong way on startup
        new Trigger(() -> driver.getPOV() == 0).onTrue(new InstantCommand(drivetrain::flipFR, drivetrain));
        new Trigger(() -> driver.getPOV() == 180).onTrue(new InstantCommand(drivetrain::flipBL, drivetrain));
        new Trigger(() -> driver.getPOV() == 90).onTrue(new InstantCommand(drivetrain::flipBR, drivetrain));
        new Trigger(() -> driver.getPOV() == 270).onTrue(new InstantCommand(drivetrain::flipFL, drivetrain));

        // GAME PIECE SET
        // new Trigger(driver::getRightBumper).onTrue(new InstantCommand(() -> collector.setGamePiece(GamePiece.CONE)));
        // new Trigger(driver::getLeftBumper).onTrue(new InstantCommand(() -> collector.setGamePiece(GamePiece.CUBE)));

        // SET DRIVE PODS TO 45
        new Trigger(driver::getXButton).whileTrue(new RunCommand(() -> drivetrain.stop(), drivetrain)); // Locks wheels to prevent sliding especially once balanced
        
        //AUTOBALANCE
        new Trigger(driver::getBButton).whileTrue(new AutoBalance(drivetrain)); // FOR TESTING



        /* copilot controls */
        
        //SETPOINTS
        // new Trigger(copilot::getAButton).onTrue(new InstantCommand(() -> wrist.setGoalState(wristStates.Ground), wrist));
        // new Trigger(copilot::getBButton).onTrue(new InstantCommand(() -> wrist.setGoalState(wristStates.Stow), wrist)); 
        // new Trigger(copilot::getXButton).onTrue(new InstantCommand(() -> wrist.setGoalState(wristStates.MidCube), wrist));
        // new Trigger(copilot::getYButton).onTrue(new InstantCommand(() -> wrist.setGoalState(wristStates.HighCube), wrist));

        //SHOOT
        // new Trigger(() -> copilot.getRightBumper() && copilot.getLeftBumper()).onTrue(new Shoot(collector, wrist));

        //FLICK TODO FIX
        // new Trigger(() -> -copilot.getLeftY() > 0.25).onTrue(new InstantCommand(() -> wrist.setTargetAngle(150))); 
        // new Trigger(() -> -copilot.getLeftY() < -0.25).onTrue(new InstantCommand(() -> wrist.setTargetAngle(10)));

        //BREAK
        // new Trigger(copilot::getRightStickButton).onTrue(new InstantCommand(lift::breakLift)); // Breaks out of current goal state and sets itself to onTarget so it can go to a new State

        // DISABLE LIFT
        // new Trigger(() -> copilot.getStartButton() && copilot.getBackButton())
        //     .onTrue(new InstantCommand(wrist::disableWrist));

        /* BUTTON Pad */
        /* 
         * Button 1 Play / Pause
         * Button 2 Next Track
         * Button 3 Previous Track
         */

        new Trigger(() -> buttonPad.getRawButton(0)).onTrue(new InstantCommand(() -> music.toggle()));
        new Trigger(() -> buttonPad.getRawButton(1)).onTrue(new InstantCommand(() -> music.nextTrack()));
        new Trigger(() -> buttonPad.getRawButton(2)).onTrue(new InstantCommand(() -> music.previousTrack()));

    }

    // Creates the autonomous commands
    @Override
    protected void configureAutonomousCommands() {
        //EXAMPLE
        // autoFactory.makeTrajectory("NAME", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds, arm),
        //         new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));

        
        //ANYWHERE
        Autonomous.register("ruh roh flick auto", new InstantCommand()); // Emergency Auton that doesn't drive
    }

    @Override
    protected void configureDefaultCommands() {

        /*
         * Set up the default command for the drivetrain. The controls are for field-oriented driving: Left
         * stick Y axis -> forward and backwards movement Left stick X axis -> left and right movement Right
         * stick X axis -> rotation
         */
        drivetrain.setDefaultCommand(new SwerveDrive(drivetrain, () -> MathUtil.applyDeadband(driver.getLeftX(), ControllerConstants.DEADBAND),
                () -> MathUtil.applyDeadband(driver.getLeftY(), ControllerConstants.DEADBAND), () -> MathUtil.applyDeadband(-driver.getRightX(), ControllerConstants.DEADBAND),
                () -> driver.getRightTriggerAxis() > 0.25, () -> driver.getLeftTriggerAxis() > 0.25));

        // leds.setDefaultCommand(new SafeToScoreLED(leds, drivetrain, collector)); // Changes LED color to RED when the arm will not hit when deploying 

        // collector.setDefaultCommand(new HoldPower(collector, () -> MathUtil.applyDeadband(copilot.getRightTriggerAxis(), ControllerConstants.DEADBAND) 
        // - MathUtil.applyDeadband(copilot.getLeftTriggerAxis(), ControllerConstants.DEADBAND), driver, copilot));

        // collector.setDefaultCommand(new Collect(collector, () -> MathUtil.applyDeadband(copilot.getRightTriggerAxis(), ControllerConstants.DEADBAND) - MathUtil.applyDeadband(copilot.getLeftTriggerAxis(), ControllerConstants.DEADBAND)));

    }

    @Override
    protected void configureSystemTests() {
        SystemTest.registerTest("fl drive test" , new DriveTrainSystemTest(drivetrain, drivetrain.getFrontLeftModule(), 0.25));
        SystemTest.registerTest("fr drive test" , new DriveTrainSystemTest(drivetrain, drivetrain.getFrontRightModule(), 0.25));
        SystemTest.registerTest("bl drive test" , new DriveTrainSystemTest(drivetrain, drivetrain.getBackLeftModule(), 0.25));
        SystemTest.registerTest("br drive test" , new DriveTrainSystemTest(drivetrain, drivetrain.getBackRightModule(), 0.25));

        // SystemTest.registerTest("Collector test", new CollectorTest(collector, 1d));
    }

    @Override
    protected void releaseDefaultCommands() {}

    @Override
    protected void initializeDashboardCommands() {}

    @Override
    protected void configureFaultCodes() {}

    @Override
    protected void configureFaultMonitors() {}

    @Override
    protected AutonomousCommandFactory getCommandFactory() {
        return autoFactory;
    }


    /* Hello this is your favorite programing monster
     * 
     * I haunt your code and your dreams, you wont sleep at night while thinking about 
     * the code issues you've been having. 
     * 
     * have fun
     * 
     * bu bye >:)
     */




}
