package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.WristAngles.wristStates;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.Collect;
import frc.robot.commands.setPoints;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Wrist;
import frc.thunder.vision.VisionBase;

/**
 * Class for creating all auton HasMaps
 */
public class Maps {

    /**
     * The general Hash map for all paths. Has calls needed for the paths to run.
     * 
     * @param drivetrain
     * @param servoturn
     * @param collector
     * @param wrist
     * @return 
     */
    public static HashMap<String, Command> getPathMap(Drivetrain drivetrain, Collector collector, Limelight frontLimelight, Wrist wrist){
        HashMap<String, Command> eventMap = new HashMap<>();
        // SET Positions TODO Change to Score command?
        eventMap.put("Ground-Collect", new InstantCommand(() -> wrist.setGoalState(wristStates.Ground)));
        eventMap.put("Mid-Score", new InstantCommand(() -> wrist.setGoalState(wristStates.MidCube)));
        eventMap.put("High-Score", new InstantCommand(() -> wrist.setGoalState(wristStates.HighCube)));
        eventMap.put("Stow", new InstantCommand(() -> wrist.setGoalState(wristStates.Stow)));

        // COLLECTOR actions
        eventMap.put("Collect", new InstantCommand(() -> collector.setPercentPower(1d), collector)); // Starts the collector intake
        eventMap.put("Stop-Collector", new InstantCommand(() -> collector.stop(), collector)); // Stops the collector
        
        eventMap.put("Hold-Power", new InstantCommand(() -> collector.setPercentPower(CollectorConstants.HOLD_POWER))); // Power to hold cube once piece is collected
        
        eventMap.put("Score", new InstantCommand(() -> collector.setPercentPower(-1d))); // Full power Score to spit out cube
        eventMap.put("Score-Slow", new InstantCommand(() -> collector.setPercentPower(-.50))); // Lower power for no roll out
        // eventMap.put("Score-Smart", new Score(collector, wrist));
        // OTHER
        eventMap.put("Auto-Balance", new AutoBalance(drivetrain)); // Call to AutoBalance command requires drivetrain, must be at the end of the path
        
        // VISION
        eventMap.put("Turn-On-Vision", new InstantCommand(() -> VisionBase.enableVision())); // Turns on vision
        eventMap.put("Turn-Off-Vision", new InstantCommand(() -> VisionBase.disableVision())); // Turns off vision
        eventMap.put("Pos1", new InstantCommand(() -> drivetrain.setAprilTagTarget(1))); // Sets the isolation to pos 1, tags 8 and 1 CABLE
        eventMap.put("Pos2", new InstantCommand(() -> drivetrain.setAprilTagTarget(2))); // Sets the isolation to pos 2, tags 7 and 2 MIDDLE
        eventMap.put("Pos3", new InstantCommand(() -> drivetrain.setAprilTagTarget(3))); // Sets the isolation to pos 3, tags 6 and 3 OPEN
        
        return eventMap;
    }
}
 