package frc.robot.subsystems;


import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.WristAngles;
import frc.robot.Constants.RobotMap.CAN;
import frc.robot.Constants.WristAngles.wristStates;
import frc.robot.Constants.WristConstants;
import frc.thunder.config.FalconConfig;
import frc.thunder.shuffleboard.LightningShuffleboard;


/** Add your docs here. */
public class Wrist extends SubsystemBase {

    TalonFX rightMotor;
    TalonFX leftMotor;

    wristStates currState;
    wristStates goalState;

    private PIDController upController = new PIDController(WristConstants.UP_kP, WristConstants.UP_kI, WristConstants.UP_kD);
    private PIDController downController = new PIDController(WristConstants.DOWN_kP, WristConstants.DOWN_kI, WristConstants.DOWN_kD);

    private boolean disableWrist = false;

    private double FOutput = 0d;
    private double PIDOutput = 0d;
    private double currentAngle;
    private double targetAngle;

    public Wrist() {
        leftMotor = FalconConfig.createMotor(CAN.LEFT_WRIST_MOTOR, "rio", WristConstants.LEFT_MOTOR_INVERT,
                WristConstants.SUPPLY_CURRENT_LIMIT, WristConstants.STATOR_CURRENT_LIMIT,
                WristConstants.NEUTRAL_MODE, WristConstants.UP_kP, WristConstants.UP_kI, WristConstants.UP_kD,
                WristConstants.DOWN_kP, WristConstants.DOWN_kI, WristConstants.DOWN_kD);

        rightMotor = FalconConfig.createMotor(CAN.RIGHT_WRIST_MOTOR, "rio",
                WristConstants.RIGHT_MOTOR_INVERT, WristConstants.SUPPLY_CURRENT_LIMIT,
                CollectorConstants.STATOR_CURRENT_LIMIT, WristConstants.NEUTRAL_MODE, WristConstants.UP_kP, WristConstants.UP_kI, WristConstants.UP_kD,
                WristConstants.DOWN_kP, WristConstants.DOWN_kI, WristConstants.DOWN_kD);

        // Channel 0 is up
        // Channel 1 is down

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public void initialize() {
        currState = wristStates.Stow;
        goalState = wristStates.Stow;

        rightMotor.setControl(new Follower(10, false)); // TODO make sure the direction is right 
    }

    
    public void setTargetAngle(double angle) {
        targetAngle = MathUtil.clamp(angle, WristConstants.MIN_ANGLE, WristConstants.MAX_ANGLE);
    }
    
    /**
     * Gets the angle of the wrist from the encoder
     * 
     * @return Rotation2d of the wrist from encoder
     */
    public Rotation2d getAngle() { //TODO FIX CONVERSION factor
        return Rotation2d.fromDegrees(MathUtil.inputModulus(leftMotor.getRotorPosition().getValue() * WristConstants.POSITION_CONVERSION_FACTOR - WristConstants.LEFT_OFFSET, -180, 180));
    }
    
    public void disableWrist() {
        stop();
        disableWrist = true;
    }
    
    /**
     * Set power to both motors
     * @param power sets power from percent output
     */
    public void setPower(double power) {
        leftMotor.set(power);
    }

    public void stop() {
        leftMotor.stopMotor();
    }

    public void setGoalState(wristStates goalState) {
        this.goalState = goalState;
    }
    
    public wristStates getCurrState() {
        return currState;
    }
    
    public wristStates getGoalState() {
        return goalState;
    }

    public boolean onTarget() {
        return (getAngle().getDegrees() - targetAngle) < WristConstants.WRIST_TOLERANCE;
    }

    public void periodic() {
        if (currState != goalState) {
            setTargetAngle(WristAngles.angleMap().get(goalState));
            currState = goalState;
        }
        
        // TODO THESE DON'T effect anything
        upController.setP(LightningShuffleboard.getDouble("Collector", "UP_kP", WristConstants.UP_kP));
        upController.setI(LightningShuffleboard.getDouble("Collector", "UP_kI", WristConstants.UP_kI));
        upController.setD(LightningShuffleboard.getDouble("Collector", "UP_kD", WristConstants.UP_kD));

        downController.setP(LightningShuffleboard.getDouble("Collector", "DOWN_kP", WristConstants.DOWN_kP));
        downController.setI(LightningShuffleboard.getDouble("Collector", "DOWN_kI", WristConstants.DOWN_kI));
        downController.setD(LightningShuffleboard.getDouble("Collector", "DOWN_kD", WristConstants.DOWN_kD));

        targetAngle = (LightningShuffleboard.getDouble("Collector", "Target Angle", 0));

        

        if(!disableWrist){
            if (targetAngle - currentAngle > 0) {
                // PIDOutput = upController.calculate(currentAngle, targetAngle);
                //TODO Figure out convertion from dergrees to rotations
                leftMotor.setControl(new PositionVoltage(targetAngle, false, WristConstants.UP_FF, 0, false));
                
            } else {
                leftMotor.setControl(new PositionVoltage(targetAngle, false, WristConstants.DOWN_FF, 1, false));
            }
        } else {
            stop();
        }
    }
}