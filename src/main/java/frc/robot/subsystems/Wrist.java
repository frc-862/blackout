package frc.robot.subsystems;

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
    private double LEFT_OFFSET = 0;
    private double RIGHT_OFFSET = 0;

    public Wrist() {
        rightMotor = FalconConfig.createMotor(CAN.RIGHT_WRIST_MOTOR,
                WristConstants.RIGHT_MOTOR_INVERT, WristConstants.SUPPLY_CURRENT_LIMIT,
                CollectorConstants.STATOR_CURRENT_LIMIT, WristConstants.NEUTRAL_MODE);
        leftMotor = FalconConfig.createMotor(CAN.LEFT_WRIST_MOTOR, WristConstants.LEFT_MOTOR_INVERT,
                WristConstants.SUPPLY_CURRENT_LIMIT, WristConstants.STATOR_CURRENT_LIMIT,
                WristConstants.NEUTRAL_MODE);

        

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public void initialize() {
        currState = wristStates.Stow;
        goalState = wristStates.Stow;

        LEFT_OFFSET = WristConstants.LEFT_OFFSET;
        RIGHT_OFFSET = WristConstants.RIGHT_OFFSET;
    }

    
    public void setTargetAngle(double angle) {
        
    }
    
    /**
     * Gets the angle of the wrist from the encoder
     * 
     * @return Rotation2d of the wrist from encoder
     */
    public Rotation2d getAngle() { //TODO FIX
        return Rotation2d.fromDegrees(MathUtil.inputModulus(leftMotor.getRotorPosition().getValue() * WristConstants.POSITION_CONVERSION_FACTOR - LEFT_OFFSET, -180, 180));
    }
    
    public void disableWrist() {
        setPower(0d);
        disableWrist = true;
    }
    
    /**
     * Set power to both motors
     * @param power sets power from percent output
     */
    public void setPower(double power) {
        rightMotor.set(power);
        leftMotor.set(power);
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

    public void periodic() {
        if (currState != goalState) {
            setTargetAngle(WristAngles.angleMap().get(goalState));
            currState = goalState;
        }

        upController.setP(LightningShuffleboard.getDouble("Collector", "UP_kP", WristConstants.UP_kP));
        upController.setI(LightningShuffleboard.getDouble("Collector", "UP_kI", WristConstants.UP_kI));
        upController.setD(LightningShuffleboard.getDouble("Collector", "UP_kD", WristConstants.UP_kD));

        downController.setP(LightningShuffleboard.getDouble("Collector", "DOWN_kP", WristConstants.DOWN_kP));
        downController.setI(LightningShuffleboard.getDouble("Collector", "DOWN_kI", WristConstants.DOWN_kI));
        downController.setD(LightningShuffleboard.getDouble("Collector", "DOWN_kD", WristConstants.DOWN_kD));

        targetAngle = (LightningShuffleboard.getDouble("Collector", "UP_F", WristConstants.UP_F));

        if (targetAngle - currentAngle > 0) {
            PIDOutput = upController.calculate(currentAngle, targetAngle);
        } else {
            PIDOutput = downController.calculate(currentAngle, targetAngle);
        }

        FOutput = WristConstants.WRIST_KF_MAP.get(getAngle().getDegrees());

        if(disableWrist){
            setPower(0);
        } else {
            setPower(PIDOutput + FOutput);
        }
    }
}