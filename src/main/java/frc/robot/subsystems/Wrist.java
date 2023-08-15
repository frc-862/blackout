package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WristAngles;
import frc.robot.Constants.RobotMap.CAN;
import frc.robot.Constants.WristAngles.wristStates;
import frc.robot.Constants.WristConstants;
import frc.thunder.config.NeoConfig;
import frc.thunder.shuffleboard.LightningShuffleboard;


/** Add your docs here. */
public class Wrist extends SubsystemBase {

    
    private CANSparkMax motor;

    wristStates currState;
    wristStates goalState;

    private PIDController upController = new PIDController(WristConstants.UP_kP, WristConstants.UP_kI, WristConstants.UP_kD);
    private PIDController downController = new PIDController(WristConstants.DOWN_kP, WristConstants.DOWN_kI, WristConstants.DOWN_kD);

    private boolean disableWrist = false;

    private double FOutput = 0d;
    private double PIDOutput = 0d;
    private double currentAngle;
    private double targetAngle;

    private double OFFSET = 0d;

    public Wrist() {
        motor = NeoConfig.createMotor(CAN.WRIST_MOTOR, WristConstants.MOTOR_INVERT, WristConstants.CURRENT_LIMIT, 
            Constants.VOLTAGE_COMPENSATION, WristConstants.MOTOR_TYPE, WristConstants.IDLE_MODE);

        // Channel 0 is up
        // Channel 1 is down

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public void initialize() {
        currState = wristStates.Stow;
        goalState = wristStates.Stow;

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
        return Rotation2d.fromDegrees(MathUtil.inputModulus(motor.getEncoder().getPosition() * WristConstants.POSITION_CONVERSION_FACTOR - OFFSET, -180, 180));
    }

    public void setOffset() {
        OFFSET = motor.getEncoder().getPosition() * WristConstants.POSITION_CONVERSION_FACTOR + WristConstants.MIN_ANGLE;
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
        motor.set(power);
    }

    public void stop() {
        motor.stopMotor();
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

    public void adjustWrist(double bias) {
        setTargetAngle(getAngle().getDegrees() + bias);
    }

    public void zero() {
        if(!isStalling()){
            motor.set(WristConstants.ZERO_SPEED);
        } else {
            stop();
            setOffset();
        }
    }

    public boolean isStalling(){
        return motor.getOutputCurrent() >= WristConstants.STALL_CURRENT;
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

        targetAngle = (LightningShuffleboard.getDouble("Collector", "Target Angle", 0));

        

        
        if (targetAngle - currentAngle > 0) {
            PIDOutput = upController.calculate(currentAngle, targetAngle);
        } else {
            PIDOutput = downController.calculate(currentAngle, targetAngle);
        }
        
        FOutput = WristConstants.WRIST_KF_MAP.get(getAngle().getDegrees());

        if(!disableWrist){
            motor.set(FOutput + PIDOutput);
        } else {
            stop();
        }
    }
}   