package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
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

    wristStates currState = wristStates.Stow;
    wristStates goalState = wristStates.Stow;

    private PIDController upController = new PIDController(WristConstants.UP_kP, WristConstants.UP_kI,
            WristConstants.UP_kD);
    private PIDController downController = new PIDController(WristConstants.DOWN_kP, WristConstants.DOWN_kI,
            WristConstants.DOWN_kD);

    // private Encoder encoder = new Encoder(0, 0, 0,
    // WristConstants.ENCODER_INVERT);
    private DutyCycleEncoder encoder = new DutyCycleEncoder(0);

    private boolean disableWrist = false;

    private double FOutput = 0d;
    private double PIDOutput = 0d;
    private double targetAngle;

    private double ABS_OFFSET = 180.9d;

    public Wrist() {
        motor = NeoConfig.createMotor(CAN.WRIST_MOTOR, WristConstants.MOTOR_INVERT, WristConstants.CURRENT_LIMIT,
                Constants.VOLTAGE_COMPENSATION, WristConstants.MOTOR_TYPE, WristConstants.IDLE_MODE);

        // Channel 0 is up
        // Channel 1 is down

        setTargetAngle(WristAngles.angleMap().get(goalState));

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
    public Rotation2d getAngle() { // TODO FIX CONVERSION factor
        return Rotation2d.fromDegrees(motor.getEncoder().getPosition() * WristConstants.POSITION_CONVERSION_FACTOR/**
                                                                                                                   * +*
                                                                                                                   * OFFSET
                                                                                                                   */
        );
    }

    public void disableWrist() {
        stop();
        disableWrist = true;
    }

    public void enableWrist() {
        disableWrist = false;
    }

    /**
     * Set power to both motors
     * 
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
        return Math.abs(getAngle().getDegrees() - targetAngle) < WristConstants.WRIST_TOLERANCE;
    }

    public void adjustWrist(double bias) {
        setTargetAngle(getAngle().getDegrees() + bias);
    }

    public boolean isStalling() {
        return motor.getOutputCurrent() >= WristConstants.STALL_CURRENT;
    }

    public boolean isMoving() {
        return Math.abs(motor.getEncoder().getVelocity()) >= WristConstants.IS_MOVING_THRESHHOLD;

    }

    public boolean getLimitSwitch() {
        return motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).isPressed();
    }

    public double getAbsoluteAngle() {
        return (encoder.get() * 360) - ABS_OFFSET;
    }

    public Rotation2d getRawAngle() {
        return Rotation2d.fromDegrees(motor.getEncoder().getPosition() * WristConstants.POSITION_CONVERSION_FACTOR);
    }

    public void setAngle(double angle) {
        // OFFSET = angle - getRawAngle().getDegrees();
        motor.getEncoder().setPosition(angle / WristConstants.POSITION_CONVERSION_FACTOR);
    }

    public void periodic() {
        if (currState != goalState) {
            setTargetAngle(WristAngles.angleMap().get(goalState));
            // if (onTarget()) {
                currState = goalState;
            // }
        }

        upController.setP(LightningShuffleboard.getDouble("Wrist", "UP_kP", WristConstants.UP_kP));
        upController.setI(LightningShuffleboard.getDouble("Wrist", "UP_kI", WristConstants.UP_kI));
        upController.setD(LightningShuffleboard.getDouble("Wrist", "UP_kD", WristConstants.UP_kD));

        // targetAngle = (LightningShuffleboard.getDouble("Wrist", "Target Angle", 0));
        LightningShuffleboard.setDouble("Wrist", "Current Relative Angle", getAngle().getDegrees());
        LightningShuffleboard.setDouble("Wrist", "Current Absolute Angle", getAbsoluteAngle());

        LightningShuffleboard.setString("Wrist", "Goal State", getGoalState().toString());
        LightningShuffleboard.setString("Wrist", "Current State", getCurrState().toString());
        LightningShuffleboard.setDouble("Wrist", "Target Angle", WristAngles.angleMap().get(goalState));

        if (targetAngle - getAngle().getDegrees() > 0) {
            PIDOutput = upController.calculate(getAngle().getDegrees(), targetAngle);
        } else {
            PIDOutput = downController.calculate(getAngle().getDegrees(), targetAngle);
        }

        LightningShuffleboard.setDouble("Wrist", "Power Output", PIDOutput);

        if (!disableWrist) {
            setPower(PIDOutput);
        } else {
            // stop();
        }

        if (getAbsoluteAngle() < -1.5) {
            setAngle(0);
        }

        if (getLimitSwitch()) {
            // OFFSET = 0;
            // OFFSET = WristConstants.MAX_ANGLE - getAngle().getDegrees();

            setAngle(WristConstants.MAX_ANGLE);
        }
    }
}