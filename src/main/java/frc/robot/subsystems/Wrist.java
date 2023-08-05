package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.WristAngles;
import frc.robot.Constants.RobotMap.CAN;
import frc.robot.Constants.WristAngles.wristStates;
import frc.robot.Constants.WristConstants;
import frc.thunder.config.FalconConfig;


/** Add your docs here. */
public class Wrist extends SubsystemBase {

    TalonFX rightMotor;
    TalonFX leftMotor;

    wristStates currState;
    wristStates goalState;

    private PIDController upController = new PIDController(WristConstants.UP_kP, WristConstants.kI, WristConstants.UP_kD);
    private PIDController downController = new PIDController(WristConstants.DOWN_kP, WristConstants.kI, WristConstants.DOWN_kD);

    private SparkMaxAbsoluteEncoder absoluteEncoder;

    private boolean disableWrist = false;

    public Wrist() {
        rightMotor = FalconConfig.createMotor(CAN.RIGHT_WRIST_MOTOR,
                WristConstants.RIGHT_MOTOR_INVERT, WristConstants.SUPPLY_CURRENT_LIMIT,
                CollectorConstants.STATOR_CURRENT_LIMIT, WristConstants.NEUTRAL_MODE);
        leftMotor = FalconConfig.createMotor(CAN.LEFT_WRIST_MOTOR, WristConstants.LEFT_MOTOR_INVERT,
                WristConstants.SUPPLY_CURRENT_LIMIT, WristConstants.STATOR_CURRENT_LIMIT,
                WristConstants.NEUTRAL_MODE);

        double ads = rightMotor.getSelectedSensorPosition();

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public void initialize() {
        currState = wristStates.Stow;
        goalState = wristStates.Stow;
    }

    public void periodic() {
        if (currState != goalState) {
            setTargetAngle(WristAngles.angleMap().get(goalState));
            currState = goalState;
        }
    }

    public void setTargetAngle(double angle) {

    }

    public double getAngle(){
        return 0;
    }

    public void disableWrist() {
        setPower(0d);
        disableWrist = true;
    }

    public void setPower(double power) {
        if (!disableWrist) {
            rightMotor.set(TalonFXControlMode.PercentOutput, power);
            leftMotor.set(TalonFXControlMode.PercentOutput, power);
        } else {
            rightMotor.set(TalonFXControlMode.PercentOutput, 0d);
            leftMotor.set(TalonFXControlMode.PercentOutput, 0d);
        }
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
}

// 2048  = 1
// 360 = 1
