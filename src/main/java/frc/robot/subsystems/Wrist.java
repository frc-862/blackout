package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.RobotMap.CAN;
import frc.robot.Constants.WristConstants;
import frc.thunder.config.FalconConfig;


/** Add your docs here. */
public class Wrist extends SubsystemBase {

    TalonFX rightMotor;
    TalonFX leftMotor;

    private boolean disableWrist = false;

    public Wrist() {
        rightMotor = FalconConfig.createMotor(CAN.RIGHT_WRIST_MOTOR,
                WristConstants.RIGHT_MOTOR_INVERT, WristConstants.SUPPLY_CURRENT_LIMIT,
                CollectorConstants.STATOR_CURRENT_LIMIT, WristConstants.NEUTRAL_MODE);
        leftMotor = FalconConfig.createMotor(CAN.LEFT_WRIST_MOTOR, WristConstants.LEFT_MOTOR_INVERT,
                WristConstants.SUPPLY_CURRENT_LIMIT, WristConstants.STATOR_CURRENT_LIMIT,
                WristConstants.NEUTRAL_MODE);

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public void setAngle(double angle) {
        
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
}
