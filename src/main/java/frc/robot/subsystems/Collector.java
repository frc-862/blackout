package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.RobotMap.*;
import frc.thunder.config.FalconConfig;
import frc.thunder.config.NeoConfig;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.shuffleboard.LightningShuffleboardPeriodic;

/**
 * The collector subsystem
 */
public class Collector extends SubsystemBase {
    // The collector motor
    private TalonFX motor;

    // The color sensor

    // Periodic Shuffleboard
    private LightningShuffleboardPeriodic periodicShuffleboard;

    private int currCurrentLimit = CollectorConstants.CURRENT_LIMIT;

    // Enum of possible game pieces
    public enum GamePiece {
        CONE, CUBE, NONE
    }

    private GamePiece gamePiece = GamePiece.CUBE;

    public Collector() {
        // Create the motor and configure it
        motor = FalconConfig.createMotor(CAN.COLLECTOR_MOTOR, CollectorConstants.MOTOR_INVERT, CollectorConstants.CURRENT_LIMIT, CollectorConstants.CURRENT_LIMIT, CollectorConstants.NEUTRAL_MODE);

        // Initialize the shuffleboard values and start logging data
        initialiizeShuffleboard();

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public void runCollector(double power) {
        motor.set(TalonFXControlMode.PercentOutput, power);
    }

    // Method to start logging
    @SuppressWarnings("unchecked")
    private void initialiizeShuffleboard() {
        periodicShuffleboard = new LightningShuffleboardPeriodic("Collector", CollectorConstants.LOG_PERIOD,
            new Pair<String, Object>("Collector motor output percent", (DoubleSupplier) () -> motor.getMotorOutputPercent()));
            // new Pair<String, Object>("Collector motor controller input voltage", (DoubleSupplier) () -> motor.getBusVoltage()),
    }

    /**
     * Sets smart current limit if its different from the current current limit
     * @param currentLimit the new smart current limit
     */
    public void setCurrentLimit(int currentLimit) {
        if(currentLimit != currCurrentLimit) {
            motor.configSupplyCurrentLimit(currentLimit);
        }
        currCurrentLimit = currentLimit;        
    }

    //Used to check if the collector is stalling. Used to detect if the collector is holding a game piece
    public boolean isStalling(){
        return motor.getOutputCurrent() > CollectorConstants.STALL_POWER;
    }

    /**
     * Gets the game piece detected by the color sensor
     * 
     * @return the game piece detected by the color sensor (Either CUBE, CONE, or NONE)
     */

    public GamePiece getGamePiece() {
        return gamePiece;
    }

    // For the driver to set the game piece manually
    public void setGamePiece(GamePiece gamePiece) {
        this.gamePiece = gamePiece;
    }

    /**
     * Sets the power of the collector motor
     * 
     * @param power the percent speed to set the elevator motor to
     */
    public void setPower(double power) {
        if(getGamePiece() == GamePiece.CONE) {
            motor.set(MathUtil.clamp(power, -1, 1));
        } else {
            motor.set(power);
        }
    }
    
    /**
     * stop Sets the power of the collector motor to 0
     */
    public void stop() {
        setPower(0d);
    }

    @Override
    public void periodic() {

        periodicShuffleboard.loop();
    }
}
