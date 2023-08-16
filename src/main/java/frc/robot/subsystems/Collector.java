package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.RobotMap.*;
import frc.thunder.config.NeoConfig;
import frc.thunder.shuffleboard.LightningShuffleboardPeriodic;
import frc.thunder.shuffleboard.LightningShuffleboard;

/**
 * The collector subsystem
 */
public class Collector extends SubsystemBase {
    // The collector motor
    private CANSparkMax motor;

    // Periodic Shuffleboard
    private LightningShuffleboardPeriodic periodicShuffleboard;

    private GamePiece gamePiece = GamePiece.CUBE;

    // private PIDController PIDController = new PIDController(CollectorConstants.MOTOR_kP, CollectorConstants.MOTOR_kI, CollectorConstants.MOTOR_kD);
    // private double kS = CollectorConstants.MOTOR_kS;
    // private double kV = CollectorConstants.MOTOR_kV;

    private int currCurrentLimit = CollectorConstants.CURRENT_LIMIT;

    public Collector() {
        // Create the motor and configure it
        motor = NeoConfig.createMotor(CAN.COLLECTOR_MOTOR, CollectorConstants.MOTOR_INVERT, CollectorConstants.CURRENT_LIMIT,
            Constants.VOLTAGE_COMPENSATION, CollectorConstants.MOTOR_TYPE, CollectorConstants.IDLE_MODE);

        // PIDController.setTolerance(50);
        
        // Initialize the shuffleboard values and start logging data
        initialiizeShuffleboard();

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    /**
     * Sets the power of the collector motors
     * -1.0 <> 1.0
     * @param power the percent speed to set the collector motors to
     */
    public void setPercentPower(double power) {
        motor.set(power);
    }

    /**
     * Sets the power of the collector motors in RPM
     * 
     * @param RPM the velocity to set the collector motors to in RPM
     */
    // public void setRPM(double RPM) {
    //     MathUtil.clamp(RPM, -CollectorConstants.MAX_RPM, CollectorConstants.MAX_RPM);
    //     motor.setVoltage(PIDController.calculate(motor.getEncoder().getVelocity(), RPM));
    // }

    // Method to start logging
    @SuppressWarnings("unchecked")
    private void initialiizeShuffleboard() {
        periodicShuffleboard = new LightningShuffleboardPeriodic("Collector", CollectorConstants.LOG_PERIOD,
            new Pair<String, Object>("Collector motor output percent", (DoubleSupplier) () -> motor.get()),
            new Pair<String, Object>("Collector RPM", (DoubleSupplier) () -> getCurrentRPM()),
            new Pair<String, Object>("Is Stalling", (BooleanSupplier) () -> isStalling()));
    }

    /**
     * Sets smart current limit if its different from the current smart current limit
     * 
     * @param currentLimit the new smart current limit
     */
    public void setCurrentLimit(int currentLimit) {
        if(currCurrentLimit != currentLimit) {
            motor.setSmartCurrentLimit(currentLimit);
            currCurrentLimit = currentLimit;
        }
    }


    /**
     * Returns true if the collector is stalling
     */
    public boolean isStalling() {
        return (motor.getOutputCurrent() >= CollectorConstants.STALL_POWER);
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

    public void setCoastMode() {
        motor.setIdleMode(IdleMode.kCoast);
    }

    public void setBrakeMode() {
        motor.setIdleMode(IdleMode.kBrake);
    }

    /**
     * gets current RPM from internal encoders
     * @return average RPM of collector
     */
    public double getCurrentRPM() {
		return motor.getEncoder().getVelocity();
	}

    /**
     * stop Sets the power of the collector motor to 0
     */
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void periodic() {

        periodicShuffleboard.loop();

        // PIDController.setP(LightningShuffleboard.getDouble("Collector", "kP", CollectorConstants.MOTOR_kP));
        // PIDController.setI(LightningShuffleboard.getDouble("Collector", "kI", CollectorConstants.MOTOR_kI));
        // PIDController.setD(LightningShuffleboard.getDouble("Collector", "kD", CollectorConstants.MOTOR_kD));

        // kS = LightningShuffleboard.getDouble("Collector", "kS", CollectorConstants.MOTOR_kS);
        // kV = LightningShuffleboard.getDouble("Collector", "kV", CollectorConstants.MOTOR_kV);


        // double FFOutput = kS * Math.signum(getCurrentRPM()) + kV * getCurrentRPM();


        LightningShuffleboard.setDouble("Collector", "Collector Amps", motor.getAppliedOutput());
        LightningShuffleboard.setDouble("Collector", "RPM", getCurrentRPM());
        // LightningShuffleboard.setDouble("Collector", "Commanded Power", motor.get());

        // LightningShuffleboard.setDouble("Colletor", "",motor. );

        // setRPM(LightningShuffleboard.getDouble("Collector", "Set RPM", 0));
    }
}
