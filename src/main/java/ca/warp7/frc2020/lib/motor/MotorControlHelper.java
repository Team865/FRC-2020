package ca.warp7.frc2020.lib.motor;

import ca.warp7.frc2020.lib.control.PID;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

/**
 * Helper functions for motor control
 */
public class MotorControlHelper {

    /**
     * Create a master VictorSPX motor controller
     *
     * @param deviceID the CAN id (shown on Phoenix Tuner)
     * @return the motor controller object with default settings
     */
    public static VictorSPX createMasterVictorSPX(int deviceID) {
        VictorSPX master = new VictorSPX(deviceID);
        master.configFactoryDefault();
        master.setNeutralMode(NeutralMode.Brake);
        master.configVoltageCompSaturation(12.0);
        master.enableVoltageCompensation(true);
        return master;
    }

    /**
     * Create a master TalonFX motor controller
     *
     * @param deviceID the CAN id (shown on Phoenix Tuner)
     * @return the motor controller object with default settings
     */
    public static TalonFX createMasterTalonFX(int deviceID) {
        TalonFX master = new TalonFX(deviceID);
        master.configFactoryDefault();
        master.setNeutralMode(NeutralMode.Brake);
        master.configVoltageCompSaturation(12.0);
        master.enableVoltageCompensation(true);
        return master;
    }

    /**
     * Create a master Spark MAX motor controller
     *
     * @param deviceID the CAN id (shown on Phoenix Tuner)
     * @return the motor controller object with default settings
     */
    public static CANSparkMax createMasterSparkMAX(int deviceID) {
        CANSparkMax master = new CANSparkMax(deviceID, CANSparkMaxLowLevel.MotorType.kBrushless);
        master.restoreFactoryDefaults();
        master.setIdleMode(CANSparkMax.IdleMode.kBrake);
        master.enableVoltageCompensation(12.0);
        return master;
    }

    /**
     * Create a follower TalonFX
     *
     * @param master     the master motor controller to follow
     * @param deviceID   the CAN id
     * @param invertType the invert type
     * @return the follower TalonFX object
     */
    public static TalonFX assignFollowerTalonFX(
            BaseMotorController master,
            int deviceID,
            InvertType invertType
    ) {
        TalonFX follower = new TalonFX(deviceID);
        follower.configFactoryDefault();
        follower.setNeutralMode(NeutralMode.Brake);
        follower.setInverted(invertType);
        follower.follow(master);
        return follower;
    }

    /**
     * Create a follower VictorSPX
     *
     * @param master     the master motor controller to follow
     * @param deviceID   the CAN id
     * @param invertType the invert type
     */
    public static void assignFollowerVictorSPX(
            BaseMotorController master,
            int deviceID,
            InvertType invertType
    ) {
        VictorSPX follower = new VictorSPX(deviceID);
        follower.configFactoryDefault();
        follower.setNeutralMode(NeutralMode.Brake);
        follower.setInverted(invertType);
        follower.follow(master);
    }

    /**
     * Create a follower Spark MAX
     *
     * @param master   the master motor controller to follow
     * @param deviceID the CAN id
     * @param inverted whether the follower is inverted
     */
    @SuppressWarnings("resource")
    public static void assignFollowerSparkMAX(
            CANSparkMax master,
            int deviceID,
            boolean inverted
    ) {
        CANSparkMax follower = new CANSparkMax(deviceID, CANSparkMaxLowLevel.MotorType.kBrushless);
        follower.restoreFactoryDefaults();
        follower.setIdleMode(CANSparkMax.IdleMode.kBrake);
        follower.follow(master, inverted);
    }

    /**
     * Configures the PID gains of a motor controller
     *
     * @param motor the motor controller to configure
     * @param pid   the PID values to configure
     */
    public static void configurePID(BaseMotorController motor, PID pid) {
        motor.config_kP(0, pid.kP);
        motor.config_kI(0, pid.kI);
        motor.config_kD(0, pid.kD);
        motor.selectProfileSlot(0, 0);
    }
}
