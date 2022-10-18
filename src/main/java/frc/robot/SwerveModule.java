package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    /** Amount of error before the swerve module begins spinning */
    private final static double STEER_ERROR = 0.05 * 2 * Math.PI;
    // TODO: Get drive wheel's Gear Ratio and Circumfrence
    /** The gear ratio of the drive wheel. */
    private final static double GEAR_RATIO = 10 / 5;
    /** The circumference of the drive wheel. */
    private final static double WHEEL_CIRCUMFERENCE = 0.0508 * 2 * Math.PI;
    private final static int ENCODER_RESOLUTION = 2048;
    private final static double COUNTS_PER_METER = GEAR_RATIO * ENCODER_RESOLUTION / WHEEL_CIRCUMFERENCE;
    private WPI_TalonFX drive;
    private WPI_TalonFX steer;

    /** The target state. If null, motors will stop */
    private SwerveModuleState targetState;

    public SwerveModule(WPI_TalonFX drive, WPI_TalonFX steer) {
        this.drive = drive;
        this.steer = steer;
    }

    /**
     * Set the target state of this swerve module.
     * 
     * @param state The target state to use, or null to stop motors.
     */
    public void setTargetState(SwerveModuleState state) {
        targetState = SwerveModuleState.optimize(state, getCurrentAngle());
    }

    /**
     * Applies steering if necessary, then returns wether this
     * module is within a reasonable error from the target angle.
     * 
     * @return Wether the current angle is close enough to the target angle.
     */
    public boolean steerWheel() {
        steer.set(TalonFXControlMode.Position, angleToCounts(targetState.angle));
        double delta = targetState.angle.getRadians() - getCurrentAngle().getRadians();
        boolean isWithinAcceptableError = Math.abs(delta) <= STEER_ERROR;
        return isWithinAcceptableError;
    }

    /**
     * Drives the wheels according to the set target speed.
     */
    public void driveWheel() {
        if (targetState == null) {
            drive.set(0);
        } else {
            drive.set(
                TalonFXControlMode.Velocity,
                // Speed in counts per 100ms
                meterSpeedToCountSpeed(targetState.speedMetersPerSecond * 10)
            );
        }
    }

    /**
     * Stops the wheel and rotates it to zero.
     */
    public void resetWheel() {
        targetState = null;
        drive.set(0);
    }

    private Rotation2d getCurrentAngle() {
        int steerRotationCounts = (int) steer.getSelectedSensorPosition() % ENCODER_RESOLUTION;
        double steerRotationPercentage = (double) steerRotationCounts / ENCODER_RESOLUTION;
        double steerRotationRadians = 2 * Math.PI * steerRotationPercentage;

        return new Rotation2d(steerRotationRadians);
    }

    private int angleToCounts(Rotation2d angle) {
        double anglePercentage = (angle.getRadians() / 2 / Math.PI);
        int angleCounts = (int) (anglePercentage * ENCODER_RESOLUTION);
        return angleCounts;
    }

    private int meterSpeedToCountSpeed(double speedMetersPerSecond) {
        return (int) (speedMetersPerSecond * COUNTS_PER_METER);
    }
}
