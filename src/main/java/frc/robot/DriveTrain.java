package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static frc.robot.Constants.*;

public class DriveTrain {
	/** Maximum speed (meters per second) a motor may spin at. */
	private static final double maximumAttainableSpeed = 0.5;
	private static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
		/* NOTE: Positive X is forwards, Positive Y is toward the left */
		new Translation2d( 0.2794,  0.2794), // Front Left
		new Translation2d( 0.2794, -0.2794), // Front Right
		new Translation2d(-0.2794, -0.2794), // Bottom Right
		new Translation2d(-0.2794,  0.2794)  // Bottom Left
	);

	// TODO: Enter Motor Indicies
    private SwerveModule frontLeft  = new SwerveModule(newDrive(0, false), newSteer(0, false));
    private SwerveModule frontRight = new SwerveModule(newDrive(0, false), newSteer(0, false));
    private SwerveModule backRight  = new SwerveModule(newDrive(0, true), newSteer(0, false));
    private SwerveModule backLeft   = new SwerveModule(newDrive(0, false), newSteer(0, false));

	/**
	 * Set the linear and angular velocity targets.
	 *
	 * @param vForwards positive-forward (meters per second).
	 * @param vSideways positive-rightward (meters per second).
	 * @param vAngle positive-clockwise (radians per second).
	 */
    public void drive(double vForwards, double vSideways, double vAngle) {
		ChassisSpeeds chassisSpeeds = new ChassisSpeeds(vForwards, -vSideways, -vAngle);
		SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maximumAttainableSpeed);
		useSwerveModuleStates(swerveModuleStates);
    }

	private void useSwerveModuleStates(SwerveModuleState[] states) {
		frontLeft.setTargetState(states[0]);
		frontRight.setTargetState(states[1]);
		backRight.setTargetState(states[2]);
		backLeft.setTargetState(states[3]);
	}

	public void periodic() {
		boolean allMotorsInPosition =
				frontLeft.steerWheel() && frontRight.steerWheel()
				&& backRight.steerWheel() && backLeft.steerWheel();

		if (allMotorsInPosition) {
			frontLeft.driveWheel();
			frontRight.driveWheel();
			backRight.driveWheel();
			backLeft.driveWheel();
		}
	}

	public void stopMotors() {
		frontLeft.resetWheel();
		frontRight.resetWheel();
		backRight.resetWheel();
		backLeft.resetWheel();
	}

	private static WPI_TalonFX newDrive(int motorIndex, boolean inverted) {
		WPI_TalonFX motor = new WPI_TalonFX(motorIndex);
		motor.setSelectedSensorPosition(0, 0, kTimeoutMs);
		motor.setInverted(inverted);
		motor.setSensorPhase(inverted);
		motor.configNeutralDeadband(0.1, kTimeoutMs);
		return motor;
	}

	private static WPI_TalonFX newSteer(int motorIndex, boolean inverted) {
		WPI_TalonFX motor = newDrive(motorIndex, inverted);
		motor.config_kF(0, kF, kTimeoutMs);
		motor.config_kP(0, kP, kTimeoutMs);
		motor.config_kI(0, (int) kI, kTimeoutMs);
		motor.config_kD(0, kD, kTimeoutMs);
		motor.config_IntegralZone(0, kIntegralZone, kTimeoutMs);
		motor.configClosedLoopPeakOutput(0, kMaxSteerPercent, kTimeoutMs);
		motor.configClosedLoopPeriod(0, kSteerLoopPeriodMs, kTimeoutMs);
		motor.selectProfileSlot(0, 0);
		return motor;
	}
}