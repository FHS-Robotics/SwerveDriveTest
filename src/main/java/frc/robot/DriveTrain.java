package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static frc.robot.SwerveMotorFactory.*;

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

    private SwerveModule frontLeft  = new SwerveModule(newDrive(0, false), newSteer(4, false));
    private SwerveModule frontRight = new SwerveModule(newDrive(1, false), newSteer(5, false));
    private SwerveModule backRight  = new SwerveModule(newDrive(2, false), newSteer(6, false));
    private SwerveModule backLeft   = new SwerveModule(newDrive(3, false), newSteer(7, false));

	/**
	 * Set the linear and angular velocity targets.
	 *
	 * @param vForwards positive-forward (meters per second).
	 * @param vSideways positive-rightward (meters per second).
	 * @param vAngle positive-clockwise (radians per second).
	 */
    public void drive(double vForwards, double vSideways, double vAngle) {
		// TODO: Have linear velocity be field oriented, if wanted.
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
}