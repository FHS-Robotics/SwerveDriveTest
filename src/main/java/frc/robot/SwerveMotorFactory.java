package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import static frc.robot.Constants.*;
import static frc.robot.Settings.*;

public class SwerveMotorFactory {
    public static WPI_TalonFX newDrive(int motorIndex, boolean inverted) {
		WPI_TalonFX motor = new WPI_TalonFX(motorIndex);
		motor.setInverted(inverted);
		return motor;
	}

	public static WPI_TalonFX newSteer(int motorIndex, boolean inverted) {
		WPI_TalonFX motor = new WPI_TalonFX(motorIndex);
		motor.setSelectedSensorPosition(0, 0, kTimeoutMs);
		motor.setInverted(inverted);
		motor.setSensorPhase(inverted);
		motor.configNeutralDeadband(0.1, kTimeoutMs);
		sF.collect((value) -> motor.config_kF(0, value, kTimeoutMs));
		sP.collect((value) -> motor.config_kP(0, value, kTimeoutMs));
		sI.collect((value) -> motor.config_kI(0, value, kTimeoutMs));
		sD.collect((value) -> motor.config_kD(0, value, kTimeoutMs));
		sIntegralZone.collect((value) -> motor.config_IntegralZone(0, value, kTimeoutMs));
		motor.configClosedLoopPeakOutput(0, kMaxSteerPercent, kTimeoutMs);
		motor.configClosedLoopPeriod(0, kSteerLoopPeriodMs, kTimeoutMs);
		motor.selectProfileSlot(0, 0);
		return motor;
	}
}
