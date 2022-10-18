package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

import static frc.robot.Settings.*;

public class Robot extends TimedRobot {
  private DriveTrain driveTrain;
  private XboxController controller;

  @Override
  public void robotInit() {
    Settings.initialize();
    driveTrain = new DriveTrain();
    controller = new XboxController(0);
  }

  @Override
  public void robotPeriodic() {
    driveTrain.periodic();
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}
  
  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    driveTrain.drive(
      // Linear Velocity (meters per second)
      controller.getLeftY() * sMaxLinearVelY.get(),
      controller.getLeftX() * sMaxLinearVelX.get(),
      // Angular Velocity (radians per second)
      controller.getRightX() * sMaxAngularVel.get() * 2 * Math.PI
    );
  }

  @Override
  public void teleopExit() {
    driveTrain.stopMotors();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
