/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

/**
 * An example command that uses an example subsystem.
 */
public class Drive extends CommandBase {
  private final DriveTrain m_drivetrain;
  private final DoubleSupplier m_left;
  private final DoubleSupplier m_right;
  private final DoubleSupplier m_elevatorDown;
  private final DoubleSupplier m_elevatorUp;
  private final double m_elevator;

  private double lastSpeed = 0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Drive(DoubleSupplier left, DoubleSupplier right, DoubleSupplier elevatorDown, DoubleSupplier elevatorUp, DriveTrain drivetrain) {
    m_drivetrain = drivetrain;
    m_left = left;
    m_right = right;
    m_elevatorDown = elevatorDown;
    m_elevatorUp = elevatorUp;
    m_elevator = elevatorUp.getAsDouble() - elevatorDown.getAsDouble();
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double thisSpeed = m_left.getAsDouble();

    // if (Math.abs(thisSpeed - lastSpeed) > Constants.speedRate) {
    //   if (thisSpeed > lastSpeed) {
    //     lastSpeed += Constants.speedRate;
    //   } else {
    //     lastSpeed -= Constants.speedRate;
    //   }
    //   m_drivetrain.drive(lastSpeed, m_right.getAsDouble());
    // } else {
    //   m_drivetrain.drive(thisSpeed, m_right.getAsDouble());
    // }
    m_drivetrain.drive(thisSpeed, m_right.getAsDouble());
    // m_drivetrain.drive(m_left.getAsDouble(), m_elevator);

    double elevatorSpeedCap = 1;
    m_drivetrain.elevator((m_elevatorUp.getAsDouble() - m_elevatorDown.getAsDouble()) * elevatorSpeedCap);
    //m_drivetrain.drive(1, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0);
    m_drivetrain.elevator(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
