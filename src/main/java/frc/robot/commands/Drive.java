/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

/**
 * An example command that uses an example subsystem.
 */
public class Drive extends CommandBase {
  private final DriveTrain m_drivetrain;
  private final DoubleSupplier m_left;
  private final DoubleSupplier m_right;
  private final DoubleSupplier m_elevator;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Drive(DoubleSupplier left, DoubleSupplier right, DoubleSupplier elevator, DriveTrain drivetrain) {
    m_drivetrain = drivetrain;
    m_left = left;
    m_right = right;
    m_elevator = elevator;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.drive(m_left.getAsDouble(), m_right.getAsDouble());
    m_drivetrain.elevator(m_elevator.getAsDouble());
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
