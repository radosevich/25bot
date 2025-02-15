// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class aligntoReef extends Command {
  private final SwerveSubsystem m_swerve;
  private final double m_degrees;
  private final double m_speed;

  /**
   * Creates a new TurnDegrees. This command will turn your robot for a desired rotation (in
   * degrees) and rotational speed.
   *
   * @param speed The speed which the robot will drive. Negative is in reverse.
   * @param degrees Degrees to turn. Leverages encoders to compare distance.
   * @param drive The drive subsystem on which this command will run
   */
  public aligntoReef(double speed, double degrees, SwerveSubsystem swerve) {
    m_degrees = degrees;
    m_speed = speed;
    m_swerve = swerve;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set motors to stop, read encoder values for starting point
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerve.drive(m_swerve.getTargetSpeeds(0, 0, Rotation2d.fromDegrees(m_degrees)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.drive(m_swerve.getTargetSpeeds(0, 0, Rotation2d.fromDegrees(m_degrees)));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return true;
    
  }
}