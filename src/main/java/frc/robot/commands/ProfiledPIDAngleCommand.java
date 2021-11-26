// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class ProfiledPIDAngleCommand extends CommandBase {
  TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(Math.PI, 2*Math.PI);
  
  ProfiledPIDController m_controller = new ProfiledPIDController(
    Constants.DriveConstants.kPAngle, 
    0, 
    Constants.DriveConstants.kDAngle, 
    m_constraints
  );

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
    0,
    DriveConstants.kvVoltSecondsPerRadian, 
    DriveConstants.kaVoltSecondsSquaredPerRadian);

  DriveSubsystem robotDrive;
  double angleGoalRadians;
  TrapezoidProfile.State goal;

  double lastTargetAngularSpeed;

  /** Creates a new ProfiledPIDAngleCommand. */
  public ProfiledPIDAngleCommand(DriveSubsystem robotDrive, double angleGoal) {
    this.robotDrive = robotDrive;
    angleGoalRadians = Math.toRadians(angleGoal);
    goal = new TrapezoidProfile.State(angleGoalRadians, 0);
    addRequirements(robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_controller.setP(SmartDashboard.getNumber("kPAngle", Constants.DriveConstants.kPAngle));
    m_controller.setD(SmartDashboard.getNumber("kDAngle", Constants.DriveConstants.kDAngle));

    m_controller.reset(new TrapezoidProfile.State(Math.toRadians(robotDrive.getHeading()), Math.toRadians(robotDrive.getAngularVelocity())));
    m_controller.setGoal(goal);
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double headingRadians = Math.toRadians(robotDrive.getHeading());
    double controllerOutput = m_controller.calculate(headingRadians);

    DifferentialDriveWheelSpeeds PID = DriveConstants.kDriveKinematics.toWheelSpeeds(new ChassisSpeeds(0, 0, controllerOutput));
    
    double targetAngularAccel = (m_controller.getSetpoint().velocity - lastTargetAngularSpeed) / 0.02;
    lastTargetAngularSpeed = m_controller.getSetpoint().velocity;

    double ff = feedforward.calculate(m_controller.getSetpoint().velocity, targetAngularAccel) / (2 / DriveConstants.kTrackwidthMeters);

    SmartDashboard.putNumber("AngleCommandTest/goal", angleGoalRadians);
    SmartDashboard.putNumber("AngleCommandTest/output", controllerOutput);
    SmartDashboard.putNumber("AngleCommandTest/angularVelSetpoint", m_controller.getSetpoint().velocity);
    SmartDashboard.putNumber("AngleCommandTest/angularPosSetpoint", m_controller.getSetpoint().position);
    SmartDashboard.putNumber("AngleCommandTest/angularAccelSetpoint", targetAngularAccel);
    SmartDashboard.putNumber("AngleCommandTest/ff", ff);

    robotDrive.tankDriveVolts(-ff + PID.leftMetersPerSecond, ff + PID.rightMetersPerSecond);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;//m_controller.atGoal();
  }
}
