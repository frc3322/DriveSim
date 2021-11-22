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
  TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(500, 1000);
  
  ProfiledPIDController m_controller = new ProfiledPIDController(
    Constants.DriveConstants.kPAngle, 
    0, 
    Constants.DriveConstants.kDAngle, 
    m_constraints
  );

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveConstants.ksAngle, DriveConstants.kvAngle, DriveConstants.kaAngle);

  DriveSubsystem robotDrive;
  double angleGoal;
  TrapezoidProfile.State goal;

  double lastTargetLeftWheelSpeed;
  double lastTargetRightWheelSpeed;
  double lastTargetAngularSpeed;

  /** Creates a new ProfiledPIDAngleCommand. */
  public ProfiledPIDAngleCommand(DriveSubsystem robotDrive, double angleGoal) {
    this.robotDrive = robotDrive;
    this.angleGoal = angleGoal;
    goal = new TrapezoidProfile.State(angleGoal, 0);
    addRequirements(robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feedforward = new SimpleMotorFeedforward(
      SmartDashboard.getNumber("kSAngle", DriveConstants.ksAngle), 
      SmartDashboard.getNumber("kVAngle", DriveConstants.kvAngle), 
      SmartDashboard.getNumber("kAAngle", DriveConstants.kaAngle));

    m_controller.setP(SmartDashboard.getNumber("kPAngle", Constants.DriveConstants.kPAngle));
    m_controller.setD(SmartDashboard.getNumber("kDAngle", Constants.DriveConstants.kDAngle));
    if(!RobotBase.isSimulation()){
      m_controller.enableContinuousInput(-180, 180);
    }

    m_controller.setTolerance(1);

    m_controller.reset(new TrapezoidProfile.State(robotDrive.getHeading(), robotDrive.getAngularVelocity()));
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double heading = robotDrive.getHeading();

    DifferentialDriveWheelSpeeds targetShaftSpeeds = 
      Constants.DriveConstants.kDriveKinematics.toWheelSpeeds(
        new ChassisSpeeds(0, 0, Math.toRadians(m_controller.getSetpoint().velocity)));

    double targetLeftWheelSpeed = targetShaftSpeeds.leftMetersPerSecond * DriveConstants.kDriveGearing * Math.PI * DriveConstants.kWheelDiameterMeters;
    double targetRightWheelSpeed = targetShaftSpeeds.rightMetersPerSecond * DriveConstants.kDriveGearing * Math.PI * DriveConstants.kWheelDiameterMeters;
    double targetLeftWheelAccel = (targetLeftWheelSpeed - lastTargetLeftWheelSpeed) / 0.02;
    double targetRightWheelAccel = (targetRightWheelSpeed - lastTargetRightWheelSpeed) / 0.02;
    lastTargetLeftWheelSpeed = targetLeftWheelSpeed;
    lastTargetRightWheelSpeed = targetRightWheelSpeed;
    
    double targetAngularAccel = (m_controller.getSetpoint().velocity - lastTargetAngularSpeed) / 0.02;
    lastTargetAngularSpeed = m_controller.getSetpoint().velocity;

    DifferentialDriveWheelSpeeds PIDWheelSpeeds =
      Constants.DriveConstants.kDriveKinematics.toWheelSpeeds(
        new ChassisSpeeds(0, 0, Math.toRadians(m_controller.calculate(heading, goal))));

    double ffLeft = feedforward.calculate(targetLeftWheelSpeed, targetLeftWheelAccel);
    double ffRight = feedforward.calculate(targetRightWheelSpeed, targetRightWheelAccel);
    double PIDLeft = PIDWheelSpeeds.leftMetersPerSecond;
    double PIDRight = PIDWheelSpeeds.rightMetersPerSecond;

    SmartDashboard.putNumber("AngleCommandTest/heading", heading);
    SmartDashboard.putNumber("AngleCommandTest/goal", angleGoal);
    SmartDashboard.putNumber("AngleCommandTest/leftTargetSpeed", targetLeftWheelSpeed);
    SmartDashboard.putNumber("AngleCommandTest/rightTargetSpeed", targetRightWheelSpeed);
    SmartDashboard.putNumber("AngleCommandTest/angularVelSetpoint", m_controller.getSetpoint().velocity);
    SmartDashboard.putNumber("AngleCommandTest/angularPosSetpoint", m_controller.getSetpoint().position);
    SmartDashboard.putNumber("AngleCommandTest/angularAccelSetpoint", targetAngularAccel);
    SmartDashboard.putNumber("AngleCommandTest/leftPID", PIDLeft);
    SmartDashboard.putNumber("AngleCommandTest/rightPID", PIDRight);
    SmartDashboard.putNumber("AngleCommandTest/ffLeft", ffLeft);
    SmartDashboard.putNumber("AngleCommandTest/ffRight", ffRight);
    SmartDashboard.putNumber("AngleCommandTest/leftOutput", ffLeft+PIDLeft);
    SmartDashboard.putNumber("AngleCommandTest/rightOutput", ffRight + PIDRight);

    robotDrive.tankDriveVolts(ffLeft+PIDLeft, ffRight+PIDRight);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_controller.reset(new TrapezoidProfile.State(robotDrive.getHeading(), robotDrive.getAngularVelocity()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_controller.atGoal();
  }
}
