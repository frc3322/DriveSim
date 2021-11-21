// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class ProfiledPIDAngleCommand extends CommandBase {
  TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(100, 2000);
  
  ProfiledPIDController m_controller = new ProfiledPIDController(
    Constants.DriveConstants.kPAngle, 
    0, 
    Constants.DriveConstants.kDAngle, 
    m_constraints
  );

  double kS = 0.1;
  double kV = 0;

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV);

  DriveSubsystem robotDrive;
  double angleGoal;
  TrapezoidProfile.State goal;

  /** Creates a new ProfiledPIDAngleCommand. */
  public ProfiledPIDAngleCommand(DriveSubsystem robotDrive, double angleGoal) {
    this.robotDrive = robotDrive;
    this.angleGoal = angleGoal;
    goal = new TrapezoidProfile.State(angleGoal, 0);
    addRequirements(robotDrive);

    SmartDashboard.putNumber("kS", kS);
    SmartDashboard.putNumber("kV", kV);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kS = SmartDashboard.getNumber("kS", kS);
    kV = SmartDashboard.getNumber("kV", kV);
    m_controller.setP(SmartDashboard.getNumber("kPAngle", Constants.DriveConstants.kPAngle));
    m_controller.setD(SmartDashboard.getNumber("kDAngle", Constants.DriveConstants.kDAngle));
    m_controller.enableContinuousInput(-180, 180);
    m_controller.setTolerance(2);
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double feedfowardOutput = feedforward.calculate(m_controller.getSetpoint().velocity);
    double PIDOutput = m_controller.calculate(robotDrive.getHeading(), goal);
    double output = feedfowardOutput + PIDOutput;

    SmartDashboard.putNumber("AngleCommandTest/heading", robotDrive.getHeading());
    SmartDashboard.putNumber("AngleCommandTest/goal", angleGoal);
    SmartDashboard.putNumber("AngleCommandTest/feedforwardOutput", feedfowardOutput);
    SmartDashboard.putNumber("AngleCommandTest/PIDOutput", PIDOutput);
    SmartDashboard.putNumber("AngleCommandTest/Output", output);

    if(RobotBase.isSimulation()){
      robotDrive.arcadeDriveSim(0, feedfowardOutput);
    } else {
      robotDrive.arcadeDrive(0, feedfowardOutput);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_controller.reset(robotDrive.getHeading(), robotDrive.getAngularVelocity());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_controller.atGoal();
  }
}
