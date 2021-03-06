// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ProfiledPIDAngleCommand;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the Robot periodic methods (other than the scheduler
 * calls). Instead, the structure of the robot (including subsystems, commands,
 * and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The robot's commands
  private final ProfiledPIDAngleCommand m_profiledPIDAngleCommand = new ProfiledPIDAngleCommand(m_robotDrive, 90);

  // The driver's controller
  XboxController m_driverController = new XboxController(Constants.OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure the drive command
    setDefaultDriveCommand();

    SmartDashboard.putData("Turn to Angle Command 1", m_profiledPIDAngleCommand);

  }
    // Configure default commands
    // Set the default drive command to split-stick arcade drive
  private void setDefaultDriveCommand() {
    if(Robot.isSimulation()){
      m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () ->
                m_robotDrive.arcadeDriveSim(
                    m_driverController.getY(GenericHID.Hand.kRight), //Axis[0]
                    -m_driverController.getX(GenericHID.Hand.kLeft)), //Axis[5]
            m_robotDrive));
    } else {
      m_robotDrive.setDefaultCommand(
        new RunCommand(
            () ->
                m_robotDrive.arcadeDrive(
                    -m_driverController.getY(GenericHID.Hand.kRight), //Axis[0]
                    m_driverController.getX(GenericHID.Hand.kLeft)), //Axis[5]
            m_robotDrive));
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive at half speed when the right bumper is held
    new JoystickButton(m_driverController, Button.kBumperRight.value)
        .whenPressed(() -> m_robotDrive.setMaxOutput(0.5))
        .whenReleased(() -> m_robotDrive.setMaxOutput(1));
  }

  public DriveSubsystem getRobotDrive() {
    return m_robotDrive;
  }

  /** Zeros the outputs of all subsystems. */
  public void zeroAllOutputs() {
    m_robotDrive.tankDriveVolts(0, 0);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_robotDrive.getRamseteCommand(m_robotDrive, m_robotDrive.getTrajFromFieldWidget("traj1", false));
  }
}
