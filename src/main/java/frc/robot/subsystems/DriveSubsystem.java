// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.CANEncoderSim;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;

public class DriveSubsystem extends SubsystemBase {
  private final CANSparkMax leftLeader = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
  private final CANSparkMax leftFollower = new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless);
  private final CANSparkMax rightLeader = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);
  private final CANSparkMax rightFollower = new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(leftLeader, rightLeader);

  // The left-side drive encoder
  private final CANEncoder m_leftEncoder = leftLeader.getEncoder();

  // The right-side drive encoder
  private final CANEncoder m_rightEncoder = rightLeader.getEncoder();

  // The gyro sensor
  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  // These classes help us simulate our drivetrain
  public DifferentialDrivetrainSim m_drivetrainSimulator;
  private CANEncoderSim m_leftEncoderSim;
  private CANEncoderSim m_rightEncoderSim;
  // The Field2d class shows the field in the sim GUI
  private Field2d m_fieldSim;
  private ADXRS450_GyroSim m_gyroSim;

  private double rightVoltage;
  private double leftVoltage;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    leftVoltage = 0;
    rightVoltage = 0;

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    if (RobotBase.isSimulation()) { // If our robot is simulated
      // This class simulates our drivetrain's motion around the field.
      m_drivetrainSimulator =
          new DifferentialDrivetrainSim(
              DriveConstants.kDrivetrainPlant,
              DriveConstants.kDriveGearbox,
              DriveConstants.kDriveGearing,
              DriveConstants.kTrackwidthMeters,
              DriveConstants.kWheelDiameterMeters / 2.0,
              null
              /*VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005)*/);

      // The encoder and gyro angle sims let us set simulated sensor readings
      m_leftEncoderSim = new CANEncoderSim(false, DriveConstants.kLeftMotor1Port);
      m_rightEncoderSim = new CANEncoderSim(false, DriveConstants.kRightMotor1Port);
      m_gyroSim = new ADXRS450_GyroSim(m_gyro);

      // the Field2d class lets us visualize our robot in the simulation GUI.
      m_fieldSim = new Field2d();
      SmartDashboard.putData("Field", m_fieldSim);
    }
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(getHeading()),
        m_leftEncoder.getPosition(),
        m_rightEncoder.getPosition());
    m_fieldSim.setRobotPose(getPose());
  }

  @Override
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the simulation,
    // and write the simulated positions and velocities to our simulated encoder and gyro.
    // We negate the right side so that positive voltages make the right side
    // move forward.
    m_drivetrainSimulator.setInputs(leftVoltage, rightVoltage);
        // leftLeader.get() * RobotController.getBatteryVoltage(),
        // -rightLeader.get() * RobotController.getBatteryVoltage());
    m_drivetrainSimulator.update(0.020);

    m_leftEncoderSim.setPosition(m_drivetrainSimulator.getLeftPositionMeters());
    m_leftEncoderSim.setVelocity(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setPosition(m_drivetrainSimulator.getRightPositionMeters());
    m_rightEncoderSim.setVelocity(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
  }

  /**
   * Returns the current being drawn by the drivetrain. This works in SIMULATION ONLY! If you want
   * it to work elsewhere, use the code in {@link DifferentialDrivetrainSim#getCurrentDrawAmps()}
   *
   * @return The drawn current in Amps.
   */
  public double getDrawnCurrentAmps() {
    return m_drivetrainSimulator.getCurrentDrawAmps();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_drivetrainSimulator.setPose(pose);
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    var batteryVoltage = RobotController.getBatteryVoltage();
    if (Math.max(Math.abs(leftVolts), Math.abs(rightVolts)) > batteryVoltage) {
      leftVolts *= batteryVoltage / 12.0;
      rightVolts *= batteryVoltage / 12.0;
    }
    leftLeader.setVoltage(leftVolts);
    rightLeader.setVoltage(-rightVolts);

    leftVoltage = leftVolts;
    rightVoltage = rightVolts;
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public CANEncoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public CANEncoder getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public TrajectoryConfig getTrajConfig(boolean reversed) {
    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            Constants.DriveConstants.ksVolts,
            Constants.DriveConstants.kvVoltSecondsPerMeter,
            Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
        Constants.DriveConstants.kDriveKinematics,
        7);
  
    // Create config for trajectory
    TrajectoryConfig config =
      new TrajectoryConfig(
        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(Constants.DriveConstants.kDriveKinematics)
          // Apply the voltage constraint
          .addConstraint(autoVoltageConstraint)
          .setReversed(reversed);

    return config;
  }

  public Trajectory getTraj() {
    Trajectory exampleTrajectory =
    TrajectoryGenerator.generateTrajectory(
        // Start at (1, 2) facing the +X direction
        new Pose2d(1, 2, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(2, 3), new Translation2d(3, 1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(4, 2, new Rotation2d(0)),
        // Pass config
        getTrajConfig(false));

    return exampleTrajectory;
  }

  public void putTrajToFieldWidget(){
    m_fieldSim.getObject("traj1").setPoses(
      /*new Pose2d(0.0, 0.0, new Rotation2d(0)),
      new Pose2d(1.0, 1.0, new Rotation2d(0))*/
    );
    m_fieldSim.getObject("traj2").setPoses();
    m_fieldSim.getObject("traj3").setPoses();
  }

  public Trajectory getTrajFromFieldWidget(String traj, boolean reversed){
    return TrajectoryGenerator.generateTrajectory(m_fieldSim.getObject(traj).getPoses(), getTrajConfig(reversed));
  }
  
  public Command getRamseteCommand(DriveSubsystem robotDrive, String traj, boolean reversed) {
    RamseteCommand ramseteCommand =
      new RamseteCommand(
          getTrajFromFieldWidget(traj, reversed),
          robotDrive::getPose,
          new RamseteController(
              Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
          new SimpleMotorFeedforward(
              Constants.DriveConstants.ksVolts,
              Constants.DriveConstants.kvVoltSecondsPerMeter,
              Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
          Constants.DriveConstants.kDriveKinematics,
          robotDrive::getWheelSpeeds,
          new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
          new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
          // RamseteCommand passes volts to the callback
          robotDrive::tankDriveVolts,
          robotDrive);
  
    // Reset odometry to starting pose of trajectory.
    robotDrive.resetOdometry(getTrajFromFieldWidget(traj, reversed).getInitialPose());
    
    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> robotDrive.tankDriveVolts(0, 0));
  }  

}



