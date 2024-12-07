// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pound;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
  double maximumSpeed = 4.0;

  SwerveDrive swerveDrive;

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    try
    {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }  

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
  }

   public void setupPathplanner(){

    //If you go to path planner docs,  you can pull from GUI which is much easier. I do it this way to prevent errors parsing, as it is still in beta
    //Eventually, we can also make a json parser for this
    double driveMotorGearing = 4.621;
    double kTrackWidth = 0.5969;
    double kWheelBase = 0.5969;
    Translation2d[] moduleOffsets = 
    new Translation2d[] {
      new Translation2d(kTrackWidth/2.0, kWheelBase/2.0),
      new Translation2d(-kTrackWidth/2.0, kWheelBase/2.0),
      new Translation2d(kTrackWidth/2.0, -kWheelBase/2.0),
      new Translation2d(-kTrackWidth/2.0, -kWheelBase/2.0)
    };

    ModuleConfig moduleConfig = new ModuleConfig(
      Inches.of(1.5), 
      MetersPerSecond.of(5.1), 
      1.19, 
      DCMotor.getNeoVortex(1).withReduction(driveMotorGearing), 
      Amps.of(50), 
      1);

    RobotConfig robotConfig = new RobotConfig(
      Pound.of(110), 
      KilogramSquareMeters.of(18), 
      moduleConfig,
      moduleOffsets);

    PPHolonomicDriveController ppHolonomicDriveController = new PPHolonomicDriveController(
            new PIDConstants(2.5*0), 
            new PIDConstants(0.5*0));

    BooleanSupplier shouldFlipPath = () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followe
        // d to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    };

    AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        this::getChassisSpeeds, 
        this::pathPlannerSetModuleStates, 
        ppHolonomicDriveController, 
        robotConfig,
        shouldFlipPath, 
        this
    );

    //Found it in YAGSL docs, don't know usefulness
    PathfindingCommand.warmupCommand().schedule();
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

    /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumVelocity(),
                                          translationY.getAsDouble() * swerveDrive.getMaximumVelocity()),
                        angularRotationX.getAsDouble() * swerveDrive.getMaximumAngularVelocity(),
                        true,
                        false);
    });
  }

  public Rotation2d getHeading(){
    return swerveDrive.getGyro().getRotation3d().toRotation2d();
  }

  public Pose2d getPose(){
    return swerveDrive.getPose();
  }

  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public ChassisSpeeds getChassisSpeeds()
  {
    return swerveDrive.getRobotVelocity();
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  private void pathPlannerSetModuleStates(ChassisSpeeds chassisSpeeds, DriveFeedforwards feedforwards){
    swerveDrive.drive(chassisSpeeds,swerveDrive.kinematics.toSwerveModuleStates(chassisSpeeds), feedforwards.linearForces());
  }


  //INVESTIGATE FURTHER INTO LIBRARY TO FIND EFFECT (does not apply to certain desired state overloads? how work? how accurate is wpilib simulation model?)
  /**
   * Replaces the swerve module feedforward with a new SimpleMotorFeedforward object.
   *
   * @param kS the static gain of the feedforward
   * @param kV the velocity gain of the feedforward
   * @param kA the acceleration gain of the feedforward
   */
  public void replaceSwerveModuleFeedforward(double kS, double kV, double kA)
  {
    swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
  }
}
