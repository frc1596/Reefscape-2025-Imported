package frc.robot.subsystems;

import org.deceivers.swerve.SwerveDrive;
import org.deceivers.swerve.SwerveModuleV3;

import com.ctre.phoenix6.hardware.CANcoder;
//import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.path.PathPlannerTrajectory;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.SparkMax;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.wpilibj.interfaces.Gyro;
//import com.studica.frc.AHRS;

//import com.kauailabs.navx.frc.AHRS;

//import com.studica.frc.AHRS;
//import com.studica.frc.AHRS.NavXComType;

//import edu.wpi.first.wpilibj.interfaces.Gyro;

/**
 * Basic simulation of a swerve subsystem with the methods needed by PathPlanner
 */
public class SwerveSubsystem extends SubsystemBase {
  private final SparkMax mDriveMotor1 = new SparkMax(2, MotorType.kBrushless);
  private final SparkMax mDriveMotor2 = new SparkMax(4, MotorType.kBrushless);
  private final SparkMax mDriveMotor3 = new SparkMax(6, MotorType.kBrushless);
  private final SparkMax mDriveMotor4 = new SparkMax(8, MotorType.kBrushless);

  private final SparkMax mAzimuth1 = new SparkMax(1, MotorType.kBrushless);
  private final SparkMax mAzimuth2 = new SparkMax(3, MotorType.kBrushless);
  private final SparkMax mAzimuth3 = new SparkMax(5, MotorType.kBrushless);
  private final SparkMax mAzimuth4 = new SparkMax(7, MotorType.kBrushless);
  private final CANcoder mAzimuthEncoder1 = new CANcoder(10); //12 is front right
  private final CANcoder mAzimuthEncoder2 = new CANcoder(12); //11 is front left
  private final CANcoder mAzimuthEncoder3 = new CANcoder(9);  //10 is back right
  private final CANcoder mAzimuthEncoder4 = new CANcoder(11); //9 is back left

  private final SwerveModuleV3 Module1 = new SwerveModuleV3(mAzimuth1, mDriveMotor1, new Translation2d(0.29845, -0.29845), "Module 1", mAzimuthEncoder1);
  private final SwerveModuleV3 Module2 = new SwerveModuleV3(mAzimuth2, mDriveMotor2, new Translation2d(-0.29845, -0.29845), "Module 2", mAzimuthEncoder2);
  private final SwerveModuleV3 Module3 = new SwerveModuleV3(mAzimuth3, mDriveMotor3, new Translation2d(-0.29845, 0.29845), "Module 3", mAzimuthEncoder3);
  private final SwerveModuleV3 Module4 = new SwerveModuleV3(mAzimuth4, mDriveMotor4, new Translation2d(0.29845,  0.29845), "Module 4", mAzimuthEncoder4);

  private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  private final SwerveDrive mSwerveDrive = new SwerveDrive(this::getRotation, Module1, Module2, Module3, Module4);
  private SimSwerveModule[] modules;
  private SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry odometry;
  private RobotConfig config;

  //public final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  //private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  //private final AHRS gyro = new AHRS(SPI.Port.kMXP);


  //private Field2d field = new Field2d();
  
  public SwerveSubsystem() {
   gyro.reset();

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
    // Configure AutoBuilder last
    AutoBuilder.configure(
            mSwerveDrive::getPose, // Robot pose supplier
            mSwerveDrive::resetPosePathplanner, // Method to reset odometry (will be called if your auto has a starting pose)
            mSwerveDrive::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> mSwerveDrive.drivePathplanner(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(3.8, 0.0, 0), // Translation PID constants
                    new PIDConstants(3.5, 0.0, 0) // Rotation PID constants
            ),config,
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
  );
  }

  @Override
  public void periodic() {
   // odometry.update(gyro.getRotation2d(), getPositions());
   // mSwerveDrive.updateOdometry();
  //SmartDashboard.putNumber("RobotXSpeed", mSwerveDrive.getChassisSpeeds().vxMetersPerSecond);
  //SmartDashboard.putNumber("RobotYSpeed", mSwerveDrive.getChassisSpeeds().vyMetersPerSecond);
  //SmartDashboard.putNumber("RobotXPos", mSwerveDrive.getPose().getX());
  //SmartDashboard.putNumber("RobotYPos", mSwerveDrive.getPose().getY());

  mSwerveDrive.periodic(); 
  }
  public void drive(double forward, double strafe, double azimuth, boolean fieldRelative){
      azimuth = azimuth*2.5;
      mSwerveDrive.drive(forward, strafe, azimuth, fieldRelative);
    }
  
    public void driveClosedLoop(double forward, double strafe, double azimuth, boolean fieldRelative){
      if (!fieldRelative){
        forward = -forward;
        strafe = -strafe;
      }
      azimuth = azimuth*2.5;
      mSwerveDrive.driveClosedLoop(forward, strafe, azimuth, fieldRelative);
    }
  
    public void stop(){
      mSwerveDrive.stop();
    }
  
    // public void followPath(double initTime, PathPlannerTrajectory pptrajectory, boolean useLimelight){
    // //  mSwerveDrive.followPath(initTime, pptrajectory, useLimelight);
    // }
  
    public void setLocation(double x, double y, double angle){
      mSwerveDrive.setLocation(x, y, angle);
    }

    public void setModulesAngle(double angle, int module){
     // mSwerveDrive.setModulesAngle(angle, module);
    }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }
  public double getRotation() {
    //SmartDashboard.putNumber("Gyro", -gyro.getAngle());
    return -gyro.getAngle();
    //return 0;
 }
  public void resetPose(Pose2d pose) {
    odometry.resetPosition(new Rotation2d(gyro.getYaw()), getPositions(), pose);
  }

  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, new Rotation2d(gyro.getYaw())));
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    //ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    ChassisSpeeds targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(robotRelativeSpeeds, new Rotation2d(gyro.getYaw()));

    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
    setStates(targetStates);
  }

  public void resetGyro(){
    gyro.reset();
  }

  public void setStates(SwerveModuleState[] targetStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, 3.81);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setTargetState(targetStates[i]);
      
    }
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  /**
   * Basic simulation of a swerve module, will just hold its current state and not use any hardware
   */
  class SimSwerveModule {
    private SwerveModulePosition currentPosition = new SwerveModulePosition();
    private SwerveModuleState currentState = new SwerveModuleState();

    public SwerveModulePosition getPosition() {
      return currentPosition;
    }

    public SwerveModuleState getState() {
      return currentState;
    }

    public void setTargetState(SwerveModuleState targetState) {
      // Optimize the state
      currentState = SwerveModuleState.optimize(targetState, currentState.angle);

      currentPosition = new SwerveModulePosition(currentPosition.distanceMeters + (currentState.speedMetersPerSecond * 0.02), currentState.angle);
    }
  }


}