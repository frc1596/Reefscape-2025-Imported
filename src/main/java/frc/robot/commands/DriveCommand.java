package frc.robot.commands;

import org.deceivers.drivers.LimelightHelpers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveCommand extends Command {
    public final SwerveSubsystem mSwerve;
    public XboxController mController;
    public CommandXboxController mController2;
    public Limelight mLimelight;

    private PIDController xController = new PIDController(0.03, 0.0, 0.0);
    private PIDController yController = new PIDController(0.03, 0.0, 0.0);
    private PIDController autoAimController = new PIDController(0.007,0,0);

    private LinearFilter aimFilter = LinearFilter.movingAverage(3);
    private LinearFilter xFilter2 = LinearFilter.movingAverage(3);
    private LinearFilter yFilter2 = LinearFilter.movingAverage(3);

    private JoystickHelper xHelper = new JoystickHelper(0);
    private JoystickHelper yHelper = new JoystickHelper(0);
    private JoystickHelper rotHelper = new JoystickHelper(0);

    private SlewRateLimiter xfilter = new SlewRateLimiter(3);
    private SlewRateLimiter yfilter = new SlewRateLimiter(3);

    private boolean lastScan;
    private double driveFactor = 1;

    public static boolean algaeTrack = false;

    public DriveCommand(SwerveSubsystem swerve, XboxController XboxController, CommandXboxController XboxController2, Limelight limelight) {
        mSwerve = swerve;
        mLimelight = limelight;
        mController2 = XboxController2;
        mController = XboxController;

        autoAimController.enableContinuousInput(-360, 360);

        addRequirements(mSwerve);
      }
       // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xVel = 0;
    double yVel = 0;
    double rotVel = 0;
    double driveDirection = 0;
    double driveMagnitude = 0;
    
    yVel = mController.getLeftY()*0.25;
    xVel = mController.getLeftX()*0.25;

    //Trigger Drive Method (comment out these 4 lines to go back to normal drive)
    driveDirection = Math.atan2(yVel, xVel);
    driveMagnitude = mController.getRightTriggerAxis();
    yVel = Math.sin(driveDirection) * driveMagnitude;
    xVel = Math.cos(driveDirection) * driveMagnitude;
    
//     if (ElevatorSubsystem.elevatorLevel >= 1){
//       driveFactor =- 0.02; 
//       driveFactor = Math.max(driveFactor, 0.1);
//     }
//     else{
//         driveFactor = 1.0;
//     }
// SmartDashboard.putNumber("Drive Factor", driveFactor);
// SmartDashboard.putNumber("Elejiojovatpr level", ElevatorSubsystem.elevatorLevel);

    //rumble when 20 seconds left
    if (DriverStation.isTeleop() && (DriverStation.getMatchTime() < 20.0) && (DriverStation.getMatchTime() > 19.0)) {
      mController.setRumble(RumbleType.kLeftRumble, 1);
      mController.setRumble(RumbleType.kRightRumble, 1);
    } else {
      mController.setRumble(RumbleType.kLeftRumble, 0);
      mController.setRumble(RumbleType.kRightRumble, 0);
    }
    rotVel = -mController.getRightX();

    yVel = yHelper.setInput(yVel).applyPower(2).value;
    xVel = xHelper.setInput(xVel).applyPower(2).value;
    rotVel = rotHelper.setInput(rotVel).applyPower(3).value;

    yVel = yVel * driveFactor;
    xVel = xVel * driveFactor;
    rotVel = rotVel * driveFactor;

    //SmartDashboard.putNumber("Tx", LimelightHelpers.getTX("limelight"));

    //double joystickMagnitude = Math.sqrt((mController.getRightY() * mController.getRightY()) + (mController.getRightX() * mController.getRightX()));
  
    if(algaeTrack){
      LimelightHelpers.setPipelineIndex("limelight", 1);
      if(LimelightHelpers.getTV("limelight")){
        rotVel = rotVel + -autoAimController.calculate(aimFilter.calculate(LimelightHelpers.getTX("limelight")),0);
      }
     }

     if(mController.getLeftBumper() && LimelightHelpers.getTV("limelight")){
      algaeTrack = false;

      LimelightHelpers.setPipelineIndex("limelight", 0);
        rotVel = -autoAimController.calculate(aimFilter.calculate(LimelightHelpers.getTX("limelight")),0);
        xVel = xController.calculate(xFilter2.calculate(LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getX()),0);
        yVel = yController.calculate(yFilter2.calculate(LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getY()),0);
     }
    //   if(mLimelight.getFid() == 4 || (mLimelight.getFid() == 7)){



    //rezero
    if (mController.getRawButton(7) & !lastScan) {
      mSwerve.resetGyro();
    }
    lastScan = mController.getRawButton(7);

    boolean fieldRelative = !mController.getAButton();
//ChassisSpeeds regularDrive = ChassisSpeeds.fromFieldRelativeSpeeds(xrVel, yrVel, joystickMagnitude, joystickAngle);
    mSwerve.drive(yfilter.calculate(yVel), xfilter.calculate(xVel), rotVel, fieldRelative);
     // mSwerve.driveFieldRelative(regularDrive);
    }

    // mDrivetrain.drive(yVel,xVel, rotVel, fieldRelative);
    // mDrivetrain.setModulesAngle(xVel);

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mSwerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  public class JoystickHelper {
    public double value;

    public JoystickHelper(double input){
        value = input;
    }

    public JoystickHelper applyDeadband(double deadband){
        if (Math.abs(value) < Math.abs(deadband)){
            value = 0;
        }

        return this;
    }

    public JoystickHelper applyPower(double power){
        value = Math.abs(Math.pow(value, power))*Math.signum(value);

        return this;
    }

    public JoystickHelper setInput(double input){
        value = input;
        return this;
    }

}

}
