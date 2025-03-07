// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final IntakePivotSubsystem intakePivotSubsystem = new IntakePivotSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  // private final ShooterPivotSubsystem shooter = new ShooterPivotSubsystem();
  XboxController driverController = new XboxController(0);
  CommandXboxController operatorController = new CommandXboxController(1);
AddressableLED m_led = new AddressableLED(9);
AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(96);
  // private final Compressor mCompressor = new Compressor(PneumaticsModuleType.CTREPCM);
  // private final DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1,0 );
  // Limelight limelight = new Limelight();
  // PowerDistribution examplePD = new PowerDistribution();

  // LED led = new LED();



  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  SendableChooser<Command> autoChooser = new SendableChooser<>();
  //private final SendableChooser<Command> autoChooser;

  @Override
  public void robotInit() {

    configureButtonBindings();
m_led.setLength(96);
m_led.start();

    NamedCommands.registerCommand("Outtake", intake.runIntakes(0.5));
    NamedCommands.registerCommand("Intake", intake.reverseIntakes(-0.5));
    NamedCommands.registerCommand("Stop Intake", intake.runIntakes(0));

     NamedCommands.registerCommand("Elevator Down", elevator.elevatorDown());
     NamedCommands.registerCommand("Elevator L1", elevator.elevatorUp(1));
     NamedCommands.registerCommand("ElevatorL2", elevator.elevatorUp(2));
     NamedCommands.registerCommand("ElevatorL3", elevator.elevatorUp(3));
     NamedCommands.registerCommand("ElevatorL4", elevator.elevatorUp(4));

     NamedCommands.registerCommand("Pivot to Station", intakePivotSubsystem.intakePivot(-1));
     NamedCommands.registerCommand("Pivot to L2L3", intakePivotSubsystem.intakePivot(0));
     NamedCommands.registerCommand("Pivot to L1", intakePivotSubsystem.intakePivot(-1));

    // NamedCommands.registerCommand("index", intake.putThatThingBackWhereItCameFromOrSoHelpMe());
    // NamedCommands.registerCommand("Intake Down", intake.intakeDown());
    NamedCommands.registerCommand("Intake up", elevator.elevatorUp(4));


    // NamedCommands.registerCommand("shooterAim", shooter.shooterAim());
    // NamedCommands.registerCommand("shooterIntake", shooter.shooterindex());
    // NamedCommands.registerCommand("shooterShoot", shooter.shooterGo());
    // NamedCommands.registerCommand("shooterprime", shooter.primeShooter());
    // NamedCommands.registerCommand("shooterprime2", shooter.primeShooter2());
    // NamedCommands.registerCommand("LookAtTheSpeakerDumbRobot", shooter.setShooterWithLimelight());
    // NamedCommands.registerCommand("shooterprime", shooter.shooterlimelightindex());
    // NamedCommands.registerCommand("NotShootShoot", shooter.shooterNotGo());

    autoChooser = AutoBuilder.buildAutoChooser();

    UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setResolution(160,120);

    //camera.setFPS(24);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putString("Selected Auto:" , autoChooser.getSelected().getName());

    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    // led.isEnabled = false;
    //led.setLEDGreen();
    for(int i = 0; i < 96; i++){
      m_ledBuffer.setRGB(i,0, 0, 0); //grb
    }
    m_led.setData(m_ledBuffer);
  }

  @Override
  public void disabledPeriodic() {
    //SmartDashboard.putNumber("FID",  limelight.getFid());
    // SmartDashboard.putNumber("Intake Current", examplePD.getCurrent(2));

  }

  
  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    m_autonomousCommand = getAutonomousCommand();

   // swerve.setLocation(0,0,0);
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove` 
    // this line or comment it out.

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
       if(alliance.get() == DriverStation.Alliance.Red){
        for(int i = 0; i < 96; i++){
          m_ledBuffer.setRGB(i,0, 100, 0); //grb
        }
        m_led.setData(m_ledBuffer);
       } else{
        for(int i = 0; i < 96; i++){
          m_ledBuffer.setRGB(i,0, 0, 100); //grb
        }
        m_led.setData(m_ledBuffer);
       };
      
    }


    swerve.setDefaultCommand(new DriveCommand(swerve, driverController, operatorController));

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }


  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
        // SmartDashboard.putNumber("Intake Current", examplePD.getCurrent(2));

    //if last half second, put pneumatics down
    // if (DriverStation.isTeleop() && (DriverStation.getMatchTime() < 0.6)){      
    //    m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    //  }

    // if (driverController.getYButtonPressed()) {
    //   m_doubleSolenoid.toggle();
    //   led.climbTime = true;
    // } 
    // if(intake.getSensor()){
    //   led.intakeNote = true;
    // } else if(led.flashDone == true){
    //   led.intakeNote = false;
    //   led.flashDone = false;
    // }

    
    // if (DriverStation.isTeleop() && (DriverStation.getMatchTime() < 40.0) && (DriverStation.getMatchTime() > 0.1)) {
    //   mCompressor.enableDigital();
    // } else {
    //   mCompressor.disable();
    // }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    // mCompressor.enableDigital();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  public Command getAutonomousCommand() {
    if(autoChooser.getSelected().getName() == "Wow Auto"){
      swerve.setLocation(2, 7, 0);
    }

    return autoChooser.getSelected();
  }

  public void configureButtonBindings(){
    // Triggers 
    // placing coral 
    Trigger elevatorLevelFour = operatorController.y();
    Trigger elevatorLevelThree = operatorController.x().and(operatorController.povRight());
    Trigger elevatorLevelTwo = operatorController.b().and(operatorController.povRight());
    Trigger elevatorLevelOne = operatorController.a().and(operatorController.povRight());
    Trigger elevatorDown = operatorController.povDown();

    Trigger humanStation = operatorController.povUp();


    //intake and out 
    Trigger intakeIn = operatorController.rightBumper().and(operatorController.povRight());
    Trigger intakeOut = operatorController.leftBumper().and(operatorController.povRight());

    Trigger intakeAlgaeIn = operatorController.rightBumper().and(operatorController.povLeft());
    Trigger intakeAlgaeOut = operatorController.leftBumper().and(operatorController.povLeft());

    // harvesting algae 
    Trigger algaeGroundIntake = operatorController.povLeft().and(operatorController.a()); 
    Trigger algaeLevelOneIntake =  operatorController.povLeft().and(operatorController.b()); 
    Trigger algaeLevelTwoIntake = operatorController.povLeft().and(operatorController.x());

    //+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=

    // Bindings 
    //coral 
    elevatorLevelFour.whileTrue(elevator.elevatorUp(4).alongWith(intakePivotSubsystem.intakePivot(5)));
    elevatorLevelThree.whileTrue(elevator.elevatorUp(3).alongWith(intakePivotSubsystem.intakePivot(5)));
    elevatorLevelTwo.whileTrue(elevator.elevatorUp(2).alongWith(intakePivotSubsystem.intakePivot(5)));
    elevatorLevelOne.whileTrue(elevator.elevatorUp(1).alongWith(intakePivotSubsystem.intakePivot(0)));
    elevatorDown.whileTrue(elevator.elevatorDown().alongWith(intakePivotSubsystem.intakePivot(0)));

    humanStation.whileTrue(elevator.elevatorUp(5).alongWith(intakePivotSubsystem.intakePivot(-1)));
    // elevatorLevelFour.whileTrue(intakePivotSubsystem.intakePivot(-1));
    // elevatorLevelThree.whileTrue(intakePivotSubsystem.intakePivot(0));
    // elevatorLevelTwo.whileTrue(intakePivotSubsystem.intakePivot(3));
    // elevatorLevelOne.whileTrue(intakePivotSubsystem.intakePivot(-3));
    // algae 
    algaeGroundIntake.whileTrue(elevator.elevatorDown().andThen(intakePivotSubsystem.intakePivot(-4)));
    algaeLevelOneIntake.whileTrue(elevator.elevatorUp(1).andThen(intakePivotSubsystem.intakePivot(2)));
    algaeLevelTwoIntake.whileTrue(elevator.elevatorUp(2).andThen(intakePivotSubsystem.intakePivot(3)));


    // intake and out 
     intakeIn.whileTrue(intake.runIntakes(0.10));
     intakeOut.whileTrue(intake.runIntakes(-0.10));

    intakeAlgaeIn.whileTrue(intake.reverseIntakes(0.6));
    intakeAlgaeOut.whileTrue(intake.runIntakes(-0.6));

  }

}

