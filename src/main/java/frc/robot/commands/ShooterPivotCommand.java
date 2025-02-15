// package frc.robot.commands;

// import frc.robot.subsystems.Limelight;
// import frc.robot.subsystems.ShooterPivotSubsystem;
// import frc.robot.util.InterpolatingDouble;
// import frc.robot.util.InterpolatingTreeMap;

// import org.deceivers.drivers.LimelightHelpers;

// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;

// /** An example command that uses an example subsystem. */
// public class ShooterPivotCommand extends Command {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//   private final ShooterPivotSubsystem m_subsystem;
//   private final XboxController m_Controller;

//   private int shootState = 1;
//   private static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> hoodMap = new InterpolatingTreeMap<>();
//   static {
//     hoodMap.put(new InterpolatingDouble(7.0), new InterpolatingDouble(-.21));
//     hoodMap.put(new InterpolatingDouble(4.16), new InterpolatingDouble(-.193));
//     hoodMap.put(new InterpolatingDouble(3.0), new InterpolatingDouble(-.167));
//     hoodMap.put(new InterpolatingDouble(2.0), new InterpolatingDouble(-0.15));
//     hoodMap.put(new InterpolatingDouble(1.0), new InterpolatingDouble(-0.09)); 
//     hoodMap.put(new InterpolatingDouble(0.5), new InterpolatingDouble(-0.09));
//   }
//   /**
//    * Creates a new ExampleCommand.
//    *
//    * @param subsystem The subsystem used by this command.
//    */
//   public ShooterPivotCommand(ShooterPivotSubsystem subsystem, XboxController controller, Limelight limelight) {
//     m_subsystem = subsystem;
//     m_Controller = controller;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(subsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {

//     if((m_Controller.getPOV() <= 315) && (m_Controller.getPOV() >= 225)){
//       m_subsystem.setPvalue(5.0);
//       m_subsystem.setAngleSlow(-0.34); //slowed down for amp

//       if(m_subsystem.getAngle() < -0.28){
//         m_subsystem.shooterIntake(1.0);
//         m_subsystem.shooterShoot(-1.0);
//       }
//   } else if((m_Controller.getPOV() <= 135) && (m_Controller.getPOV() >= 45)){
//          m_subsystem.setPvalue(5.5);
//     m_subsystem.setAngle(-0.117); //-0.2
//     m_subsystem.shooterIntake(.4);
//   }
//    else if(m_Controller.getRightBumper()){
//     m_subsystem.setAngle(-0.2);

//   }

//   else if(m_Controller.getLeftBumper()){
         
//         SmartDashboard.putNumber("LimelightZ", LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getZ());

//         double outputAngle = hoodMap.getInterpolated(new InterpolatingDouble(LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getZ())).value;
//      m_subsystem.setPvalue(5.5);

//       m_subsystem.setAngle(outputAngle); //-0.28
//    // m_subsystem.setAngle(-0.2);
//       m_subsystem.shooterIntake(0);
//       m_subsystem.shooterShootStop();
//   }
//   else {
//      m_subsystem.setPvalue(2.2);
//      m_subsystem.setAngle(-0.005); //-0.05
//      m_subsystem.shooterIntake(0);
//      m_subsystem.shooterShootStop();
//   }
 
//   if(m_Controller.getLeftTriggerAxis() > 0.1){
//     m_subsystem.shooterShoot(1);
//     m_subsystem.setPvalue(6.5);
        
//     m_subsystem.setAngle(-0.11);
//   }

//    if(shootState == 1){
//     if(m_Controller.getRightTriggerAxis() > 0.1){
//       shootState = 2;
//     }
//    } else if(shootState == 2){
//     if (m_subsystem.getAngle() < -0.115) {
//         m_subsystem.shooterIntake(-0.1);
//         m_subsystem.shooterShoot(0.5);}

//       if(!m_subsystem.getSensor()){
//         shootState = 3;
//       }
   
//       if(!(m_Controller.getRightTriggerAxis() > 0.1)){
//           shootState = 1;
//       }
//    } else if( shootState == 3){
//       m_subsystem.shooterShoot(-6); //used t be -6
//       if(m_subsystem.getShooterVelocity() < -4.0){ //used tp be -4.6 .6 is mega powerful
//           m_subsystem.shooterIntake(1);
//       }

//       if(!(m_Controller.getRightTriggerAxis() > 0.1)){
//           shootState = 1;
//       }
//     }

//     if(m_Controller.getYButton()){
//       m_subsystem.shooterIntake(0.8);
    
//     }

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
  
// }

