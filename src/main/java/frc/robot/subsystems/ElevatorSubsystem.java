package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import javax.lang.model.util.ElementScanner14;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorSubsystem extends SubsystemBase {
  
  private static double kDt = 0.02; //some sort of timing thing, don't touch probably

  private final SparkMax elevatorOneSparkMax = new SparkMax(14, MotorType.kBrushless);
  SparkMaxConfig elevatorOneConfig = new SparkMaxConfig();

  private final SparkMax elevatorTwoSparkMax = new SparkMax(15, MotorType.kBrushless);
  SparkMaxConfig elevatorTwoConfig = new SparkMaxConfig();

  private final RelativeEncoder mElevatorEncoder;
  private final SparkClosedLoopController mElevatorPID;

  private final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(2000, 110));
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  public static int elevatorLevel = 0; 

  public ElevatorSubsystem() {
    //Configure elevator motor 1 
    elevatorOneConfig.idleMode(IdleMode.kBrake);
    elevatorOneConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.3, 0, 0); 
    elevatorOneConfig.encoder.positionConversionFactor(1);//(360.0/(60.0));
    elevatorOneConfig.encoder.velocityConversionFactor(1); //(360.0/(60.0*10));
    elevatorOneConfig.smartCurrentLimit(30);
    elevatorOneSparkMax.configure(elevatorOneConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 
    //Set initial encoder position to 0
    mElevatorEncoder = elevatorOneSparkMax.getEncoder();
    mElevatorEncoder.setPosition(0);
    mElevatorPID = elevatorOneSparkMax.getClosedLoopController();

    elevatorTwoConfig.idleMode(IdleMode.kBrake);
    //elevatorTwoConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1, 0, 0); 
    elevatorTwoConfig.encoder.positionConversionFactor(1);//(360.0/(60.0));
    elevatorTwoConfig.encoder.velocityConversionFactor(1); //(360.0/(60.0*10));
    elevatorTwoConfig.smartCurrentLimit(30);
    elevatorTwoConfig.follow(14);
    // Configure motor 2 to follow motor 1
    elevatorTwoSparkMax.configure(elevatorTwoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Calculate the set point for the elevator
    m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);

    // New posistion to the PID controller
    mElevatorPID.setReference(m_setpoint.position, com.revrobotics.spark.SparkBase.ControlType.kPosition);
  }

    // set target position for the elevator
    public void setPosistion(double posistion) {
        m_goal = new TrapezoidProfile.State(posistion, 0);
    }

    public void doNothing(){}
    
    public void setLevel(int level){
      elevatorLevel = level;
    }
    public Command setLevelCommand(int level){
      return this.runEnd(() ->setLevel(level), () ->doNothing());
    }
    public Command elevatorUp(int level){
      // move the elevator up
      if (level == 1){
        elevatorLevel = 1; 
        return this.startEnd(() -> setPosistion(33), () -> doNothing()).until(() -> moveInPosistion());
      }
      else if (level == 2){
        elevatorLevel = 2; 
        return this.startEnd(() -> setPosistion(48), () -> doNothing()).until(() -> moveInPosistion());
      }
      else if (level == 3){
        elevatorLevel = 3; 
        return this.startEnd(() -> setPosistion(90), () -> doNothing()).until(() -> moveInPosistion());
      }
      else if (level == 4){
        elevatorLevel = 4;
        //160 is max!!! Stuff will probably break if you go above it
        return this.startEnd(() -> setPosistion(160), () -> doNothing()).until(() -> moveInPosistion());
      }
      else if (level == 5){
        elevatorLevel = 5;
        return this.startEnd(() -> setPosistion(50), () -> doNothing()).until(() -> moveInPosistion());
      }
      else{
        return this.startEnd(() -> setPosistion(0), () -> doNothing()).until(() -> moveInPosistion());
      }
    }

    public Command elevatorDown(){
      // move the elevator down
      elevatorLevel = 0;
      return this.startEnd(() -> setPosistion(0), () -> doNothing()).until(() -> moveInPosistion());
    }

    public boolean moveInPosistion() {
      return Math.abs(m_setpoint.position - m_goal.position) < 5;
    }
}
