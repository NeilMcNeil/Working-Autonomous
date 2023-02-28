package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.TimedRobot;

public class AutonomousCommand extends CommandBase {
  /** Creates a new AutonomousDrive. */

  
  static double initTime;
  Timer clock = new Timer();
  public AutonomousCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    initTime = clock.get();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.m_robotContainer.driveSubsystem.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double ultrasonicRange = RobotMap.ultrasonic.getRangeInches();
    // Analog voltage to range in inches converter
    /*
     * Based on the type of ultrasonic sensor, use one of these two variables for
     * the range
     */
    // double timePassed = clock.get() - initTime;
    // double analogDistFromObj = RobotMap.getDistFromObj();
    // double analogDistFromBall = RobotMap.getDistFromBall();
    moveRobot();

   
  }


  private void moveRobot() {
    Robot.m_robotContainer.driveSubsystem.drive(Constants.slowRobotSpeed, 0);

  }

  // private void moveRobot() {
  //   Robot.m_robotContainer.driveSubsystem.drive(Constants.slowRobotSpeed, 0);
  // }



  // Called once the command ends or is interrupted.
  // @Override
  // public void end(boolean interrupted) {
  //   Robot.m_robotContainer.driveSubsystem.stop();
  // }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

