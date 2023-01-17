// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDouble(0);
  private Joystick Joystick1 = new Joystick(0);
  private Joystick Joystick2 = new Joystick(1);

  private CANSparkMax leftmotor1 = new CANSparkMax(2, MotorType.kBrushed);
  private CANSparkMax leftmotor2 = new CANSparkMax(3, MotorType.kBrushed);
  private CANSparkMax rightmotor1 = new CANSparkMax(9, MotorType.kBrushed);
  private CANSparkMax rightmotor2 = new CANSparkMax(4, MotorType.kBrushed);

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private boolean aTrigger = false;
  private boolean oldTrigger = true;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    boolean newTrigger = Joystick1.getTrigger();
    
    if(newTrigger == true && oldTrigger == false){
      aTrigger = true;
    }

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    double v = tv.getDouble(0.0);
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double a = ta.getDouble(0.0);

     SmartDashboard.putNumber("LimelightX", x);
     SmartDashboard.putNumber("LimelightY", y);
     SmartDashboard.putNumber("LimelightArea", a);
     SmartDashboard.putNumber("Sensitivity", (1 - Joystick1.getThrottle()));

     double xsign;

     if(x < 0){
      xsign = -1;
     }
     else{
      xsign = 1;
     }

     double lrmultiply = (-0.5 * Math.pow(x / 27, 1));
     double lrms = 0.05 * xsign;
     double spinspeed = 0.1;
     double straightspeed = 0.1;
     
     if(Joystick2.getTrigger() == true){
       leftmotor1.set(0);
       leftmotor2.set(0);
       rightmotor1.set(0);
       rightmotor2.set(0);
       aTrigger = false;
     }
     else if(aTrigger == true && v == 0){
      leftmotor1.set(spinspeed);
      leftmotor2.set(spinspeed);
      rightmotor1.set(spinspeed);
      rightmotor2.set(spinspeed);
     }
     else if(aTrigger == true){
      if(x < -5 || x > 5){
        leftmotor1.set(lrmultiply);
        leftmotor2.set(lrmultiply);
        rightmotor1.set(lrmultiply);
        rightmotor2.set(lrmultiply);
      }
      else if(-2 < x && x < 2){
        if(a < 8){
         leftmotor1.set(straightspeed);
         leftmotor2.set(straightspeed);
         rightmotor1.set(-straightspeed);
         rightmotor2.set(-straightspeed);
        }
        else{
         leftmotor1.set(-straightspeed);
         leftmotor2.set(-straightspeed);
         rightmotor1.set(straightspeed);
         rightmotor2.set(straightspeed);
        }
      }
      else{
       leftmotor1.set(lrms);
       leftmotor2.set(lrms);
       rightmotor1.set(lrms);
       rightmotor2.set(lrms);
      }
     }
     else{
       leftmotor1.set((1-Joystick1.getThrottle()) * Joystick1.getY());
       leftmotor2.set((1-Joystick1.getThrottle()) * Joystick1.getY());
       rightmotor1.set(-(1-Joystick1.getThrottle()) * Joystick2.getY());
       rightmotor2.set(-(1-Joystick1.getThrottle()) * Joystick2.getY());
     }

     oldTrigger = newTrigger;
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}