// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* REV Imports */
//import com.revrobotics.CANSparkBase.IdleMode;

import com.revrobotics.ResetMode;

import org.w3c.dom.Text;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /* Set up our motors */
  SparkMax driveLeftA = new SparkMax(1,MotorType.kBrushed);
  SparkMax driveLeftB = new SparkMax(3,MotorType.kBrushed);
  SparkMax driveRightA = new SparkMax(4,MotorType.kBrushed);
  SparkMax driveRightB = new SparkMax(2,MotorType.kBrushed);
  SparkMax launcherMotor = new SparkMax(7, MotorType.kBrushed);
  SparkMax hopperMotor = new SparkMax(6, MotorType.kBrushed);
 
  // These functions set the speed of the drive motors
  // any bias between the left and right motors is handled by 
  // applying a multiplier to the left motors
  private void setLeftSpeed(double speed)
  {
    //Calibrate: Change bias to offset drift on Motors
    //Don't change bias more than between .9 and 1.1
    double leftWheelBias = 1;
    driveLeftA.set(speed * leftWheelBias);
    driveLeftB.set(speed * leftWheelBias);
  };

  private void setRightSpeed(double speed)
  {
    driveRightA.set(speed);
    driveRightB.set(speed);
  };

  // Command launch and hopper motors
  // intake: launch motor (-) | hopper motor (-)
  // launch: launch motor (-) | hopper motor (+)
  // unstick: launch motor (0) | hopper motor (-)
  // empty hopper: launch motor (+) | hopper motor (+)
  private void setEjectSpeeds(double launchspeed, double hopperspeed) {
  
  // clamp inputs
  launchspeed = Math.max(-1.0, Math.min(1.0, launchspeed));
  hopperspeed = Math.max(-1.0, Math.min(1.0, hopperspeed));

  if (safetyFaultActive) {
    ejectDelayActive = false;
    safeState();
    return;
  }

  // Launch flow
  if  (launchspeed < 0.0 && hopperspeed > 0.0) {
    double spinUpPower = Math.signum(launchspeed) * 1.0; // always max effort in launch direction

    // If not already in spin-up, and launcher currently near 0, start timer and spin at max
    if (!ejectDelayActive && Math.abs(launcherMotor.get()) < EPS) {
      ejectDelayActive = true;
      ejectStartTime_s = Timer.getFPGATimestamp();
      launcherMotor.set(spinUpPower);
      hopperMotor.set(0.0);
      return;
    }

    // If spin-up active, check elapsed time
    if (ejectDelayActive) {
      double elapsed = Timer.getFPGATimestamp() - ejectStartTime_s;
      if (elapsed < ejectDelay_s) {
        // Continue spin-up at max effort, keep hopper off
        launcherMotor.set(spinUpPower);
        hopperMotor.set(0.0);
        return;
      } else {
        // Spin-up complete: set requested speeds and clear spin-up state
        ejectDelayActive = false;
        launcherMotor.set(launchspeed);
        hopperMotor.set(hopperspeed);
        return;
      }
    }

    // Fallback: if launch requested but we didn't trigger spin-up (e.g., launcher already running),
    // set requested speeds immediately
    launcherMotor.set(launchspeed);
    hopperMotor.set(hopperspeed);
    return;
  }

  // Stop / reset
  if (Math.abs(launchspeed) < EPS && Math.abs(hopperspeed) < EPS) {
    ejectDelayActive = false;
    launcherMotor.set(0.0);
    hopperMotor.set(0.0);
    return;
  }

  // Intake: both negative -> run immediately
  if (launchspeed < 0.0 && hopperspeed < 0.0) {
    ejectDelayActive = false;
    launcherMotor.set(launchspeed);
    hopperMotor.set(hopperspeed);
    return;
  }

  // Unstick: launcher 0, hopper negative -> run immediately
  if (Math.abs(launchspeed) < EPS && hopperspeed < 0.0) {
    ejectDelayActive = false;
    launcherMotor.set(0.0);
    hopperMotor.set(hopperspeed);
    return;
  }

  // Empty hopper: both positive -> run immediately
  if (launchspeed > 0.0 && hopperspeed > 0.0) {
    ejectDelayActive = false;
    launcherMotor.set(launchspeed);
    hopperMotor.set(hopperspeed);
    return;
  }

  // Any other unexpected combination: set directly (spin-up not used)
  System.err.println("Error: Unknown state in setEjectSpeeds() - no delay triggered for launch command");
  ejectDelayActive = false;
  launcherMotor.set(launchspeed);
  hopperMotor.set(hopperspeed);
}

  public void setSafetyFault(String message)
  {
      safetyFaultActive = true;
      System.err.println("Error: "+ message);
  }

  public void safeState() {
    // Disable all motors
    // This function is used to set the robot to a safe state without disabling it
    driveLeftA.set(0.0);
    driveLeftB.set(0.0);
    driveRightA.set(0.0);
    driveRightB.set(0.0);
    launcherMotor.set(0.0);
    hopperMotor.set(0.0);

    // Only display message on first call
    if (DisplaySafeState) {
      System.err.println("Safe State: All motors set to zero.");
      DisplaySafeState = false;
    }
  }

  private double k_MotorSpeed(double motorSpeed) {

    // Calibrate: factor k = speed in inches per second / motor speed command
    double k = 156; // 156 for drive, 116 for turn, need to integrate
    
    // if (motorSpeed < 0.4) { 
    //   k = 129;
    // } else {
    //   k = 19.37 * motorSpeed + 96.39;
    // }
    
    double k_default = 150; // only returned in case of error

    if (k == 0) {
      safeState(); // set motors to a safe state  
      System.err.println("Error: k_MotorSpeed() returned zero!");
      k = k_default;
    }
    return k;
  }

  /* Motoin FIlters */
  SlewRateLimiter driveFilter = new SlewRateLimiter(4);
  SlewRateLimiter turnFilter = new SlewRateLimiter(4);  

  /* set up our controllers */
  XboxController driverController = new XboxController(0);
  Joystick opController = new Joystick(1);

  /* Global variables */
  boolean safetyFaultActive = false;
  boolean DisplaySafeState = true;
  boolean ejectDelayActive = false;
  double ejectStartTime_s = 0;
  double driveFactor = 0.6;
  double autoStart = 0;
  private static final double EPS = 0.01; // deadband: treat |v| < EPS as zero
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // print a message to the driver station, at a lower rate
  private int consolePrintCounter = 0;
  private static final int CONSOLE_PRINT_INTERVAL = 25; // print once every 25 loops (~0.5s @ 20ms)
  private void printThrottled(String msg) {
   if (++consolePrintCounter % CONSOLE_PRINT_INTERVAL == 0) {
     System.out.println(msg);
   }
  }

  // Calibrate: Motor Voltage Compensation
  // Set the nominal voltage (usually between 10.0 and 12.0V)
  // nominal voltage is typically set near minimum voltage, to provide consistent performance as battery voltage drops during matches
  private static final double VOLTS_NOMINAL = 11.0;  

  // Calibrate: Launch Motor Commands
  // Two launch modes are supported: slow launch for better accuracy and fast launch for high delivery speeds
  // Slow Launch is typically used for autonomous, because fuel is limited
  // Slow and Fast Launch modes are available by button mapping in Teleop
  double slowLaunchSpeed = -0.83; // -1.0 value of launch speed for slow launch
  double slowHopperSpeed = 0.75; // 0.6 value of slow hopper speed for slow launch
  double fastLaunchSpeed = -1.0; // -1.0 value of launch speed for fast launch
  double fastHopperSpeed = 0.75; // 1.0 value of fast hopper speed for fast launch
  double ejectDelay_s = 0.5; // 0.5 time it takes for launcher to spin up, typically 0.2 to 1 seconds
  double unstickHopperSpeed = -1.0; // -1.0 value of hopper speed for unsticking fuel

  // Calibrate: Intake Motor Commands
  double IntakeFrontSpeed = -0.75; // -0.75 value of intake front speed
  double IntakeHopperSpeed = -1.0; // -1.0 value of intake hopper speed
  double EmptyFrontSpeed = 0.75; // 0.75 value of front speed for emptying hopper
  double EmptyHopperSpeed = 1.0; // 1.0 value of hopper speed for emptying hopper

  SparkMaxConfig configInverted = new SparkMaxConfig();
  SparkMaxConfig configNormal = new SparkMaxConfig();
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    /* Set up our motor settings*/

    CameraServer.startAutomaticCapture(0);
    
    // Sets the settings on the SparkMax configs and applies them to the motors
    configInverted.inverted(true);
    configInverted.idleMode(IdleMode.kBrake);
    configInverted.voltageCompensation(VOLTS_NOMINAL);
    configNormal.inverted(false);
    configNormal.idleMode(IdleMode.kBrake);
    configNormal.voltageCompensation(VOLTS_NOMINAL);
    
    // Configure the motors with the specified settings
    driveLeftA.configure(configInverted, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    driveLeftB.configure(configInverted, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    driveRightA.configure(configNormal, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    driveRightB.configure(configNormal, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    launcherMotor.configure(configNormal, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    hopperMotor.configure(configNormal, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /* Autonomous Mode Global Definitions */
  private double loop_s; 
  private double k; 

  /* These are the available routines and drive modes */
  private enum startLoc {TEST, LEFT, CENTER, RIGHT};
  private enum driveMode { DRIVE, TURN, EJECT, PAUSE }
  
  // Autonomous global variables
  private double t_max_autonomous = 20;  // max step time cannot exceed autonomous time, safety measure
  private double accel_rate, motorCommand = 0; // set elsewhere
  private double velocity_target = 0;
  private int numSteps;
  private boolean stepInitialized[], forward, AutonomousComplete = false, TimerStarted; 
  private boolean[] stepDownInit; // per-step flag used to initialize deceleration phase
  private driveMode Mode[];
  private double Magnitude[], MotorCommands[], stepStartTime[], t_total_s, t_accel, M_step_up, M_step_down;
  private int stepIdx;

  // Calibrate: Set robot track width in inches
  private double trackwidth = 21.5;

  // Calibrate: Drive Motor commands
  // These are the initial and final motor command targets
  // to overcome friction and inertia
  private double motorCommand_start = 0.09; // 0 to 0.2
  private double motorCommand_stop = 0.09;  // -0.2 to 0.2
  private double accel_offset = 0.12; // .12 offset for acceleration

  /* The Autonomous Routines are defined here */

    // Calibrate: TEST Autonomous Routine
    private driveMode[] testModes = {
      driveMode.TURN
    };
    
    private double[] testMagnitudes = {
      180
    };
  
    /* motor command for each step */
    private double[] testMotorCommands = {
      0.4
    };

    // Calibrate: LEFT Autonomous Routine
    private driveMode[] leftModes = {
      driveMode.DRIVE,
      driveMode.TURN,
      driveMode.EJECT
    };
  
  private double[] leftMagnitudes = {
    60, // forward inches
    45, // right degrees
    12, // eject for 12 seconds
  };

  /* motor command for each step */
  private double[] leftMotorCommands = {
    0.4, // DRIVE
    0.4, // TURN
    12 // EJECT for 12 seconds
  };

  // Calibrate: CENTER Autonomous Routine
  private driveMode[] centerModes = {
    driveMode.DRIVE,
    driveMode.EJECT
  };
  
  private double[] centerMagnitudes = {
    60, // forward inches
    12 // eject for x seconds
  };

  /* motor command for each step */
  private double[] centerMotorCommands = {
    0.4, // DRIVE
    12 // eject for x seconds
  };

  // Calibrate: RIGHT Autonomous Routine
  private driveMode[] rightModes = {
    driveMode.DRIVE,
    driveMode.TURN,
    driveMode.EJECT
  };
  
  private double[] rightMagnitudes = {
    60, // forward inches
    -45, // left degrees
    12 // eject for x seconds
  };

  /* motor command for each step */
  private double[] rightMotorCommands = {
    0.4, // DRIVE
    0.4, // TURN
    12 // eject for x seconds
  };

  /*
  This autonomous (along with the chooser code above) shows how to select between different
  autonomous modes using the dashboard. The sendable chooser code works with the Java
  SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
  uncomment the getString line to get the auto name from the text box below the Gyro

  You can add additional auto modes by adding additional comparisons to the switch structure
  below with additional strings. If using the SendableChooser make sure to add them to the
  chooser code above as well.
   */

  @Override
  public void autonomousInit() {
    //m_autoSelected = m_chooser.getSelected(); // is this used?
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    //System.out.println("Auto selected: " + m_autoSelected);

    // Reset safety faults
    safetyFaultActive = false;

    // Initialize the routine step counter
    stepIdx = 0;
    
    // Calibrate: Pick a routine
    startLoc routine = startLoc.TEST;
    System.out.println(routine + " Routine Loaded");
    
    // Load the autonomous routine
    switch (routine) {

      case TEST:
        Mode = testModes;
        Magnitude = testMagnitudes;
        MotorCommands = testMotorCommands;
        break;
      
      case LEFT:
        Mode = leftModes;
        Magnitude = leftMagnitudes;
        MotorCommands = leftMotorCommands;
        break;
        
      case CENTER:
        Mode = centerModes;
        Magnitude = centerMagnitudes;
        MotorCommands = centerMotorCommands;
        break;

      case RIGHT:
        Mode = rightModes;
        Magnitude = rightMagnitudes;
        MotorCommands = rightMotorCommands;
        break;

      default:
      setSafetyFault("Routine not defined");
        break;
    }

    numSteps = Mode.length;

    // Print the values to the console:
    System.out.println("Number of steps: " + numSteps);

    // Initialize array to false (default)
    stepInitialized = new boolean[numSteps];
    stepDownInit = new boolean[numSteps]; // initialize decel flags per-step
    // Initialize start time
    stepStartTime = new double[numSteps];

    // Initialize the motor command to zero
    motorCommand = 0;

  }

  /** This function is called periodically during autonomous. */
  //CTS
  @Override
  public void autonomousPeriodic() {

    // Initialize variables
    // These values get overwritten
    double v_max = 0;
    double distance = 0;
    double stepTime;


    // Check for end of routine
    // index starts at zero and increments to (numSteps - 1)
    if (stepIdx >= numSteps) {
      safeState(); // Set all motors to zero
      
      // Display complete message once
      if (!AutonomousComplete) {
        System.out.println("All Steps Complete");
        AutonomousComplete = true;
      }
      return;
    }

    // Initialize the drive or turn calculation
    if (!stepInitialized[stepIdx]) {
      
      // Log
      System.out.println("Initializing Step " + (stepIdx+1) + " of " + numSteps + ": " + Mode[stepIdx]);

      /* Negative motor speed commands are not supported here
      To drive backwards, command negative distance
      To turn left, command negative angle */
      if (MotorCommands[stepIdx] <= 0) {
        setSafetyFault("Motor command is negative in drive or turn function");
      }

      /* These variables may or may not change in each periodic, but are calculated again here, just in case*/
      loop_s = getPeriod(); 
      k = k_MotorSpeed(MotorCommands[stepIdx]);

      //System.out.println("looptime: " + loop_s);
      System.out.println("Motor Conversion k: " + k);


      /* Convert negative distance to direction */
      forward = true;
      if (Magnitude[stepIdx] < 0) {
        // backwards
        forward = false;
      }
      Magnitude[stepIdx] = Math.abs(Magnitude[stepIdx]);

      System.out.println("Magnitude: " + Magnitude[stepIdx]);
      System.out.println("Motor Command: " + MotorCommands[stepIdx]);

      /* Convert motor speed command to inches per second */
      double v_command_ips = k * MotorCommands[stepIdx];

      //System.out.println("Velocity Command: " + v_command_ips + " in/s");

      /* Define wheel distance to travel */
      switch (Mode[stepIdx]) {
          case DRIVE:
          
            /* DRIVE works in terms of distance
            (both wheels moving together) */
            distance = Magnitude[stepIdx];

            /* Calibrate: Max without prevent slipping 
            acceleration rate for DRIVE steps
            should be between 100 and 600 */
            accel_rate = 200;
            break;
          
          case TURN:

            /* Calibrate: Max without prevent slipping 
            acceleration rate for TURN steps
            should be between 100 and 600 */
            accel_rate = 200;

            /* TURN works in terms of angle which converts to distance (arclength)
            (wheels turning in opposite directions) */
            distance = (trackwidth * Math.PI * Magnitude[stepIdx]) / 360.0;
            break;

          case EJECT:
            /* No ramp needed for ejecting */
            accel_rate = 9999;

            /* EJECT magnitude is not actually distance, but time instead */
            distance = Magnitude[stepIdx];            
            break;

          case PAUSE:
          
            /* PAUSE is not actually distance, but time instead */
            distance = Magnitude[stepIdx];
            
            /* Not applicable to PAUSE */
            accel_rate = 9999;
            break;

        default:
          distance = Magnitude[stepIdx];
          accel_rate = 9999;
          setSafetyFault("Unknown Drive State");
          break;              
      }

      System.out.println("Accel Rate: " + accel_rate + " in/s/s");
      System.out.println("Distance Command: " + distance + " in");

      /* Convert acceleration rate to motor step per loop */
      M_step_up = (accel_rate / k - motorCommand_start) * loop_s;
      M_step_down = (accel_rate / k - motorCommand_stop) * loop_s;

      //System.out.println("Motor Step Up: " + M_step_up + " per loop");
      //System.out.println("Motor Step Down: " + M_step_down + " per loop");

      /* Calibrate: This adjustment factor accounts for estimated error in the ramp rate function
      If controller loop rate is changed, this factor will change, previously 1.65 */
      distance = distance + 0 * MotorCommands[stepIdx];

      //System.out.println("Adjusted distance: " + distance + " in");      
      
      /* Maximum velocity that can be achieved in the distance given
      assuming accel rate is equal to decel rate */
      v_max = Math.sqrt(accel_rate * distance);
      velocity_target = Math.min(v_command_ips,v_max); // clipped command

      //System.out.println("Max Velocity for Distance: " + v_max + " in/s");
      //System.out.println("Target Velocity: " + velocity_target);

      /* Error if arbitrated motor speed is 0 */
      if (velocity_target <= 0) {
        safetyFaultActive = true;
        System.err.println("Error: Arbitrated motor speed is zero");      
      }

      // Calculate ramp time
      t_accel = velocity_target / accel_rate;

      // Calculate total time
      t_total_s = distance/velocity_target + t_accel; 

      System.out.println("Total Motor On Time: " + t_total_s + " seconds");
      System.out.println("Ramp Time (each): " + t_accel + " seconds");

      /* If drive mode is EJECT, override time with EJECT time */
      if (Mode[stepIdx] == driveMode.EJECT) {
        t_total_s = Magnitude[stepIdx];
      } else if (Mode[stepIdx] == driveMode.PAUSE) { // If drive mode is PAUSE, override time with PAUSE time
        t_total_s = Magnitude[stepIdx];
      }

      /* Error if arbitrated motor speed is 0 */
      if ((t_total_s <= 0) || (t_total_s > t_max_autonomous)) {
        setSafetyFault("Calculated step time is invalid");
      }

      /* Initialize the motor command to start value plus acceleration offset */
      motorCommand = motorCommand_start + accel_offset;

      /* Reset Timer Boolean */
      TimerStarted = false;

      /* Set initialization complete */
      stepInitialized[stepIdx] = true;
      stepDownInit[stepIdx] = false; // flag to initialize deceleration phase

      System.out.println("Step " + (stepIdx+1) + " of " + numSteps + ": Initialization Complete");
    
    /* Step has been initialized, calculate step commands */
    } else {

      loop_s = getPeriod(); 
      k = k_MotorSpeed(MotorCommands[stepIdx]);

      //System.out.println("looptime: " + loop_s);
      //System.out.println("Motor Conversion k: " + k);

      /* Execute the step timer */
      if (!TimerStarted){
        /* Start the timer */
        stepStartTime[stepIdx] = Timer.getFPGATimestamp();
        stepTime = 0;
        TimerStarted = true;

      } else {
        /* Measure current drive time for this step */
        stepTime = Timer.getFPGATimestamp() - stepStartTime[stepIdx];
      }

      if (stepTime < t_accel) {
        // Ramp up motor command
        motorCommand = motorCommand + M_step_up;

      } else if (stepTime >= (t_total_s - t_accel)) {
        // Ramp down motor command
        if (!stepDownInit[stepIdx]) {
          motorCommand = velocity_target / k - accel_offset;
          stepDownInit[stepIdx] = true;
        } else {
          motorCommand = motorCommand - M_step_down;
        }
      } else {
        // constant at target velocity
        motorCommand = velocity_target / k;
      }

      /*
      motorCommand = motorCommand_amount(t_total_s, stepTime, accel_rate, velocity_target);

      public double motorCommand_amount(double commandedTime, double currentTime, double accelRate, double motorTarget) {
        if (commandedTime >= currentTime) {
          if (motorTarget / accelRate * 2 >= commandedTime) {
            if (motorTarget / accelRate * currentTime < motorTarget) {
              return accelRate * currentTime;

            } else if (commandedTime - currentTime - motorTarget / accelRate <= 0) {
              final double decelTime = commandedTime - currentTime;
              return decelTime - accelRate * currentTime;

            } else {
              return motorTarget;

            }

          } else {
            if (currentTime / 2 <= commandedTime) {
              return accelRate * currentTime;

            } else {
              final double decelTime = commandedTime - currentTime;
              return decelTime - accelRate * currentTime;

            }
          }
        }
      }
      */

      SmartDashboard.putNumber("Step Time", stepTime);
      SmartDashboard.putNumber("motorCommand", motorCommand);

      /* Clip motor command between limits (usually 0 to 1) */
      motorCommand = Math.max(Math.min(motorCommand, 1), motorCommand_stop - accel_offset);

      SmartDashboard.putNumber("motorCommand", motorCommand);

      /* Fault active, go to safe state */
      if (safetyFaultActive) {
        safeState();
        System.err.println("Error: Safety Fault Active, Exiting Routine");
        return; // exit autonomous

      } else {
        /* Set Motor Commands */
        switch (Mode[stepIdx]) {
          case DRIVE:
            if (forward){
              // drive forward
              setLeftSpeed(motorCommand);
              setRightSpeed(motorCommand);
              SmartDashboard.putNumber("Driving Forward Power", motorCommand);
      
            } else {
              // drive backward
              setLeftSpeed(-motorCommand);
              setRightSpeed(-motorCommand);
              SmartDashboard.putNumber("Driving Backward Power", motorCommand);
            }
            break;
        
          case TURN:
            if (forward){
              /* Right turn */
              setLeftSpeed(motorCommand);
              setRightSpeed(-motorCommand);
              SmartDashboard.putNumber("Turning Power", motorCommand);
      
            } else {
              /* Left turn */
              setLeftSpeed(-motorCommand);
              setRightSpeed(motorCommand);
              SmartDashboard.putNumber("Turning Power", motorCommand);
            }
            break;

          case EJECT:
            if (forward){
              /* Launch fuel */
              setEjectSpeeds(slowLaunchSpeed, slowHopperSpeed); // set eject speeds, which handles the delay logic internally
              SmartDashboard.putNumber("Eject Time Commanded", motorCommand); // Display commanded eject time

            } else {
              /* Intake fuel */
              setEjectSpeeds(IntakeFrontSpeed, IntakeHopperSpeed);
              SmartDashboard.putNumber("Intake Time Commanded", motorCommand); // Display commanded intake time
            }
            break;

          case PAUSE:
            /* Do Nothing */
            SmartDashboard.putNumber("Pause Time Commanded", motorCommand);
            break;

          default:
            setSafetyFault("Invalid Step Mode Commanded");
            break;              
        }
      }

      /* Check for step complete */
      if (stepTime >= t_total_s){
        safeState(); // Go to a safe state
        DisplaySafeState = true; // reset safe state display        
        stepIdx = stepIdx + 1; // Go to the next step (in the next loop)
        System.out.println("Step " + (stepIdx+1) + " Complete");      
      }
    }
  }
    
  /* This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /* This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    /* Driver contols */

//Percentage of motor speed ONLY SET BETWEEN 0 and 1
//Calibrate: Teleop Motor Speeds, Fast and Slow
    if(driverController.getLeftBumper()==true) {
      driveFactor = 0.7; // 0.7
    } else {
      driveFactor= 0.5; // 0.5
    }
    double forward = driveFilter.calculate(driverController.getRawAxis(1)); // y-axis, left joystick
    double turn = turnFilter.calculate(driverController.getRawAxis(4)); // x-axis, right joystick
    double driveLeftPower;
    double driveRightPower;

    // Drive Forward
    if (driverController.getRawButton(6))
    {
      driveLeftPower = forward + turn;
      driveRightPower = forward - turn;
      setLeftSpeed(driveLeftPower * driveFactor);
      setRightSpeed(driveRightPower * driveFactor);
    }
    
    // Drive in reverse with reversed controls
    else
    {
      driveLeftPower = forward - turn;
      driveRightPower = forward + turn;
      setLeftSpeed(driveLeftPower * driveFactor * -1);
      setRightSpeed(driveRightPower * driveFactor * -1);
    }

    // /* Test Code */
    // double testAxis = driveFilter.calculate(driverController.getRawAxis(1)); // y-axis, left joystick
    // double testCommand = testAxis*0.25-0.75;
    // launcherMotor.set(testCommand);
    // if (testAxis < -0.1) {
    //   hopperMotor.set(0.75);
    // }
    // printThrottled("Test Motor Command: " + testCommand);
    // /* Test Code End */

    /* Op controls */

    //Turning speed Control only change in the IF commands
    double frontSpeed = 0;
    double backSpeed = 0;
    //^^^^ Dont change this one
    //launcherMotor.set(opController.getTwist());
    //CTS

    // launch
    if (opController.getRawButton(2))
    {
      // launch fast
      frontSpeed = fastLaunchSpeed;
      backSpeed = fastHopperSpeed;
    }else if (opController.getRawButton(6)) //|| opController.getRawButton(5))
    {
      // launch slow
      frontSpeed = slowLaunchSpeed;
      backSpeed = slowHopperSpeed;
    }

    // trigger to intake
    else if(opController.getRawButton(1))

    {
      frontSpeed = IntakeFrontSpeed;
      backSpeed = IntakeHopperSpeed;
 
    }
    else if (opController.getRawButton(3))
    {
      // Unstick
      frontSpeed = 0;
      backSpeed = unstickHopperSpeed;
    }
    else if (opController.getRawButton(5))
    {
      // Empty Hopper
      frontSpeed = EmptyFrontSpeed;
      backSpeed = EmptyHopperSpeed;
    }
    else
    {
      // Action for no buttons pressed
      // Do not change away from 0 
      frontSpeed = 0;
      backSpeed = 0;
    }
    setEjectSpeeds(frontSpeed, backSpeed); // set eject motor speeds
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

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
