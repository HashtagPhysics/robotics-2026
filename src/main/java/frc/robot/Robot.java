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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* REV Imports */
//import com.revrobotics.CANSparkBase.IdleMode;

// SparkMAX API imports
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // Set up our motor types and assign CAN ID's
  // VERY IMPORTANT: NEO motors must be configured as brushless!
  // If a NEO is configured as brushed, it will destroy the motor
  SparkMax driveLeftA = new SparkMax(1,MotorType.kBrushed);
  SparkMax driveLeftB = new SparkMax(3,MotorType.kBrushed);
  SparkMax driveRightA = new SparkMax(4,MotorType.kBrushed);
  SparkMax driveRightB = new SparkMax(2,MotorType.kBrushed);
  SparkMax launcherMotor; // will be constructed in the initializer
  SparkMax hopperMotor = new SparkMax(6,MotorType.kBrushed);

  // Build configs
  SparkMaxConfig configInverted = new SparkMaxConfig();
  SparkMaxConfig configNormal = new SparkMaxConfig();
  SparkMaxConfig configLauncher = new SparkMaxConfig();
  
  // Calibrate: Motor Voltage Compensation
  // To enable, set USE_VOLT_COMP to true and 
  // specify the nominal voltage (usually between 10.0 and 12.0V)
  // nominal voltage is typically set near minimum voltage, 
  // to provide consistent performance as battery voltage drops during matches
  static final boolean USE_VOLT_COMP = true;
  static final double VOLTS_NOMINAL = 11.0;  
  
  // Calibrate: Motor Configuration
  // VERY IMPORTANT: NEO motors must be configured as brushless!
  // If a NEO is configured as brushed, it will destroy the motor
  static final boolean launcherMotorNEO = true; // Set to true if using NEO motor for launcher, false if using brushed motor 
  static final boolean launcherMotorClosedLoop = true; // Set to true to use closed loop control for launcher motor, false for open loop voltage control

  // Calibrate: Teleop Motor Speeds, Fast and Slow
  // Percentage of motor speed ONLY SET BETWEEN 0 and 1
  static final double driveSpeedBoost = 1.0; // fast drive speed
  static final double driveSpeedNormal = 0.5; // normal drive speed

  //Calibrate: Change bias to offset drift on Motors
  //Don't set bias outside of .9 to 1.1
  static final double leftWheelBias = 1;

  // Calibrate: Launch Motor Commands
  // Two launch modes are supported: slow launch for better accuracy and fast launch for high delivery speeds
  // Slow Launch is typically used for autonomous, because fuel is limited
  // Slow and Fast Launch modes are available by button mapping in Teleop
  // launcher now configured so positive command -> launch direction
  static final double slowLaunchSpeed = 0.83; // positive value for slow launch
  static final double fastLaunchSpeed = 1.0;  // positive value for fast launch
  static final double launchHopperSpeed = 0.8; // 0.6 value of slow hopper speed for slow launch
  static final double unstickHopperSpeed = -1.0; // -1.0 value of hopper speed for unsticking fuel

  // Calibrate: NEO motor controls
  static final double slowLaunchRPM = 2900; // target RPM for slow launch mode, if using NEO motor
  static final double fastLaunchRPM = 3500; // target RPM for fast launch mode, if using NEO motor
  static final double launchRPMTolerance = 150; // tolerance for considering launcher "at speed", in RPM, if using NEO motor

  // Calibrate: Intake Motor Commands
  static final double IntakeHopperSpeed = -1.0; // -1.0 value of intake hopper speed
  static final double EmptyHopperSpeed = 1.0; // 1.0 value of hopper speed for emptying hopper
  static final double EmptyFrontRPM = -3300;

  // Calibrate: Set robot track width in inches
  static final double trackwidth = 21.5;

  // Calibrate: Drive Motor commands
  // These are the initial and final motor command targets
  // to overcome friction and inertia
  static final double motorCommand_start = 0.09; // 0 to 0.2
  static final double motorCommand_stop = 0.09;  // -0.2 to 0.2
  static final double accel_offset = 0.12; // .12 offset for acceleration

  /* Instance initializer: construct and configure controllers using the SparkMax + SparkMaxConfig API */
  {
    // construct launcher according to calibration flag
    if (launcherMotorNEO) {
      launcherMotor = new SparkMax(10, MotorType.kBrushless);
    } else {
      launcherMotor = new SparkMax(7, MotorType.kBrushed);
    }

    // Drive config (invert left, brake)
    configInverted.inverted(true);
    configInverted.idleMode(IdleMode.kBrake);
    if (USE_VOLT_COMP) {
      configInverted.voltageCompensation(VOLTS_NOMINAL);
    } else {
      configInverted.disableVoltageCompensation();
    }

    configNormal.inverted(false);
    configNormal.idleMode(IdleMode.kBrake);
    if (USE_VOLT_COMP) {
      configNormal.voltageCompensation(VOLTS_NOMINAL);
    } else {
      configNormal.disableVoltageCompensation();
    }

    // Launcher config
    configLauncher.inverted(true);
    configLauncher.smartCurrentLimit(40);
    configLauncher.idleMode(IdleMode.kCoast);
    if (launcherMotorNEO && launcherMotorClosedLoop) {
      configLauncher
        .closedLoop    
          .pid(0.0018, 0, 0) // slot 0
          .feedForward
            .kS(0) // slot 0 by default
            .kV(0.00211);
    }
    if (USE_VOLT_COMP) {
      configLauncher.voltageCompensation(VOLTS_NOMINAL);
    } else {
      configLauncher.disableVoltageCompensation();
    }
    
  }

  // Initialize the closed loop controller, 
  // for any motors where we want to actively control speed
  SparkClosedLoopController launchController = launcherMotor.getClosedLoopController();

  // These functions set the speed of the drive motors
  // any bias between the left and right motors is handled by 
  // applying a multiplier to the left motors
  private void setLeftSpeed(double speed)
  {    
    driveLeftA.set(speed * leftWheelBias);
    driveLeftB.set(speed * leftWheelBias);
  }
 
  private void setRightSpeed(double speed)
  {
    driveRightA.set(speed);
    driveRightB.set(speed);
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
    setLauncherSpeed(0.0);
    hopperMotor.set(0.0);

    // Only display message on first call
    if (DisplaySafeState) {
      System.err.println("Safe State: All motors set to zero.");
      DisplaySafeState = false;
    }
  }

  // Helper Function: command launcher with an RPM value
  // If CIM motor is used for launcher, targetRPM is converted to a motor command using a feedforward model
  private void setLauncherSpeed(double targetRPM) {
      
      // If target RPM is very small, set to zero to prevent motor from trying to hold position
     if (( Math.abs(targetRPM) < minAbsTargetRPM) || safetyFaultActive) {
      launcherTargetRPM = 0.0;
      launcherMotor.set(launcherTargetRPM);
       return;
     } else {
      launcherTargetRPM = targetRPM;
     }
 
     // If using NEO with closed loop control, command target RPM directly
     if (launcherMotorNEO) {
       
       if (launcherMotorClosedLoop) {
         // Closed loop control with NEO
         launchController.setSetpoint(targetRPM, ControlType.kVelocity);
       } else {
         // Open loop control with NEO, using a feedforward model to convert target RPM to motor command
         double cmd = 0.0001748*targetRPM + 0.0624444;
         cmd = Math.max(-1.0, Math.min(1.0, cmd));
         launcherMotor.set(cmd);
       }
     
     // If using CIM, use a feedforward model to convert target RPM to motor command
     } else {
       double cmd = (targetRPM + 33.364) / 3849.5;
       // clamp to safe percent range
       cmd = Math.max(-1.0, Math.min(1.0, cmd));
       launcherMotor.set(cmd);
     }
   }
  
  private double k_MotorSpeed(double motorSpeed) {

    // Calibrate: factor k = speed in inches per second / motor speed command
    double k = 156; // 156 for drive, 116 for turn, need to integrate
    
    /*
    if (motorSpeed < 0.4) { 
      k = 129;
    } else {
      k = 19.37 * motorSpeed + 96.39;
    }
    */
    
    double k_default = 156; // only returned in case of error

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
  double driveFactor = 0.6;
  double autoStart = 0;
  double currentRPM = 0;
  boolean isAtSpeed = false;
  double launcherTargetRPM = 0.0;
      
  // Minimum absolute target RPM considered "non-zero" for closed loop motor commands
  private static final double minAbsTargetRPM = 500;
  
  // print a message to the driver station, at a lower rate
  private int consolePrintCounter = 0;
  private static final int CONSOLE_PRINT_INTERVAL = 25; // print once every 25 loops (~0.5s @ 20ms)
  private void printThrottled(String msg) {
   if (++consolePrintCounter % CONSOLE_PRINT_INTERVAL == 0) {
     System.out.println(msg);
   }
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    /* Set up our motor settings*/

    //CameraServer.startAutomaticCapture(0);
     
  // Configure the motors with the specified settings
    driveLeftA.configure(configInverted, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    driveLeftB.configure(configInverted, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    driveRightA.configure(configNormal, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    driveRightB.configure(configNormal, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    launcherMotor.configure(configLauncher, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    hopperMotor.configure(configInverted, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
    // Get the current velocity from the encoder
    // Check if we are close enough to the target
    if (launcherMotorClosedLoop) {
      currentRPM = launcherMotor.getEncoder().getVelocity();
      isAtSpeed = Math.abs(currentRPM - launcherTargetRPM) < launchRPMTolerance;
    } else {
      isAtSpeed = true;
    }
  }

  /* Autonomous Mode Global Definitions */
  private double loop_s; 
  private double k; 

  /* These are the available routines and drive modes */
  private enum startLoc { TEST, LEFT, CENTER, RIGHT };
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


  @Override
  public void autonomousInit() {
 
    // Spin up the launcher
    setLauncherSpeed(slowLaunchRPM);

    // Calibrate: Set the starting location
    startLoc routine = startLoc.CENTER;

    System.out.println(routine + " Routine Loaded");

    // Reset safety faults
    safetyFaultActive = false;

    // Initialize the routine step counter
    stepIdx = 0;
    
    //int angleOnWall = 30;
    //double disFromCenter = 17.5 + Math.cos(angleOnWall) * 17; // distance verticaly from tower
    //double disFromTower = Math.sqrt(Math.pow(Math.sqrt(Math.pow(17, 2) + Math.pow(Math.cos(30) * 17, 2)) + 24, 2) + Math.pow(disFromCenter, 2));
    double shootDist = 54; // shooting distance in inches, from the back of the robot (shoots backwards)
    double startDist; // distance from back of robot to center of hub, different for each routine
    double emptyHopperTime = 5.5; // time to empty hopper in seconds
    double autoDriveSpeed = 0.5; // motor command for drive


    // Calibrate Autonomous Routines
    switch (routine) {
      case TEST: {
        Mode = new driveMode[] { 
          driveMode.TURN,
          driveMode.PAUSE,
          driveMode.TURN
        };
        Magnitude = new double[] { 
          -138,
          4,
          138,
        };
        MotorCommands = new double[] { 
          autoDriveSpeed,
          autoDriveSpeed,
          autoDriveSpeed
        };
        break;

      }
      case RIGHT: {
        startDist = 47;
        Mode = new driveMode[] {
           driveMode.DRIVE,
           driveMode.EJECT,
           driveMode.DRIVE,
           driveMode.TURN,
           driveMode.DRIVE,
           driveMode.PAUSE,
           driveMode.DRIVE,
           driveMode.TURN,
           driveMode.DRIVE,
           driveMode.EJECT
        }; 
          
        Magnitude = new double[] { 
          shootDist - startDist,
          emptyHopperTime,
          120 - (shootDist - startDist) + 12,
          -276 - 2,
          -48 - 12,
          3,
          48,
          276 - 6,
          -(120-(shootDist - startDist)),
          emptyHopperTime
        };
        MotorCommands = new double[] { 
          autoDriveSpeed, 
          autoDriveSpeed,
          autoDriveSpeed,
          autoDriveSpeed,
          autoDriveSpeed,
          autoDriveSpeed,
          autoDriveSpeed,
          autoDriveSpeed,
          autoDriveSpeed,
          autoDriveSpeed
        };
        break;

      }
      case CENTER: {
        startDist = 23.5;
        Mode = new driveMode[] {
          driveMode.PAUSE, // pause at the start to allow launcher to spin up 
          driveMode.DRIVE,
           driveMode.EJECT,
           driveMode.TURN,
           driveMode.DRIVE,
           driveMode.TURN,
           driveMode.DRIVE,
           driveMode.PAUSE,
           driveMode.DRIVE,
           driveMode.TURN,
           driveMode.DRIVE,
           driveMode.EJECT           
        }; 
          
        Magnitude = new double[] { 
          1, // pause for 1 second at the start to allow launcher to spin up
          shootDist - startDist,
          emptyHopperTime,
          -90,
          127,
          -90,
          -93,
          4,
          48,
          138,
          -(120-(shootDist - startDist)),
          emptyHopperTime

        };
        MotorCommands = new double[] { 
          autoDriveSpeed,
          autoDriveSpeed, 
          autoDriveSpeed,
          autoDriveSpeed,
          autoDriveSpeed,
          autoDriveSpeed,
          autoDriveSpeed,
          autoDriveSpeed,
          autoDriveSpeed,
          autoDriveSpeed,
          autoDriveSpeed,
          autoDriveSpeed,

        };
        break;

      }
      case LEFT: {
        startDist = 47;
        Mode = new driveMode[] {
           driveMode.DRIVE,
           driveMode.EJECT,
           driveMode.DRIVE,
           driveMode.TURN,
           driveMode.DRIVE,
           driveMode.TURN,
           driveMode.DRIVE,
           driveMode.PAUSE,
           driveMode.DRIVE,
           driveMode.TURN,
           driveMode.DRIVE,
           driveMode.EJECT
        }; 
          
        Magnitude = new double[] { 
          shootDist - startDist,
          emptyHopperTime,
          33-(shootDist - startDist),
          -132,
          205,
          -90,
          -93,
          4,
          48,
          138,
          -(120-(shootDist - startDist)),
          emptyHopperTime
        };
        MotorCommands = new double[] { 
          autoDriveSpeed, 
          autoDriveSpeed,
          autoDriveSpeed,
          autoDriveSpeed,
          autoDriveSpeed,
          autoDriveSpeed,
          autoDriveSpeed,
          autoDriveSpeed,
          autoDriveSpeed,
          autoDriveSpeed,
          autoDriveSpeed,
          autoDriveSpeed
        };
        break;
      }
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

    if (safetyFaultActive) {
      safeState(); // Set all motors to zero
      return; // Skip the rest of the function
    }
    else {
      setLauncherSpeed(slowLaunchRPM); // Keep launcher spinning during autonomous
    }

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
      double stepMagnitude = Magnitude[stepIdx];
      if (stepMagnitude < 0) {
        // backwards
        forward = false;
        stepMagnitude = -stepMagnitude; // use absolute value for calculations
      }

      System.out.println("Magnitude: " + stepMagnitude);
      System.out.println("Motor Command: " + MotorCommands[stepIdx]);
 
      /* Convert motor speed command to inches per second */
      double v_command_ips = k * MotorCommands[stepIdx];
 
      //System.out.println("Velocity Command: " + v_command_ips + " in/s");
 
      /* Define wheel distance to travel */
      switch (Mode[stepIdx]) {
          case DRIVE:
          
            /* DRIVE works in terms of distance
            (both wheels moving together) */
            distance = stepMagnitude;
 
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
            // Calibrate Turn Adjustment
            distance = trackwidth * Math.PI * 1.15 * stepMagnitude / 360.0;
            break;
 
          case EJECT:
            /* No ramp needed for ejecting */
            accel_rate = 9999;
 
            /* EJECT magnitude is not actually distance, but time instead */
            distance = stepMagnitude;            
            break;
 
          case PAUSE:
          
            /* PAUSE is not actually distance, but time instead */
            distance = stepMagnitude;
            
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
        t_total_s = stepMagnitude;
      } else if (Mode[stepIdx] == driveMode.PAUSE) { // If drive mode is PAUSE, override time with PAUSE time
        t_total_s = stepMagnitude;
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
              /* Launch fuel */
              // Assumes launcher is already spinning
              hopperMotor.set(launchHopperSpeed);
              SmartDashboard.putNumber("Eject Time Commanded", motorCommand); // Display commanded eject time
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
        System.out.println("Step " + (stepIdx+1) + " Complete");     
        stepIdx = stepIdx + 1; // Go to the next step (in the next loop)
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

//Teleop Motor Speeds, Fast and Slow
    if(driverController.getLeftBumper()==true) {
      driveFactor = driveSpeedBoost; 
    } else {
      driveFactor= driveSpeedNormal; 
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

    /* Op controls */

    //Turning speed Control only change in the IF commands
    double frontSpeed = 0;
    double backSpeed = 0;
    //^^^^ Dont change this one
    //launcherMotor.set(opController.getTwist());

  
  // intake: launch motor (-) | hopper motor (-)
  // launch: launch motor (-) | hopper motor (+)
  // unstick: launch motor (0) | hopper motor (-)
  // empty hopper: launch motor (+) | hopper motor (+)

    // convert joystick flipper axis to a launch speed multiplier between 0 and 1
    double launchSpeedMult =  -0.5 * opController.getRawAxis(3) + 0.5; // convert axis to 0 to 1
    
    // if the launch speed multiplier is below a certain threshold, turn launcher off
    double offDeadband = 0.1; // deadband threshold for launcher off, can be adjusted based on joystick flipper sensitivity
    if (launchSpeedMult < offDeadband) {
      launcherTargetRPM = 0;
    }
    // set the launch speed
    else {
      double norm = (launchSpeedMult - offDeadband) / (1.0 - offDeadband);
      norm = Math.max(0.0, Math.min(1.0, norm)); // clamp just in case
      launcherTargetRPM = slowLaunchRPM + norm * (fastLaunchRPM - slowLaunchRPM);
    }

    // main telop button controls, priority from top to bottom (if multiple buttons pressed)
    if (opController.getRawButton(2))
    {
      // launch speed adjustment, between slow and fast launch speeds, based on the joystick flipper axis
      frontSpeed = launcherTargetRPM;
      backSpeed = launchHopperSpeed;
      // If launch speed is zero while button is held, print a throttled warning to prompt flipper adjustment
      if (launcherTargetRPM == 0.0) {
        printThrottled("Warning: No launch! Turn on joystick flipper.");
      }
    }

    // trigger to intake
    else if(opController.getRawButton(1))
    {
      frontSpeed = launcherTargetRPM;
      backSpeed = IntakeHopperSpeed;
    }
    else if (opController.getRawButton(3))
    {
      // Unstick
      frontSpeed = launcherTargetRPM;
      backSpeed = unstickHopperSpeed;
    }
    else if (opController.getRawButton(5))
    {

      // Reverse launcher to empty hopper
      frontSpeed = EmptyFrontRPM;

      // Wait to empty hopper until launcher has reversed direction, to prevent jamming
      if (isAtSpeed) {
        backSpeed = EmptyHopperSpeed;  
      }
      
      
    }
    else
    {
      // Action for no buttons pressed
      // Do not change away from 0 
      frontSpeed = launcherTargetRPM;
      backSpeed = 0;
    }
  
    setLauncherSpeed(frontSpeed);
    hopperMotor.set(backSpeed);

    /* Motor Test Code, comment out when not testing */
    // double testAxis = driveFilter.calculate(driverController.getRawAxis(1)); // y-axis, left joystick
    // double testCommand = testAxis*0.25-0.75;
    // launcherMotor.set(testCommand);
    // if (testAxis < -0.1) {
    //   hopperMotor.set(0.75);
    // }
    // printThrottled("Test Motor Command: " + testCommand);
    /* Test Code End */
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
