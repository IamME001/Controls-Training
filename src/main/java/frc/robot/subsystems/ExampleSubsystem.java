// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  //declares Xbox controller and A button
  private final XboxController controller;
  private final JoystickButton ButtonA;

  //declares motors
  private final CANSparkMax motor;
  private final CANSparkMax motor2;
  
  //declares talon
  private final WPI_TalonSRX talon;

  //declares ir sensors
  private final AnalogPotentiometer ir0;
  private final AnalogPotentiometer ir1;
  private final AnalogPotentiometer ir2;
  
  //timer for motors running time
  Timer timer_m;

  //timer for talon running time
  Timer timer_t;

  //boolean to check if motor is running
  private boolean running_m;

  //boolean to check if talon is running
  private boolean running_t;
  
  public ExampleSubsystem() {

    //initializes Xbox controller and A button
    controller = new XboxController(0);
    ButtonA = new JoystickButton(controller, XboxController.Button.kA.value);

    //initializes motors
    motor = new CANSparkMax(3, MotorType.kBrushless);
    motor2 = new CANSparkMax(16, MotorType.kBrushless);
    //motor2.follow(motor); //makes motor 2 turn with the other motor so they run the same
    
    //intitializes talon
    talon = new WPI_TalonSRX(8);
    talon.configFactoryDefault();

    //initializes ir sensors
    ir0 = new AnalogPotentiometer(0);
    ir1 = new AnalogPotentiometer(1);
    ir2 = new AnalogPotentiometer(2);
    
    //initializes timers
    timer_m = new Timer();
    timer_t = new Timer();

    //initializes running booleans
    running_m = false;
    running_t = false;
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    /*
    runs the motors at 1 speed and talon at 0.2% speed when A button pressed
    to launch the ball and run the talon to move the next ball up at the same time
    */

    //if the A button is pressed and the motor isn't already running,
    //start the timer and change the boolean running to true
    if(!running_m && controller.getAButtonPressed()) {
      running_m = true;
      timer_m.start();
    }
    //if the boolean running is true, run the motors at 1 speed and the talon at 0.2% speed
    else if(running_m) {
      motor.set(-0.3);
      motor2.set(-0.3);
      talon.set(ControlMode.PercentOutput, -0.1);
    }
    //if the boolean running is true and and the timer has been running for 2 seconds,
    //stop and reset the timer, set running to false, and stop the motors and talon
    else if(running_m && timer_m.hasElapsed(2)) {
      timer_m.stop();
      timer_m.reset();
      running_m = false;
      motor.set(0);
      motor2.set(0);
      talon.set(0);
    }
    
    
    /*
    runs talon if the lower IR sensor detects a ball and the upper IR sensor does not,
    //moving the balls up as they enter, as well as making sure that they don't fall out of the robot
    */

    //if the talon isn't already running, the lower IR sensor detects a ball, and the upper IR sensor doesn't
    //detect a ball, set the boolean running to true
    if(!running_t && ir0.get() > 0.4 && ir2.get() < 0.4) {
      running_t = true;
    }
    //if the boolean running is true, run the talon at 0.2 speed
    else if(running_t) {
      talon.set(ControlMode.PercentOutput, -0.1);
    }
    //if the boolean running is true and and the upper IR sensor detects a ball,
    //set running to false and stop the talon
    else if(running_t && ir2.get() > 0.4) {
      running_t = false;
    }
  
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
