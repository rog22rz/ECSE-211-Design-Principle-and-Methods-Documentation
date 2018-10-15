package ca.mcgill.ecse211.lab3;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
/**
 * THis class is the main class that runs the robot. 
 * 
 * @author Boyang Ma 
 * @author Roger Zhang
 */
public class Lab3 {
	
	// Motor Sensor Objects, and Robot related parameters
	  public static final EV3LargeRegulatedMotor leftMotor =
	      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	  public static final EV3LargeRegulatedMotor rightMotor =
	      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	  private static final Port usPort = LocalEV3.get().getPort("S1");
	  
	  /**
	   * Initialize the wheel radius and the wheel base(TRACK), the track was modified between 10.2 cm to 
	   *  and 8.85 cm in order to make the robot go straight and turn with a relatively perfect. 
	   * 
	   * @param WHEEL_RAD Wheel radius
	   * @param TRACK Wheel base
	   */
	  public static final double WHEEL_RAD = 2.0;
	  public static final double TRACK = 8.85;
	  
      public static void main(String[] args) throws OdometerExceptions {
		  
	  
	  // Setup ultrasonic sensor
	    // There are 4 steps involved:
	    // 1. Create a port object attached to a physical port (done already above)
	    // 2. Create a sensor instance and attach to port
	    // 3. Create a sample provider instance for the above and initialize operating mode
	    // 4. Create a buffer for the sensor data

	  @SuppressWarnings("resource") // Because we don't bother to close this resource
	  SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
	  SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
	                                                              // this instance
	  float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are
	                                                         // returned
	  int buttonChoice;
	  
	  // Odometer related objects
	    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); 
	    Navigation navigation = new Navigation(odometer); 
	    Obstacle obstacle = new Obstacle(odometer);                        
	    Display odometryDisplay = new Display(lcd); 
	    
	    
	    do {
			// clear the display
			lcd.clear();

			// ask the user whether the motors should drive in a square or float
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("       |        ", 0, 1);
			lcd.drawString(" Float | Lab 3  ", 0, 2);
			lcd.drawString("motors |        ", 0, 3);
			lcd.drawString("       |        ", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) {
			// Float the motors
			leftMotor.forward();
			leftMotor.flt();
			rightMotor.forward();
			rightMotor.flt();

			// Display changes in position as wheels are (manually) moved

			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();

		} 
		else {
			// clear the display
			lcd.clear();

			// ask the user whether odometery correction should be run or not
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString(" Navi- | with   ", 0, 1);
			lcd.drawString(" gate  | obst-  ", 0, 2);
			lcd.drawString("       | acle   ", 0, 3);
			lcd.drawString("       |        ", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
			
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();
			
			if (buttonChoice == Button.ID_LEFT) {
				// Start Navigation and display threads
				Thread Navigation = new Thread(navigation);
				Navigation.start();

			}  
			else if (buttonChoice == Button.ID_RIGHT) {
				// Start Obstacle and display threads
				UltrasonicPoller usPoller = new UltrasonicPoller (usDistance, usData, obstacle);
				Thread Obstacle = new Thread(obstacle);
				Obstacle.start(); 
				usPoller.start();
			}
	  
        }
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
      }
}
