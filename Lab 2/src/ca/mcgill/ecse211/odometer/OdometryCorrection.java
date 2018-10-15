/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.Sound;
/**
 * This class is the odometer class. 
 * 
 * @author Boyang Ma
 * @author Roger Zhang
 */

public class OdometryCorrection implements Runnable {
  /**
   * Initialize the constants, declare the class variables. 
   * 
   * @param CORRECTION_PERIOD
   * @param LINESPERSIDE
   * @param odometer
   * @param odometerdata
   * @param redSensor
   * @param OFFSETX The distance from the light sensor to the center of the wheels
   * @param OFFSETY The distance from the light sensor to the center of the wheels
   */
  private static final long CORRECTION_PERIOD = 10;
  private static final int LINESPERSIDE  = 3;
  private static final double OFFSETX  = 4.5;   
  private static final double OFFSETY  = 4.5;   
  private Odometer odometer;
  private OdometerData odometerdata;
  private EV3ColorSensor redSensor;

  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection(EV3ColorSensor redSensor) throws OdometerExceptions {

    this.odometer = Odometer.getOdometer();
    this.redSensor = redSensor; 
    redSensor.setCurrentMode("Red");

  }

  /**
   * Here is where the odometer correction code should be run.
   * 
   * @throws OdometerExceptions
   * @param correctionStart 
   * @param correctionEnd
   * @param coordinates current coordinates
   * @param currentValue current sample value from the light sensor
   * @param threshold color intensity to compare with, set as 0.27
   * @param numOfLine number of line passed
   */
  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;
    double[] coordinates = new double[3];
    float[] currentValue = new float [1]; 
    float threshold = (float) 0.27; 
    int numOfLine = 0; 
    redSensor.setCurrentMode("Red");  // Use the red mode for the light sensor. 
    
    
    while (true) {
    	
      correctionStart = System.currentTimeMillis();
     
      redSensor.fetchSample(currentValue, 0); // Get the current sample from the light sensor. 
     
      coordinates = odometer.getXYT();  // Get the current coordinate from the odometer. 
      
      /**
       * If the current color intensity is smaller to the threshold, it means it is a dark line. 
       * Therefore, number of line increase by one. 
       */
      if(currentValue[0] < threshold ) { 
    	  	numOfLine = numOfLine + 1; 
    	  	
    	  	/**
    	  	 * If the absolute value of the cosine theta is bigger than 0.5, it means the robot is traveling in Y direction. 
    	  	 */
    	  	if(Math.abs(Math.cos(Math.toRadians(coordinates[2]))) > 0.5) { 
    	  		Sound.beep();
    	  		
    	  		/**
        	  	 * If the cosine theta is negative, it means the robot is traveling in negative Y direction.
        	  	 * Therefore, decrease the y value, but need to add the offset.  
        	  	 * 
        	  	 * If the cosine theta is positive, it means the robot is traveling in positive Y direction.
        	  	 * Therefore, increase the y value, but need to deduct the offset.  
        	  	 */
    	  		if(Math.cos(Math.toRadians(coordinates[2])) < 0) {
    	  			//double offset = Math.cos(coordinates[2]) * 2.5; 
    	  			odometer.setXYT(coordinates[0], (LINESPERSIDE-numOfLine)*30.48 + OFFSETY, coordinates[2]);
    	  		}
    	  		else if(Math.cos(Math.toRadians(coordinates[2])) >= 0) {
    	  			//double offset = Math.cos(coordinates[2]) * 2.5; 
    	  			odometer.setXYT(coordinates[0], (numOfLine - 1)*30.48 - OFFSETY, coordinates[2]);
    	  		}
    	  	
    	  	}
    	  	
    		/**
    	  	 * If the absolute value of the sine theta is bigger than 0.5, it means the robot is traveling in Y direction. 
    	  	 */
    	  	else if(Math.abs(Math.sin(Math.toRadians(coordinates[2]))) > 0.5 ) { 
    	  		Sound.beep();
    	  		
    	  		/**
        	  	 * If the sine theta is negative, it means the robot is traveling in negative X direction.
        	  	 * Therefore, decrease the x value, but need to add the offset.  
        	  	 * 
        	  	 * If the cosine theta is positive, it means the robot is traveling in positive X direction.
        	  	 * Therefore, increase the x value, but need to deduct the offset.  
        	  	 */
    	  		if(Math.sin(Math.toRadians(coordinates[2])) < 0) {
    	  			//double offset = Math.sin(coordinates[2]) * 1.5; 
    	  			odometer.setXYT((LINESPERSIDE-numOfLine)*30.48 + OFFSETX , coordinates[1], coordinates[2]);
    	  		}
    	  		else if(Math.sin(Math.toRadians(coordinates[2])) >= 0) {
    	  			//double offset = Math.sin(coordinates[2]) * 1.5; 
    	  			odometer.setXYT((numOfLine - 1)*30.48 - OFFSETX, coordinates[1], coordinates[2]);
    	  		}
    	  	
    	  	}
    	  	numOfLine = numOfLine % LINESPERSIDE; // Reset the number of line after completing one side of the rectangle. 
    	  
      }
      
      // TODO Trigger correction (When do I have information to correct?)
      
      // TODO Calculate new (accurate) robot position

      // TODO Update odometer with new calculated (and more accurate) vales

      //odometer.setXYT(0.3, 19.23, 5.0);

      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here
        }
      }
    }
  }
}
