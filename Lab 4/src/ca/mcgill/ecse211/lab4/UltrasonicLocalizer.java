package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.lab4.*;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

/**
 * This class uses the ultrasonic sensor to localize the zero degree position. 
 * 
 * @author Boyang Ma
 * 
 */

public class UltrasonicLocalizer implements UltrasonicController, Runnable {
	
	public enum LoclizerType {FALLINGEDGE, RISINGEDGE}  // For Lab4 class to run the two mmehtod. 
	public LoclizerType loclizerType;
	
	
	private static final int ROTATE_SPEED = 45;
	private static final double AngleOfAdjustment = 90; 
	
	public Odometer odo;
	
	private int distance;
	
	private double fallingEdgeFirstAngle; 
	private double fallingEdgeSecodnAngle; 
	private double RisingEdgeFirstAngle; 
	private double RisingEdgeSecondAngle; 
	
	private static int FILTEROUT = 3;
	private int filterCont;
	private float previousDistance;
	private SampleProvider usSensor;
	private float[] usData;
	
	public UltrasonicLocalizer(Odometer odo,SampleProvider usSensor, float[] usData, LoclizerType loclizerType){ //constructor
		this.odo = odo;
		this.loclizerType = loclizerType;
		this.usData = usData;
		this.usSensor = usSensor; 
		filterCont = 0;
		previousDistance = 100;
	}
	/** 
	 * Run method for running the Ultrasonic Localizer Thread. 
	 */
	public void run(){
		if (this.loclizerType == LoclizerType.FALLINGEDGE) {
			fallingEdge();
		}
		else if (this.loclizerType == LoclizerType.RISINGEDGE) {
			risingEgde();
		}
	}
	/**
	 *  The robot will detect the falling edge of the wall to figure out where it is
	 *  relative to the origin. Switch direction when the robot detects the wall. 
	 */
	public void fallingEdge () {
		Lab4.leftMotor.setSpeed(ROTATE_SPEED);
		Lab4.rightMotor.setSpeed(ROTATE_SPEED);
		
		// Turn away from the wall until it sees a wall again 
		while (getFilteredDistance() < 30) {
			Lab4.leftMotor.forward();
			Lab4.rightMotor.backward();
		}
		while (getFilteredDistance() > 30) {
			Lab4.leftMotor.forward();
			Lab4.rightMotor.backward();
		}

		// Get the first angle
		fallingEdgeFirstAngle = odo.getXYT()[2];

		// Turn away from the wall from above until it sees a wall again
		while (getFilteredDistance() < 30) {
			Lab4.leftMotor.backward();
			Lab4.rightMotor.forward();
		}
	
		while (getFilteredDistance() > 30) {
			Lab4.leftMotor.backward();
			Lab4.rightMotor.forward();
		}
	    // Stop the robot
		Lab4.leftMotor.stop(true);
		Lab4.rightMotor.stop(false);
		
		// Get the second angle
		fallingEdgeSecodnAngle = odo.getXYT()[2];
		
		// Calculate the angle that the robot should turn to 0 degree. 
		if(fallingEdgeFirstAngle > fallingEdgeSecodnAngle){
			fallingEdgeFirstAngle = fallingEdgeFirstAngle - 360;
	    }
		
		double averageAngle = (fallingEdgeFirstAngle + fallingEdgeSecodnAngle)/2;
		double relaiveZeroAngle =  fallingEdgeSecodnAngle - averageAngle - 12; // Maybe +; 

		turnTo(relaiveZeroAngle);

		// Mover closer to the (0, 0) point for next part of the demo
		Lab4.leftMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, AngleOfAdjustment), true);
		Lab4.rightMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, AngleOfAdjustment), false);
		Lab4.leftMotor.rotate(convertDistance(Lab4.WHEEL_RAD, 8), true);
		Lab4.rightMotor.rotate(convertDistance(Lab4.WHEEL_RAD, 8), false);
		Lab4.leftMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, AngleOfAdjustment), true);
		Lab4.rightMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, AngleOfAdjustment), false);
		Lab4.leftMotor.rotate(convertDistance(Lab4.WHEEL_RAD, 8), true);
		Lab4.rightMotor.rotate(convertDistance(Lab4.WHEEL_RAD, 8), false);

		odo.setXYT(0, 0, 0);

		Lab4.leftMotor.stop(true);
		Lab4.rightMotor.stop(false);
	}
	
	/**
	 *  The robot will detect the rising edge of the wall to figure out where it is
	 *  relative to the origin. Switch direction when the robot can't see the wall
	 *  anymore.  
	 */
	public void risingEgde() {
		Lab4.leftMotor.setSpeed(ROTATE_SPEED);
		Lab4.rightMotor.setSpeed(ROTATE_SPEED);
		
		// Turn into the wall until it can't see the wall 
		while (getFilteredDistance() > 30) {
			Lab4.leftMotor.backward();
			Lab4.rightMotor.forward();
		}

		while (getFilteredDistance() < 30) {
			Lab4.leftMotor.backward();
			Lab4.rightMotor.forward();
		}

		// Get the first angle
		RisingEdgeFirstAngle = odo.getXYT()[2];

		// Turn back into the wall until it can't see the wall again 
		while (getFilteredDistance() > 30) {
			Lab4.leftMotor.forward();
			Lab4.rightMotor.backward();
		}

		while (getFilteredDistance() < 30) {
			Lab4.leftMotor.forward();
			Lab4.rightMotor.backward();
		}
		
		// Stop the robot
		Lab4.leftMotor.stop(true);
		Lab4.rightMotor.stop(false);
		
		// Get the second angle
		RisingEdgeSecondAngle = odo.getXYT()[2];
		
		// Calculate the angle that the robot should turn to 0 degree. 
		if(RisingEdgeFirstAngle > RisingEdgeSecondAngle){
			RisingEdgeFirstAngle = RisingEdgeFirstAngle - 360;
		}
		
		double avgAngleRE = (RisingEdgeFirstAngle + RisingEdgeSecondAngle)/2;
		double relaiveZeroAngle =  RisingEdgeSecondAngle - avgAngleRE - 38;

		turnTo(relaiveZeroAngle);
        
		// Mover closer to the (0, 0) point for next part of the demo
		Lab4.leftMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, AngleOfAdjustment), true);
		Lab4.rightMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, AngleOfAdjustment), false);
		Lab4.leftMotor.rotate(convertDistance(Lab4.WHEEL_RAD, 8), true);
		Lab4.rightMotor.rotate(convertDistance(Lab4.WHEEL_RAD, 8), false);
		Lab4.leftMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, AngleOfAdjustment), true);
		Lab4.rightMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, AngleOfAdjustment), false);
		Lab4.leftMotor.rotate(convertDistance(Lab4.WHEEL_RAD, 8), true);
		Lab4.rightMotor.rotate(convertDistance(Lab4.WHEEL_RAD, 8), false);

		odo.setXYT(0, 0, 0);

		Lab4.leftMotor.stop(true);
		Lab4.rightMotor.stop(true);
		
	}
	
	/**
	  * Make the robot turn a certain angle.
	  * @param theta The angle that the robot suppose to turn
	  */
	 public void turnTo(double theta) {
		 Lab4.leftMotor.setSpeed(ROTATE_SPEED);
		 Lab4.rightMotor.setSpeed(ROTATE_SPEED);

		 Lab4.leftMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, theta), true);
		 Lab4.rightMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, theta), false);
	 }
	 
	/**
	  * This method allows the conversion of a distance to the total rotation of each wheel need to
	  * cover that distance.
	  * 
	  * @param radius
	  * @param distance
	  * @return
	  */
   private static int convertDistance(double radius, double distance) {
		    return (int) ((180.0 * distance) / (Math.PI * radius));
	 }
	 
   /**
    * This method allows the conversion of an angle to the total rotation of each wheel need to
	  * cover that distance.
    * @param radius
    * @param width
    * @param angle
    * @return
    */
   private static int convertAngle(double radius, double width, double angle) {
		    return convertDistance(radius, Math.PI * width * angle / 360.0);
	 }
   
   // Distant filter needed because of the uncertainty of the sensor. 
   private float getFilteredDistance() {
		usSensor.fetchSample(usData, 0);
		float distance = (int)(usData[0]*100.0);
		float result = 0;
		// Filter out bad distance
		if (distance > 50 && filterCont < FILTEROUT) {
			filterCont ++;
			result = previousDistance;
		} else if (distance > 50){
			result = 50; 
		} else {
			// Distance went below 50, reset everything.
			filterCont = 0;
			result = distance;
		}
		previousDistance = distance;
		return result;
	}
   
	@Override
	public void processUSData(int distance) {
		// TODO Auto-generated method stub
		this.distance = distance;
	}

	@Override
	public int readUSDistance() {
		// TODO Auto-generated method stub
		return this.distance;
	}

}
