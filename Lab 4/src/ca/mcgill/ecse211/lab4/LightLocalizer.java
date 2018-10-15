package ca.mcgill.ecse211.lab4;


import java.util.concurrent.TimeUnit;

import ca.mcgill.ecse211.lab4.*;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

/**
 * This class will use the light sensor to make the robot go 
 * back to the origin
 * 
 * @author Boyang Ma
 *
 */
public class LightLocalizer implements UltrasonicController, Runnable {
	
	private static final int FORWARD_SPEED = 45;
	private static final int ROTATE_SPEED = 45;
	
	
	private static final double OFFSET_FROM_SENSOR = 10; //
	
    private double currentX; 
	private double currentY; 
	private double currentTheta; 
	private int numOfLine; 
	
	private double firstAngle; 
	private double secondAngle; 
	private double thirdAngle; 
	private double forthAngle; 
	private double thetaX; 
	private double thetaY; 
	private double possitionInX; 
	private double possitionInY; 
	
	
	public Odometer odo;
	private int distance;
	private static SampleProvider samples;
    private float [] collectedData; 
	
	private static final EV3ColorSensor redSensor = new EV3ColorSensor(LocalEV3.get().getPort("S2"));
	
	
	public LightLocalizer(Odometer odo) throws OdometerExceptions{ //constructor
		this.odo = Odometer.getOdometer();
		redSensor.setCurrentMode("Red");
		this.samples = redSensor.getRedMode(); 
		this.collectedData = new float[redSensor.sampleSize()];
	}
	
	

	public void run() {
		// TODO Auto-generated method stub
		try {
			lightLocalizer();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
	
	/**
	 * This method will make the robot to rotate around 360 degrees to detect
	 * the dark lines in order to get the angles and calculate the position
	 * 
	 * @throws InterruptedException
	 */
	public void lightLocalizer() throws InterruptedException{
		
		odo.setXYT(0, 0, 0);
		TimeUnit.SECONDS.sleep(2);
		numOfLine = 0; 
		// Keep rotating
		while (numOfLine < 4) { 
			while(true){
				samples.fetchSample(collectedData, 0); 
				Lab4.leftMotor.forward();
				Lab4.rightMotor.backward();
				Lab4.leftMotor.setSpeed(ROTATE_SPEED);
				Lab4.rightMotor.setSpeed(ROTATE_SPEED);
				
				// Record the angles. 
				if(collectedData[0] < 0.30) {
					Sound.beep(); 
					numOfLine = numOfLine + 1; 
					if(numOfLine == 1) {
						firstAngle = odo.getXYT()[2]; 
					}
					else if(numOfLine == 2) {
						secondAngle = odo.getXYT()[2]; 
					}
	                else if(numOfLine == 3) {
	                	    thirdAngle = odo.getXYT()[2]; 
					}
	                else if(numOfLine == 4) {
	                   	forthAngle = odo.getXYT()[2]; 
					}
				}
	            // Exist the loop when complete the rotation and detection. 
				if(odo.getXYT()[2] > 357 &&  odo.getXYT()[2] < 360) {
					Lab4.leftMotor.stop(true);
					Lab4.rightMotor.stop(false);
					break;
				}
			}
		}
		
		// Calculate the current position
		thetaX = thirdAngle - firstAngle;
		thetaY = forthAngle - secondAngle;
		
		possitionInX = -OFFSET_FROM_SENSOR*(Math.cos(Math.PI*thetaX/360));
		possitionInY = -OFFSET_FROM_SENSOR*(Math.cos(Math.PI*thetaY/360));

		odo.setXYT(possitionInX, possitionInY, 0);

		travelTo(0,0); // Travel to origin 
		turnTo(-45);   // Turn back to 0 degree. 
	}
	
	/**
	  * Calculates the distance and the angle to reach the coordinates. 
	  * 
	  * @param x The x coordinate that the robot has to travel to 
	  * @param y The Y coordinate that the robot has to travel to
	  */
	public void travelTo( double x, double y) { 
		 
		 double[] coordinates = new double[3]; 
		 double distanceToDisX; 
		 double distanceToDisY; 
		 double angleToTurn; 
		 double distanceTravel; 
		 double calculatedAngle; 
		 
		 // Get the current coordinate. 
		 coordinates = odo.getXYT(); 
		 currentX = coordinates[0]; 
		 currentY = coordinates[1]; 
		 currentTheta = coordinates[2]; 
		 
		 // Calculate the distance travel for x and y direction. 
		 distanceToDisX = x - currentX; 
		 distanceToDisY = y - currentY;
		 
		 // Calculate the actual distance that the robot should travel.
		 distanceTravel = Math.hypot(distanceToDisX, distanceToDisY); 
		 
		 // Calculate the actual angle that the robot should turn. 
		 calculatedAngle = Math.toDegrees(Math.atan2(distanceToDisX, distanceToDisY)); 
		 angleToTurn = calculatedAngle - currentTheta; 
		 
		 // Check the condition of the angle to make the correct turns. 
		 if(angleToTurn < -180) {
			 turnTo(angleToTurn + 360); 
		 }
		 else if(angleToTurn > 180 ){
			 turnTo(angleToTurn - 360); 
		 }
		 else {
			 turnTo(angleToTurn);
		 }
		 
		 // Sleep for 1 seconds
		 try {
		      Thread.sleep(1000);
		    } catch (InterruptedException e) {
		      
		 }
		 
		 actualTravelTo(distanceTravel);  
	 }
	
	 /**
	  * Make the robot travel a certain distance.
	  * @param distance length of the distance that the robot suppose to travel
	  */
	 public void actualTravelTo(double distance) {
		 //Go forward. 
		 Lab4.leftMotor.setSpeed(FORWARD_SPEED);
		 Lab4.rightMotor.setSpeed(FORWARD_SPEED);

		 Lab4.leftMotor.rotate(convertDistance(Lab4.WHEEL_RAD, distance), true);
		 Lab4.rightMotor.rotate(convertDistance(Lab4.WHEEL_RAD, distance), false);
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
