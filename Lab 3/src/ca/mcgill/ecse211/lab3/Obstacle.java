package ca.mcgill.ecse211.lab3;

/**
 * This class uses the methods from navigation class to avoid the obstacles placed on the way 
 * to the final coordinate. 
 * 
 * @author Boyang Ma
 * @author Roger Zhang 
 */
public class Obstacle extends Thread implements UltrasonicController {
	
	private static final int FORWARD_SPEED = 250;
    private static final int ROTATE_SPEED = 150;
	private static final double TILE_SIZE = 30.48;
	private static double ANGLE_To_AVOID;
	private int distance;
	private Odometer odo;  

	
	public double currentX; 
	public double currentY; 
	public double currentTheta; 
	public double destinationX; 
	public double destinationY; 
	
	
	public Obstacle(Odometer odo) {
		this.odo = odo; 
	}
	
	/**
	  * Run method that calls the methods for the robot. 
	  */
	public void run () {
		
		 travelTo(1,1);
		 travelTo(0,2);
		 travelTo(2,2);
		 travelTo(2,1);
		 travelTo(1,0);
		 
	}
	/**
	  * This method calculates the distance and the angle to reach the coordinates, 
	  * but at the same time it detects whether if there is an obstacle at the front.  
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
		
		this.destinationX = x; 
		this.destinationY = y; 
		 
		// Get the current coordinate. 
		coordinates = odo.getXYT(); 
		currentX = coordinates[0]; 
		currentY = coordinates[1]; 
		currentTheta = coordinates[2]; 
		
		// Calculate the distance travel for x and y direction. 
		distanceToDisX = x * TILE_SIZE - currentX; 
		distanceToDisY = y * TILE_SIZE - currentY;
		
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
		 
	    // Sleep for 2 seconds
		try {
		     Thread.sleep(2000);
	    } catch (InterruptedException e) {
		      
	    }
		// Check the distance from the obstacle
	    if(this.distance < 34) { 
			avoidObstacle((int) distance); 
		}
		else {
			actualTravelTo(distanceTravel); 
		}	
	}
	
	 /**
	  * Make the robot travel a certain distance.
	  * @param distance length of the distance that the robot suppose to travel
	  */
	public void actualTravelTo(double distance) {
		//Go forward. 
	    Lab3.leftMotor.setSpeed(FORWARD_SPEED);
		Lab3.rightMotor.setSpeed(FORWARD_SPEED);

		Lab3.leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, distance), true);
		Lab3.rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, distance), false);
	}
	/**
	 * This method make the robot the avoid the obstacles by taking the distance read from the sensor. 
	 * 
	 * @param distance
	 */
	public void avoidObstacle(int distance) { 
		// If the robot is too close to the obstacle, turn sharper and travel enough to avoid the obstacle 
		if( this.distance > 0  && this.distance < 10 ){
			
			ANGLE_To_AVOID = 60; 
			Lab3.leftMotor.setSpeed(ROTATE_SPEED);
			Lab3.rightMotor.setSpeed(ROTATE_SPEED);
			Lab3.leftMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, ANGLE_To_AVOID), true);
			Lab3.rightMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, ANGLE_To_AVOID), false);
			Lab3.leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 25), true);
			Lab3.rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 25), false);
			Lab3.leftMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, ANGLE_To_AVOID), true);
			Lab3.rightMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, ANGLE_To_AVOID), false);
			Lab3.leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 30), true);
			Lab3.rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 30), false);
			
		}
		
		/*If the robot is relatively close to the obstacle, turn less sharper and travel 
		 enough to avoid the obstacle. */
		else if(this.distance > 10 && this.distance < 20 ) { 
			
			ANGLE_To_AVOID = 50; 
			Lab3.leftMotor.setSpeed(ROTATE_SPEED);
			Lab3.rightMotor.setSpeed(ROTATE_SPEED);
			Lab3.leftMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, ANGLE_To_AVOID), true);
			Lab3.rightMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, ANGLE_To_AVOID), false);
			Lab3.leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 25), true);
			Lab3.rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 25), false);
			Lab3.leftMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, ANGLE_To_AVOID), true);
			Lab3.rightMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, ANGLE_To_AVOID), false);
			Lab3.leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 30), true);
			Lab3.rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 30), false);	
			
		}
		/* If the robot is too far to the obstacle, turn even less sharper and travel 
		   longer to avoid the obstacle.*/
		else if(this.distance > 20 && this.distance < 34 ) { 
			
			ANGLE_To_AVOID = 40; 
			Lab3.leftMotor.setSpeed(ROTATE_SPEED);
			Lab3.rightMotor.setSpeed(ROTATE_SPEED);
			Lab3.leftMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, ANGLE_To_AVOID), true);
			Lab3.rightMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, ANGLE_To_AVOID), false);
			Lab3.leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 30), true);
			Lab3.rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 30), false);
			Lab3.leftMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, ANGLE_To_AVOID), true);
			Lab3.rightMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, ANGLE_To_AVOID), false);
			Lab3.leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 35), true);
			Lab3.rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 35), false);
			
		}
		
		try {
		      Thread.sleep(2000);
		    } catch (InterruptedException e) {
		}
		
		// Double check if the robot is still close to the wall
		if(this.distance < 34) { // changed from 36
			avoidObstacle(distance); 
		}
		else{ 
			travelTo(destinationX, destinationY); 
		}
	}
	
	/**
	  * Make the robot turn a certain angle.
	  * @param theta The angle that the robot suppose to turn
	  */
	public void turnTo(double theta) {
		 Lab3.leftMotor.setSpeed(ROTATE_SPEED);
		 Lab3.rightMotor.setSpeed(ROTATE_SPEED);

		 Lab3.leftMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, theta), true);
		 Lab3.rightMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, theta), false);
	}
	 
	/**
	  * Check the status of the robot, whether if it is moving or not.
	  * @return boolean value for the status of the robot
	  */
	public boolean isNavigating() {
		 if(Lab3.leftMotor.isMoving() && Lab3.rightMotor.isMoving()) { 
			 return true; 
		 }
		 return false; 
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
