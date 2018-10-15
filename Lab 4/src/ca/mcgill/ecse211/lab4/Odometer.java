/**
 * This class is meant as a skeleton for the odometer class to be used.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 */


package ca.mcgill.ecse211.lab4;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
/**
 * This class is the odometer class. 
 * 
 * @author Boyang Ma
 * @author Roger Zhang
 */
public class Odometer extends OdometerData implements Runnable {

  private OdometerData odoData;
  private static Odometer odo = null; // Returned as singleton

  // Motors and related variables
  private int leftMotorTachoCount;
  private int rightMotorTachoCount;
  private int leftMotorLastTachoCount;
  private int rightMotorLastTachoCount;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  private final double TRACK;
  private final double WHEEL_RAD;

  private double[] position; // Declare a double array for storing the coordinates. 


  private static final long ODOMETER_PERIOD = 25; // odometer update period in ms

  /**
   * This is the default constructor of this class. It initiates all motors and variables once.It
   * cannot be accessed externally.
   * 
   * @param leftMotor
   * @param rightMotor
   * @throws OdometerExceptions
   */
  private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
    odoData = OdometerData.getOdometerData(); // Allows access to x,y,z
                                              // manipulation methods
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    // Reset the values of x, y and z to 0
    odoData.setXYT(0, 0, 0);
    // Set the MotorTachoCounts to 0. 
    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;
    this.leftMotorLastTachoCount = 0;
    this.rightMotorLastTachoCount = 0;

    this.TRACK = TRACK;
    this.WHEEL_RAD = WHEEL_RAD;

  }

  /**
   * This method is meant to ensure only one instance of the odometer is used throughout the code.
   * 
   * @param leftMotor
   * @param rightMotor
   * @return new or existing Odometer Object
   * @throws OdometerExceptions
   */
  public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, final double TRACK, final double WHEEL_RAD)
      throws OdometerExceptions {
    if (odo != null) { // Return existing object
      return odo;
    } else { // create object and return it
      odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
      return odo;
    }
  }

  /**
   * This class is meant to return the existing Odometer Object. It is meant to be used only if an
   * odometer object has been created
   * 
   * @return error if no previous odometer exists
   */
  public synchronized static Odometer getOdometer() throws OdometerExceptions {

    if (odo == null) {
      throw new OdometerExceptions("No previous Odometer exits.");

    }
    return odo;
  }

  /**
   * This method is where the logic for the odometer will run. Use the methods provided from the
   * OdometerData class to implement the odometer.
   * 
   * @param distL left wheel displacement 
   * @param distR right wheel displacement 
   * @param deltaD robot displacement 
   * @param deltaT delta theta 
   * @param dX displacement in X direction
   * @param dY displacement in Y direction
   */
  // run method (required for Thread)
  public void run() {
    long updateStart, updateEnd;
    double distL, distR, deltaD, deltaT, dX, dY; 
    // Get last tacho counts. 
    leftMotorLastTachoCount = leftMotor.getTachoCount();
    rightMotorLastTachoCount = rightMotor.getTachoCount();
    
    while (true) {
      updateStart = System.currentTimeMillis();
      
      // Get current tacho counts and the current position. 
      leftMotorTachoCount = leftMotor.getTachoCount();
      rightMotorTachoCount = rightMotor.getTachoCount();
      position = getXYT(); 
      
      // TODO Calculate new robot position based on tachometer counts
      /**
       * Compute the left wheel and right wheel displacements using the circumference equation. 
       * However, the tahco counts are in degrees. Therefore needs to divided by 360 degrees.  
       */
      
      distL = Math.PI * ( WHEEL_RAD) * (leftMotorTachoCount - leftMotorLastTachoCount) / 180; 
      distR = Math.PI * ( WHEEL_RAD) * (rightMotorTachoCount - rightMotorLastTachoCount) / 180;
      // Save tacho counts for next iteration. 
      leftMotorLastTachoCount = leftMotorTachoCount; 
      rightMotorLastTachoCount = rightMotorTachoCount; 
      
      // TODO Update odometer values with new calculated values
      /**
       * Compute the robot displacement and the change in angle of heading. Note the angles are in radians.
       * Therefore it has to be converted to degrees to be stored as the coordinates.  
       */
      
      deltaD = 0.5 * (distL + distR); 
      deltaT = (distL - distR)/TRACK; 
      deltaT = Math.toDegrees(deltaT); 
      position[2] += deltaT; 
      
      // Compute the X and Y displacement using the heading angle. 
      dX = deltaD * Math.sin(Math.toRadians(position[2])); 
    	  dY = deltaD * Math.cos(Math.toRadians(position[2])); 
    	  
      odo.update(dX, dY, deltaT);   // Updating the current position. 

      // this ensures that the odometer only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
  }

}

