/* Navigation.java
 * This class is used to allow the robot to navigate to specified coordinates using the odometer.
 * 
 * This navigation class was used from lab 3 and modified for the purposes of lab4 to work with the provided odometer.
 * 
 * Written by:
 * Hadi Sayar, Student ID: 260531679 
 * Antonio D'Aversa, Student ID: 260234498
 */

import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.Sound;

import java.util.Stack;

public class Navigation {
	private TwoWheeledRobot robot;

	private Odometer odo;
	NXTRegulatedMotor leftMotor = Motor.A, rightMotor = Motor.B;
	private boolean isNav = false;
	private double currentX = 0, currentY = 0, currentTheta = 0;
	private double deltaX = 0, deltaY = 0, deltaTheta = 0, heading;
	private static final double PI = Math.PI;
	private double wheelRadii, width;
	private double[] pos = new double[3];
	private boolean isTurning = false;
	private UltraSensor uSonic;
	private int threshold = 15;
	private Wrangler pathF;
	
	// Navigation constructor
	/**
	 * 
	 * @param odometer
	 * @param uSonic
	 */
	public Navigation(Odometer odometer, UltraSensor uSonic) {
		this.odo = odometer;
		this.robot = odo.getTwoWheeledRobot();
		wheelRadii = Startup.WHEELRADIUS; // wheel radius
		width = Startup.WHEELWIDTH; // width between wheels
		this.pathF = new Wrangler(odometer, robot, null, uSonic, leftMotor);
		//set the acceleration to prevent slipping.
		this.robot.getLeftMotor().setAcceleration(500);
		this.robot.getRightMotor().setAcceleration(500);
		this.uSonic = uSonic;
	}

	/**
	 * 
	 * @param odometer
	 * @param uSonic
	 * @param pathF
	 */
	public Navigation(Odometer odometer, UltraSensor uSonic, Wrangler pathF) {
		this.odo = odometer;
		this.robot = odo.getTwoWheeledRobot();
		wheelRadii = Startup.WHEELRADIUS;  // wheel radius
		width = Startup.WHEELWIDTH;  // width between wheels
		this.pathF = pathF;
		//set the acceleration to prevent slipping.
		this.robot.getLeftMotor().setAcceleration(1000);
		this.robot.getRightMotor().setAcceleration(1000);
		this.uSonic = uSonic;
	}

	/**
	 * 
	 * @param wheelRadii
	 * @param width
	 */
	public void setRobotParts(double wheelRadii, double width){
		this.width = width;
		this.wheelRadii = wheelRadii;
	}

	public boolean hasBlock(){
		return false;
	}

	/**
	 * 
	 * @param threshold
	 */
	public void setThreshold(int threshold){
		this.threshold = threshold;
	}
	
	// Travel to method which determines the distance that robot needs to travel
	/**
	 * 
	 * @param x
	 * @param y
	 * @param threshold
	 * @param inputStack
	 * @param backPedalStack
	 * @return
	 */
	public Stack<Point> travelTo(double x, double y, int threshold, Stack<Point> inputStack, Stack<Point> backPedalStack) {
		// Sets Navigation to true
		isNav = true;
	
		// set the motor speeds
		this.robot.setForwardSpeed(150);
		// clear and write to the LCD
		odo.getPosition(pos);
		// Calculate the heading of the robot and turn towards it
		currentX = this.odo.getX();
		currentY = this.odo.getY();
		deltaX = x - currentX;
		deltaY = y - currentY;
		heading = Math.atan2(deltaX, deltaY);
		turnTo(heading);
		this.odo.getPosition(pos);
		
		// Until the robot is at its destination within 1 cm, it will move
		// forward in the heading's direction

		while (Math.sqrt(Math.pow(x - this.odo.getX(),2) + Math.pow(y - this.odo.getY(), 2)) >= 7.0) {
			// 15 cms before reaching its destination and if the angle Theta of the odometer is off, recalculate the heading and
			// change the direction of the odometer.
			//correct the first if
			//LCD.clear(5);
			//LCD.drawInt((int) Math.sqrt(Math.pow(x - this.odo.getX(),2) + Math.pow(y - this.odo.getY(), 2)), 0 , 5);
			//LCD.drawInt((int) x, 0, 5);
			//LCD.drawInt((int) y, 6, 5);
			
			LCD.clear(3);
			LCD.drawString("First Loop", 0, 3);
			//LCD.clear(5);
			//LCD.drawInt((int) angleDiff(this.odo.getAng(), Math.toDegrees(heading)), 0, 5);
			//LCD.drawInt((int)this.odo.getAng(), 0, 5);
			//LCD.drawInt((int)Math.toDegrees(heading), 7, 5);
			
			if( Math.sqrt(Math.pow(x - this.odo.getX(),2) + Math.pow(y - this.odo.getY(), 2)) <= 15.0){
				odo.getPosition(pos);
				currentX = this.odo.getX();
				currentY = this.odo.getY();
				deltaX = x - currentX;
				deltaY = y - currentY;
				
				//compute the new heading, compare it to the robot's current theta and turnTo if necessary
				heading = Math.atan2(deltaX, deltaY);
				
				
				// TODO FIX THIS SHIT!!! (specificlly the if statement below)
				//IF we are more than 5 degrees off course, stop and turnTo correct.
				if(angleDiff(this.odo.getAng(), Math.toDegrees(heading)) > 10){
					
					this.robot.stop(0);
					turnTo(heading);
				}
				
			}
			
			//Paused to allow odometryCorrection to perform a correction
			if (odo.isPaused() == false && uSonic.getDist() > this.threshold) {
				LCD.clear(4);
				//LCD.drawString("odo is not paused", 0, 4);
				LCD.drawInt(uSonic.getDist(), 0, 4);
				this.robot.getLeftMotor().setSpeed(200);
				this.robot.getRightMotor().setSpeed(200);
				this.robot.getLeftMotor().forward();
				this.robot.getRightMotor().forward();
			}
			else if (uSonic.getDist() <= this.threshold){
				this.robot.stop(0);
				Point newP = new Point();
				if (this.odo.getAng() < 10 || this.odo.getAng() > 350){
					newP = Point.upAdjacentPoint(inputStack.peek());
					this.pathF.insertIntoDangerList(newP.x, newP.y, newP.x, newP.y+1);
				} else if (this.odo.getAng() > 80 || this.odo.getAng() < 100){
					newP = Point.rightAdjacentPoint(inputStack.peek());
					this.pathF.insertIntoDangerList(newP.x, newP.y, newP.x + 1, newP.y);
				} else if (this.odo.getAng() > 170 || this.odo.getAng() < 190){
					newP = Point.downAdjacentPoint(inputStack.peek());
					this.pathF.insertIntoDangerList(newP.x, newP.y, newP.x, newP.y-1);
				} else if (this.odo.getAng() > 260 || this.odo.getAng() < 280){
					newP = Point.leftAdjacentPoint(inputStack.peek());
					this.pathF.insertIntoDangerList(newP.x, newP.y, newP.x-1, newP.y);
				}
				inputStack.pop();
				inputStack.push(backPedalStack.peek());	
				inputStack.push(backPedalStack.peek());
			}
		}
		Sound.beep();
		LCD.drawString("Exited the loop", 0, 3);
		// When it exits the loop, STOP
		this.robot.stop(0);
		// 1 second cat-nap
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// there is nothing to be done here because it is not expected
			// that the odometer will be interrupted by another thread
		}

		// sets navigation to false when it gets to destination
		isNav = false;

		// UPDATE LCD
		// this.odo.getPosition(pos);
		return inputStack;
	}

	//takes input in radians?
	/**
	 * 
	 * @param theta
	 */
	public void turnTo(double theta) {
		isNav = true;
		isTurning = true;
		
		// Finds current heading according to odometer
		currentTheta = pos[2];
		deltaTheta = theta - Math.toRadians(currentTheta);

		// computes acute angle (Shortest turn)
		if (deltaTheta < -PI) {
			deltaTheta += 2 * PI;
		} else if (deltaTheta > PI) {
			deltaTheta -= 2 * PI;
		}

		// Performs rotation
		deltaTheta = Math.toDegrees(deltaTheta);
		this.robot.getLeftMotor().rotate(convertAngle(wheelRadii, width, deltaTheta), true);
		this.robot.getRightMotor().rotate(-convertAngle(wheelRadii, width, deltaTheta), false);
		
		isTurning = false;
	}
	
	//THE INPUT ANGLE MUST BE IN DEGREES
	//Compute the angle difference between the heading and the odometer angles..
	//Return the minimum difference between the two angles
	/**
	 * 
	 * @param odometerAngle
	 * @param newHeading
	 * @return
	 */
	public static double angleDiff(double odometerAngle, double newHeading) {

		if (Math.abs(newHeading - (odometerAngle + 360) % 360) < Math.abs(newHeading - (odometerAngle - 360) % 360)) {
			return Math.abs(newHeading - (odometerAngle + 360) % 360);
		} 
		else {
			return Math.abs(newHeading - (odometerAngle - 360) % 360);
		}

	}

	/**
	 * 
	 * @return
	 */
	boolean isNavigating() {
		return isNav;
	}
	
	//added in order to work with Odometry Correction.
	boolean isTurning(){
		return isTurning;
	}

	// Method "borrowed" from SquareDriver.java
	/**
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	// Method "borrowed" from SquareDriver.java
	/**
	 * 
	 * @param radius
	 * @param width
	 * @param angle
	 * @return
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
