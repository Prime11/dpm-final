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
	private Detection detector;
	private Stack<Point>  emptyStack = new Stack<Point>();
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
	public Navigation(Odometer odometer, UltraSensor uSonic, Wrangler pathF, Detection detector) {
		this.odo = odometer;
		this.robot = odo.getTwoWheeledRobot();
		wheelRadii = Startup.WHEELRADIUS;  // wheel radius
		width = Startup.WHEELWIDTH;  // width between wheels
		this.pathF = pathF;
		//set the acceleration to prevent slipping.
		this.robot.getLeftMotor().setAcceleration(1000);
		this.robot.getRightMotor().setAcceleration(1000);
		this.uSonic = uSonic;
		this.detector = detector;

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
	public Stack<Point> travelTo(boolean localizing, double x, double y, int threshold, Stack<Point> inputStack, Stack<Point> backPedalStack) {
		// Sets Navigation to true
		isNav = true;
	
		// set the motor speeds
		this.robot.setForwardSpeed(150);
		// clear and write to the LCD
		this.odo.getPosition(pos);
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

		while (Math.sqrt(Math.pow(x - this.odo.getX(),2) + Math.pow(y - this.odo.getY(), 2)) >= 3.0 
				/*|| (localizing && this.pathF.insideGreenZone(this.odo.getX(), this.odo.getY()))*/) {
			// 15 cms before reaching its destination and if the angle Theta of the odometer is off, recalculate the heading and
			// change the direction of the odometer.
			//correct the first if
			
			if (Math.sqrt(Math.pow(x - this.odo.getX(), 2)
					+ Math.pow(y - this.odo.getY(), 2)) <= 15.0) {
				this.odo.getPosition(pos);
				currentX = this.odo.getX();
				currentY = this.odo.getY();
				deltaX = x - currentX;
				deltaY = y - currentY;
				// compute the new heading, compare it to the robot's current
				// theta and turnTo if necessary
				heading = Math.atan2(deltaX, deltaY);
				// TODO FIX THIS SHIT!!! (Specifically the if statement below)
				// IF we are more than 5 degrees off course, stop and turnTo
				// correct.
				if (angleDiff(this.odo.getAng(), Math.toDegrees(heading)) > 5) {
					//LCD.clear(3);
					//LCD.drawInt((int)angleDiff(this.odo.getAng(), Math.toDegrees(heading)), 0, 3);
					this.robot.stop(0);
					turnTo(heading);
				}
			}
			
			//Paused to allow odometryCorrection to perform a correction
			if (localizing || odo.isPaused() == false && uSonic.getDist() > this.threshold) {
				LCD.clear(4);
				LCD.drawInt(uSonic.getDist(), 0, 4);
				this.robot.getLeftMotor().setSpeed(300);
				this.robot.getRightMotor().setSpeed(300);
				this.robot.getLeftMotor().forward();
				this.robot.getRightMotor().forward();
			}
			else if (!localizing && uSonic.getDist() <= this.threshold){
				this.robot.stop(0);
				Point newP = new Point();
				if (this.odo.getAng() < 10 || this.odo.getAng() > 350){
					newP = Point.downAdjacentPoint(inputStack.peek());
					this.pathF.insertIntoDangerList(inputStack.peek().x, inputStack.peek().y, newP.x, newP.y);
					inputStack.pop();
					inputStack.push(Point.upAdjacentPoint(Point.upAdjacentPoint(backPedalStack.peek())));
					inputStack.push(Point.upAdjacentPoint(Point.upAdjacentPoint(Point.rightAdjacentPoint(backPedalStack.peek()))));
					inputStack.push(Point.rightAdjacentPoint(backPedalStack.peek()));
				} else if (this.odo.getAng() > 80 || this.odo.getAng() < 100){
					newP = Point.leftAdjacentPoint(inputStack.peek());
					this.pathF.insertIntoDangerList(inputStack.peek().x, inputStack.peek().y, newP.x, newP.y);
					inputStack.pop();
					inputStack.push(Point.rightAdjacentPoint(Point.rightAdjacentPoint(backPedalStack.peek())));
					inputStack.push(Point.rightAdjacentPoint(Point.rightAdjacentPoint(Point.upAdjacentPoint(backPedalStack.peek()))));
					inputStack.push(Point.upAdjacentPoint(backPedalStack.peek()));
				} else if (this.odo.getAng() > 170 || this.odo.getAng() < 190){
					newP = Point.upAdjacentPoint(inputStack.peek());
					this.pathF.insertIntoDangerList(inputStack.peek().x, inputStack.peek().y, newP.x, newP.y);
					inputStack.pop();
					inputStack.push(Point.downAdjacentPoint(Point.downAdjacentPoint(backPedalStack.peek())));
					inputStack.push(Point.downAdjacentPoint(Point.downAdjacentPoint(Point.leftAdjacentPoint(backPedalStack.peek()))));
					inputStack.push(Point.leftAdjacentPoint(backPedalStack.peek()));
				} else if (this.odo.getAng() > 260 || this.odo.getAng() < 280){
					newP = Point.rightAdjacentPoint(inputStack.peek());
					this.pathF.insertIntoDangerList(inputStack.peek().x, inputStack.peek().y, newP.x, newP.y);
					inputStack.pop();
					inputStack.push(Point.leftAdjacentPoint(Point.leftAdjacentPoint(backPedalStack.peek())));
					inputStack.push(Point.leftAdjacentPoint(Point.leftAdjacentPoint(Point.downAdjacentPoint(backPedalStack.peek()))));
					inputStack.push(Point.downAdjacentPoint(backPedalStack.peek()));
				}
				else {
					//newP = Point.rightAdjacentPoint(inputStack.peek());
					//this.pathF.insertIntoDangerList(inputStack.peek().x, inputStack.peek().y, newP.x, newP.y);
					//inputStack.pop();
					inputStack.push(Point.leftAdjacentPoint(Point.leftAdjacentPoint(backPedalStack.peek())));
					inputStack.push(Point.leftAdjacentPoint(Point.leftAdjacentPoint(Point.downAdjacentPoint(backPedalStack.peek()))));
					inputStack.push(new Point(backPedalStack.peek().x - 30.48*Math.cos(Math.toRadians(this.odo.getAng())), backPedalStack.peek().y + 30.48*Math.sin(Math.toRadians(this.odo.getAng())), false));
					inputStack.push(new Point(backPedalStack.peek().x + 0, backPedalStack.peek().y + 2*30.48*Math.cos(Math.toRadians(this.odo.getAng())), false));
					inputStack.push(new Point(backPedalStack.peek().x + 30.48*Math.sin(Math.toRadians(this.odo.getAng())), backPedalStack.peek().y + 30.48*Math.cos(Math.toRadians(this.odo.getAng())), false));
				}
				this.pathF.generatePath(false, backPedalStack.peek().x,
						backPedalStack.peek().y, inputStack.peek().x,
						inputStack.peek().y, inputStack);
				inputStack.push(backPedalStack.peek());
				inputStack.push(backPedalStack.pop());
				break;
			}
		}
		
		this.robot.stop(0);
		if (!localizing && this.detector.hasThreeBlocks()){
			this.detector.resetNumberOfBlocks();
			inputStack.clear();
			backPedalStack.clear();
			
			inputStack.push(new Point(this.pathF.getFinishX(), this.pathF.getFinishY(), false));
			inputStack.push(new Point(this.pathF.getFinishX(), this.pathF.getFinishY(), false));
			
			//Localizing is true so that it ignores block avoidance and avoids an infinite loop
			//travelTo(true, this.pathF.getFinishX()*30.48, this.pathF.getFinishY()*30.48, 15, emptyStack, emptyStack);
			this.robot.stop(0);
			
			//this.pathF.generatePath(true, backPedalStack.peek().x, backPedalStack.peek().y, this.pathF.getFinishX(), this.pathF.getFinishY(), inputStack);
		}
		//Sound.beep();
		
		LCD.drawString("Exited the loop", 0, 3);
		LCD.clear(3);
		// When it exits the loop, STOP
		
		// 1 second cat-nap
		
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
		robot.getLeftMotor().setAcceleration(250);
		robot.getRightMotor().setAcceleration(250);
		this.robot.getLeftMotor().rotate(convertAngle(wheelRadii, width, deltaTheta), true);
		this.robot.getRightMotor().rotate(-convertAngle(wheelRadii, width, deltaTheta), false);
		this.robot.stop(0);
		this.robot.getLeftMotor().setAcceleration(900);
		this.robot.getRightMotor().setAcceleration(900);
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
	public static int convertDistance(double radius, double distance) {
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
	public static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
