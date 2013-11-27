/*
 * File: TwoWheeledRobot.java
 * Class Provided for the assignment
 * 
 * Modified by:
 * Hadi Sayar, Student ID: 260531679 
 * Antonio D'Aversa, Student ID: 260234498
 * 
 */

import lejos.nxt.NXTRegulatedMotor;

public class TwoWheeledRobot {
	public static final double DEFAULT_LEFT_RADIUS = Startup.WHEELRADIUS; // 2.21
	public static final double DEFAULT_RIGHT_RADIUS = Startup.WHEELRADIUS; ;
	public static final double DEFAULT_WIDTH = Startup.WHEELWIDTH;
	private NXTRegulatedMotor leftMotor, rightMotor;
	private double leftRadius, rightRadius, width;
	private double forwardSpeed;
	private double rotationSpeed;

	/**
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @param width
	 * @param leftRadius
	 * @param rightRadius
	 */
	public TwoWheeledRobot(NXTRegulatedMotor leftMotor,
			NXTRegulatedMotor rightMotor, double width, double leftRadius,
			double rightRadius) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.leftRadius = leftRadius;
		this.rightRadius = rightRadius;
		this.width = width;
		// this.leftMotor.setAcceleration(100);
		// this.rightMotor.setAcceleration(100);
	}

	/**
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 */
	public TwoWheeledRobot(NXTRegulatedMotor leftMotor,
			NXTRegulatedMotor rightMotor) {
		this(leftMotor, rightMotor, DEFAULT_WIDTH, DEFAULT_LEFT_RADIUS,
				DEFAULT_RIGHT_RADIUS);
	}

	/**
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @param width
	 */
	public TwoWheeledRobot(NXTRegulatedMotor leftMotor,
			NXTRegulatedMotor rightMotor, double width) {
		this(leftMotor, rightMotor, width, DEFAULT_LEFT_RADIUS,
				DEFAULT_RIGHT_RADIUS);
	}

	// accessors
	public double getDisplacement() {
		return (leftMotor.getTachoCount() * leftRadius + rightMotor
				.getTachoCount() * rightRadius)
				* Math.PI / 360.0;
	}

	public double getHeading() {
		return (leftMotor.getTachoCount() * leftRadius - rightMotor
				.getTachoCount() * rightRadius)
				/ width;
	}

	/**
	 * 
	 * @param data
	 */
	public void getDisplacementAndHeading(double[] data) {
		int leftTacho, rightTacho;
		leftTacho = leftMotor.getTachoCount();
		rightTacho = rightMotor.getTachoCount();

		data[0] = (leftTacho * leftRadius + rightTacho * rightRadius) * Math.PI
				/ 360.0;
		data[1] = (leftTacho * leftRadius - rightTacho * rightRadius) / width;
	}

	public NXTRegulatedMotor getRightMotor() {
		return this.rightMotor;
	}

	public NXTRegulatedMotor getLeftMotor() {
		return this.leftMotor;
	}

	// mutators

	/**
	 * 
	 * @param wheelRadii
	 * @param width
	 */
	public void setRobotParts(double wheelRadii, double width) {
		this.leftRadius = wheelRadii;
		this.rightRadius = wheelRadii;
		this.width = width;
	}

	/**
	 * 
	 * @param speed
	 */
	public void setForwardSpeed(double speed) {
		this.forwardSpeed = speed;
		setSpeeds(forwardSpeed, rotationSpeed);
	}

	/**
	 * 
	 * @param speed
	 */
	public void setRotationSpeed(double speed) {
		this.rotationSpeed = speed;
		setSpeeds(forwardSpeed, rotationSpeed);
	}

	/**
	 * 
	 * @param f
	 */
	// moves robot forward until told to stop in...
	public void goForward(int f) {
		if (f % 2 == 0) {
			this.leftMotor.forward();
			this.rightMotor.forward();
		} else {
			this.rightMotor.forward();
			this.leftMotor.forward();
		}
	}

	// stop method
	/**
	 * 
	 * @param f
	 */
	public void stop(int f) {
		leftMotor.setAcceleration(1000);
		rightMotor.setAcceleration(1000);
		leftMotor.stop(true);
		rightMotor.stop(false);
		leftMotor.setAcceleration(500);
		rightMotor.setAcceleration(500);
	}

	// Added two methods to rotate the robot either Clockwise or CountClockwise
	// below
	public void rotateClockwise() {
		// LCD.drawString("Rotating", 0, 7);
		leftMotor.setSpeed((int) rotationSpeed);
		rightMotor.setSpeed((int) rotationSpeed);
		leftMotor.forward();
		rightMotor.backward();
	}

	public void rotateCounterClockwise() {
		leftMotor.setSpeed((int) rotationSpeed);
		rightMotor.setSpeed((int) rotationSpeed);
		leftMotor.backward();
		rightMotor.forward();
	}

	/**
	 * 
	 * @param forwardSpeed
	 * @param rotationalSpeed
	 */
	public void setSpeeds(double forwardSpeed, double rotationalSpeed) {
		double leftSpeed, rightSpeed;

		this.forwardSpeed = forwardSpeed;
		this.rotationSpeed = rotationalSpeed;

		leftSpeed = (forwardSpeed + rotationalSpeed * width * Math.PI / 360.0)
				* 180.0 / (leftRadius * Math.PI);
		rightSpeed = (forwardSpeed - rotationalSpeed * width * Math.PI / 360.0)
				* 180.0 / (rightRadius * Math.PI);

		// set motor directions
		if (leftSpeed > 0.0)
			leftMotor.forward();
		else {
			leftMotor.backward();
			leftSpeed = -leftSpeed;
		}

		if (rightSpeed > 0.0)
			rightMotor.forward();
		else {
			rightMotor.backward();
			rightSpeed = -rightSpeed;
		}

		// set motor speeds
		if (leftSpeed > 900.0)
			leftMotor.setSpeed(900);
		else
			leftMotor.setSpeed((int) leftSpeed);

		if (rightSpeed > 900.0)
			rightMotor.setSpeed(900);
		else
			rightMotor.setSpeed((int) rightSpeed);
	}

	// added methods to work with Odometry Correction
	public void leftStop() {
		leftMotor.stop(true);
	}

	public void rightStop() {
		rightMotor.stop(true);

	}

}
