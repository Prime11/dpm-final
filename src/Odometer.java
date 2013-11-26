/*
 * File: Odometer.java
 * Class Provided for the assignment. Performs the functions of an odometer. 
 * 
 * Modified by: Group 11
 * 
 */

import lejos.util.Timer;
import lejos.util.TimerListener;

public class Odometer implements TimerListener {
	public static final int DEFAULT_PERIOD = 25;
	private TwoWheeledRobot robot;
	private ObjectDetection oDetect;
	private Timer odometerTimer;
	private Navigation nav;
	// position data
	private Object lock;
	private double x, y, theta;
	private double [] oldDH, dDH;
	private UltraSensor ultra;
	private boolean pause;
	
	/**
	 * 
	 * @param robot
	 * @param period
	 * @param start
	 */
	public Odometer(TwoWheeledRobot robot, int period, boolean start) {
		// initialise variables
		this.robot = robot;
		this.nav = new Navigation(this/*, oDetect*/, ultra);
		odometerTimer = new Timer(period, this);
		x = 0.0;
		y = 0.0;
		theta = 0.0;
		oldDH = new double [2];
		dDH = new double [2];
		lock = new Object();
		
		// start the odometer immediately, if necessary
		if (start)
			odometerTimer.start();
	}
	
	/**
	 * 
	 * @param robot
	 */
	public Odometer(TwoWheeledRobot robot) {
		this(robot, DEFAULT_PERIOD, false);
	}
	
	/**
	 * 
	 * @param robot
	 * @param start
	 */
	public Odometer(TwoWheeledRobot robot, boolean start) {
		this(robot, DEFAULT_PERIOD, start);
	}
	
	/**
	 * 
	 * @param robot
	 * @param period
	 */
	public Odometer(TwoWheeledRobot robot, int period) {
		this(robot, period, false);
	}
	
	public void timedOut() {
		robot.getDisplacementAndHeading(dDH);
		dDH[0] -= oldDH[0];
		dDH[1] -= oldDH[1];
		
		// update the position in a critical region
		synchronized (lock) {
			theta += dDH[1];
			theta = fixDegAngle(theta);
			
			x += dDH[0] * Math.sin(Math.toRadians(theta));
			y += dDH[0] * Math.cos(Math.toRadians(theta));
		}
		
		oldDH[0] += dDH[0];
		oldDH[1] += dDH[1];
	}
	
	// accessors
	/**
	 * 
	 * @param pos
	 */
	public void getPosition(double [] pos) {
		synchronized (lock) {
			pos[0] = x;
			pos[1] = y;
			pos[2] = theta;
		}
	}
	
	public TwoWheeledRobot getTwoWheeledRobot() {
		return robot;
	}
	
	public Navigation getNavigation() {
		return this.nav;
	}
	
	// mutators
	/**
	 * 
	 * @param pos
	 * @param update
	 */
	public void setPosition(double [] pos, boolean [] update) {
		synchronized (lock) {
			if (update[0]) x = pos[0];
			if (update[1]) y = pos[1];
			if (update[2]) theta = pos[2];
		}
	}
	
	/**
	 * 
	 * @param wheelRadii
	 * @param width
	 */
	public void setRobotParts(double wheelRadii, double width){
		nav.setRobotParts(wheelRadii, width);
	}

	// Added getter methods to retrieve X, Y and the Angle.
	public double getX(){
		synchronized(lock){
			return x;
		}
	}

	public double getY(){
		synchronized(lock){
			return y;
		}
	}
	
	public double getAng(){
		synchronized(lock){
			return theta;
		}
	}
	
	// static 'helper' methods
	/**
	 * 
	 * @param angle
	 * @return
	 */
	public static double fixDegAngle(double angle) {		
		if (angle < 0.0){
			angle = 360.0 + (angle % 360.0);
		}
		return angle % 360.0;
	}
	
	/**
	 * 
	 * @param a
	 * @param b
	 * @return
	 */
	public static double minimumAngleFromTo(double a, double b) {
		double d = fixDegAngle(b - a);
		
		if (d < 180.0)
			return d;
		else
			return d - 360.0;
	}
	
	/**
	 * 
	 * @param x
	 */
	public void setX(double x) {
		synchronized (lock) {
			this.x = x;
		}
	}
	
	/**
	 * 
	 * @param y
	 */
	public void setY(double y) {
		synchronized (lock) {
			this.y = y;
		}
	}

	/**
	 * 
	 * @param theta
	 */
	public void setTheta(double theta) {
		synchronized (lock) {
			this.theta = theta;
		}
	}
	
	/**
	 * 
	 * @return
	 */
	public boolean isPaused(){
		return pause;
	}
	
	/**
	 * 
	 * @param value
	 */
	public void setPause(boolean value){
		pause = value;
	}
	
}
