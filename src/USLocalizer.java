/* USLocalizer.java
 * This class is used to allow the robot to localize itself using both RISING EDGE and FALLING EDGE methods.

 * 
 * Written by:
 * Hadi Sayar, Student ID: 260531679 
 * Antonio D'Aversa, Student ID: 260234498
 */

import lejos.nxt.LCD;
import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;

public class USLocalizer {
	public enum LocalizationType {
		FALLING_EDGE, RISING_EDGE
	};

	public static int ROTATION_SPEED = 300;
	public static int FORWARD_SPEED = 250;
	public static int WALL_THRESHOLD = 50;

	private Odometer odo;

	private TwoWheeledRobot robot;
	private UltrasonicSensor us;
	private LocalizationType locType;
	private Navigation nav;
	private UltraSensor ultra;
	private Detection oDetect;

	/**
	 * Constructor
	 * @param odo
	 * @param us
	 * @param locType
	 */
	public USLocalizer(Odometer odo, UltrasonicSensor us,
			LocalizationType locType) {
		this.odo = odo;
		this.robot = odo.getTwoWheeledRobot();
		this.us = us;
		this.locType = locType;
		this.nav = new Navigation(this.odo/*, this.oDetect*/, this.ultra);

		// switch off the ultrasonic sensor
		// us.off();
	}

	/**
	 * Performs Localization
	 */
	public void doLocalization() {
		double[] pos = new double[3];
		int filterCount = 0;
		int filterMax = 2; // may vary with battery life apparently
		double initAngle = 0;
		double angleA = 0, angleB = 0;
		double gammaA = 0;
		double deltaAngle = 0;
		
		//set the rotation speeds and get the robots current position
		robot.setSpeeds(FORWARD_SPEED, ROTATION_SPEED);
		odo.getPosition(pos);
		
		if (locType == LocalizationType.FALLING_EDGE) {
			// rotate the robot until it sees no wall and latch the angle
			robot.rotateClockwise();
			//acquire and filter the data
			while (getFilteredData() > 0) {
				if (getFilteredData() > 30 && filterCount < filterMax) {
					filterCount++;
				} else if (filterCount == filterMax) {
					Sound.beep();
					odo.setPosition(new double[] { 0.0, 0.0, 0.0 },
							new boolean[] { true, true, true });
					odo.getPosition(pos);
					initAngle = pos[2];
					break;
				}
			}
			// keep rotating until the robot sees a wall, then latch the angle
			robot.rotateClockwise();
			filterCount = 0;
			//acquire and filter the data
			while (getFilteredData() > 0) {
				if (getFilteredData() <= 29 && filterCount < filterMax) {
					filterCount++;
				} else if (filterCount == filterMax) {
					Sound.beep();
					odo.getPosition(pos);
					filterCount = 0;
					angleA = pos[2];// -angleA;
					break;
				}
			}

			// switch direction and wait until it sees no wall, latch the angle and compute gammaB
			robot.rotateCounterClockwise();
			
			//acquire and filter the data
			while (getFilteredData() > 0) {
				if (getFilteredData() > 30 && filterCount < filterMax) {
					filterCount++;
				} else if (filterCount == filterMax) {
					Sound.beep();
					odo.getPosition(pos);
					filterCount = 0;
					gammaA = angleA - pos[2];
					break;
				}
			}
			// keep rotating until the robot sees a wall, then latch the angle and compute angleB
			robot.rotateCounterClockwise();
			//acquire and filter the data
			while (getFilteredData() > 0) {
				if (getFilteredData() <= 29 && filterCount < filterMax) {
					filterCount++;
				} else if (filterCount == filterMax) {
					Sound.beep();
					filterCount = 0;
					odo.getPosition(pos);
					angleB = angleA + 360 - pos[2];
					break;
				}
			}

			// angleA is clockwise from angleB, so assume the average of the
			// angles to the right of angleB is 45 degrees past 'north'
			
			//Compute where to turn such that the robot faces the y direction and turn to that direction.
			odo.getPosition(pos);
			deltaAngle = angleB - 90 - gammaA;// 45a
			nav.turnTo(Math.toRadians(deltaAngle));

			// update the odometer position (example to follow:)
			odo.setPosition(new double[] { 0.0, 0.0, 0.0 }, new boolean[] {
					true, true, true });
		} else {
			/*
			 * The robot should turn until it sees the wall, then look for the
			 * "rising edges:" the points where it no longer sees the wall. This
			 * is very similar to the FALLING_EDGE routine, but the robot will
			 * face toward the wall for most of it.
			 */

			// rotate the robot until it sees a wall and latch the angle
			robot.rotateCounterClockwise();
			//acquire and filter the data
			while (getFilteredData() > 0) {
				if (getFilteredData() <= 29 && filterCount < filterMax) {
					filterCount++;
				} else if (filterCount == filterMax) {
					Sound.beep();
					odo.setPosition(new double[] { 0.0, 0.0, 0.0 },
							new boolean[] { true, true, true });
					odo.getPosition(pos);
					initAngle = pos[2];
					break;
				}
			}
			// keep rotating until the robot sees no wall, then latch the angle
			robot.rotateCounterClockwise();
			filterCount = 0;
			//acquire and filter the data
			while (getFilteredData() > 0) {
				if (getFilteredData() > 30 && filterCount < filterMax) {
					filterCount++;
				} else if (filterCount == filterMax) {
					Sound.beep();
					odo.getPosition(pos);
					filterCount = 0;
					angleA = pos[2];// -angleA;
					break;
				}
			}

			// switch direction and wait until it sees a wall and compute gammaA using the newly recorded angle pos[2]
			robot.rotateClockwise();
			//acquire and filter the data
			while (getFilteredData() > 0) {
				if (getFilteredData() <= 29 && filterCount < filterMax) {
					filterCount++;
				} else if (filterCount == filterMax) {
					Sound.beep();
					odo.getPosition(pos);
					filterCount = 0;
					gammaA = angleA - pos[2];
					break;
				}
			}
			// keep rotating until the robot sees no wall, compute angleB using the newly recorded angle pos[2]
			robot.rotateClockwise();
			//acquire and filter the data
			while (getFilteredData() > 0) {
				if (getFilteredData() > 30 && filterCount < filterMax) {
					filterCount++;
				} else if (filterCount == filterMax) {
					Sound.beep();
					filterCount = 0;
					odo.getPosition(pos);
					angleB = angleA - 45 - pos[2];
					break;
				}
			}

			// angleA is clockwise from angleB, so assume the average of the
			// angles to the right of angleB is 45 degrees past 'north'
			
			//Compute where to turn such that the robot faces the y direction and turn to that direction.
			odo.getPosition(pos);
			deltaAngle = angleB - 90 - gammaA;// 45a
			nav.turnTo(Math.toRadians(deltaAngle));

			// update the odometer position (example to follow:)
			odo.setPosition(new double[] { 0.0, 0.0, odo.getAng() }, new boolean[] {
					true, true, true });
		}
	}


	/**
	 * Gets FilteredData and returns Distance between sensor and brick
	 * @return
	 */
	private int getFilteredData() {
		int distance;

		// do a ping
		us.ping();

		// wait for the ping to complete
		try {
			Thread.sleep(15);
		} catch (InterruptedException e) {
		}

		// there will be a delay here
		distance = us.getDistance();

		if (distance > WALL_THRESHOLD) {
			distance = WALL_THRESHOLD;
		}
		LCD.drawInt(distance, 0, 7);

		return distance;
	}

}
