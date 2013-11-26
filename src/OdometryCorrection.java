import lejos.nxt.ColorSensor;
import lejos.nxt.LCD;

/* 
 * OdometryCorrection.java
 * Do not hardcode this!
 */

public class OdometryCorrection extends Thread {
	// private static final long CORRECTION_PERIOD = 5; //may need to be changed
	private Odometer odometer;
	private ColorSensor leftSensor;
	private ColorSensor rightSensor;
	private TwoWheeledRobot robot;
	private Wrangler strangle;

	private boolean motorStop = false, leftStop = false, rightStop = false;

	// for both the arrays before the positions are {X, Y, HEADING}
	private boolean[] update = { true, true, true };
	private double[] position = { 0.0, 0.0, 0.0 };

	// constructor
	/**
	 * 
	 * @param odometer
	 * @param leftSensor
	 * @param rightSensor
	 * @param robot
	 * @param strangle
	 */
	public OdometryCorrection(Odometer odometer, ColorSensor leftSensor,
			ColorSensor rightSensor, TwoWheeledRobot robot, Wrangler strangle) {
		this.odometer = odometer;
		this.leftSensor = leftSensor;
		this.rightSensor = rightSensor;
		this.robot = robot;
		this.strangle = strangle;
		leftSensor.setFloodlight(0);
		rightSensor.setFloodlight(0);
	}

	// run method (required for Thread)
	/**
	 * 
	 */
	public void run() {
		// 0 = red, 1 = green and 2 = blue
		// set the floodlight's of both sensors
		long correctionStart, correctionEnd;

		while (true) {
			correctionStart = System.currentTimeMillis();

			// Sound.beep();
			// Do not correct if the motors have not stopped.
			// add isTurning to the Navigation class.
			while (motorStop == false && this.strangle.isTurning() == false) {

				// to debug
				LCD.drawInt(leftSensor.getNormalizedLightValue(), 0, 5);
				LCD.drawInt(rightSensor.getNormalizedLightValue(), 0, 6);

				// calibrate the Normalized light values so they work for
				// competition day
				if (leftSensor.getNormalizedLightValue() < 430
						&& leftStop == false) {
					this.odometer.setPause(true);
					this.robot.setForwardSpeed(50);
					this.robot.leftStop();

					// this.robot.getLeftMotor().stop();
					// this.robot.getRightMotor().forward();

					// this.odometer.getTwoWheeledRobot().leftStop();
					leftStop = true;
				} else if (rightSensor.getNormalizedLightValue() < 490
						&& rightStop == false) {
					this.odometer.setPause(true);
					this.robot.setForwardSpeed(50);
					// this.robot.getRightMotor().stop();
					this.robot.rightStop();
					// this.robot.getLeftMotor().forward();

					// this.odometer.getTwoWheeledRobot().rightStop();
					rightStop = true;
				} else if (rightStop == true && leftStop == true) {
					// set motorStop = true
					rightStop = false;
					leftStop = false;
					motorStop = true;

				}

			}

			// Apply the correction and then continue travelling.
			if (motorStop == true) {
				// There are 4 correction cases: for 90, 180, 270, and 0
				// degrees.
				// In each case one of the X or Y values can also be corrected
				// but not both.

				position[0] = odometer.getX();
				position[1] = odometer.getY();
				position[2] = odometer.getAng();

				double[] newPosition = new double[3];
				newPosition = computeCorrection(position);

				odometer.setPosition(newPosition, update);

				this.odometer.setPause(false);
				this.robot.getLeftMotor().setSpeed(200);
				this.robot.getRightMotor().setSpeed(200);

				motorStop = false; // reset the boolean for the next correction

			}

			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			// LCD.drawString("" + (correctionEnd - correctionStart), 0, 6);
			// if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
			// sleep for 1 second.
			try {
				Thread.sleep(1000);
				// Thread.sleep(CORRECTION_PERIOD - (correctionEnd -
				// correctionStart));
			} catch (InterruptedException e) {
				// there is nothing to be done here because it is not
				// expected that the odometry correction will be
				// interrupted by another thread
			}
			// }
			// LCD.clear();
		}
	}

	// the data array will be structured as follow {angle, x-cord, y-cord}
	/**
	 * 
	 * @param pos
	 * @return
	 */
	public static double[] computeCorrection(double[] pos) {
		double[] corrected = new double[3];
		double tempY = 0.0, tempX = 0.0;
		double tileLength = 30.48;

		// correct if the heading is 0 degrees
		if (pos[0] > 350 || pos[0] < 10) {
			// correct the angle to 0 degrees since we snapped to line
			corrected[0] = 0.0;

			// x remains the same
			corrected[1] = pos[1];

			// now correct y
			tempY = pos[2];
			tempY = Math.round(tempY / tileLength);

			tempY = tempY * tileLength;
			corrected[2] = tempY;
		}
		// correct x
		else if (pos[0] > 80 && pos[0] < 100) {
			// correct the angle to 90 degrees since we snapped to line
			corrected[0] = 90.0;

			// y remains the same
			corrected[2] = pos[2];

			// now correct x
			tempX = pos[1];
			tempX = Math.round(tempX / tileLength);

			tempX = tempX * tileLength;
			corrected[1] = tempX;
		}
		// correct y
		else if (pos[0] > 170 && pos[0] < 190) {
			// correct the angle to 180 degrees since we snapped to line
			corrected[0] = 180.0;

			// x remains the same
			corrected[1] = pos[1];

			// now correct y
			tempY = pos[2];
			tempY = Math.round(tempY / tileLength);

			tempY = tempY * tileLength;
			corrected[2] = tempY;
		}
		// correct x
		else if (pos[0] > 260 && pos[0] < 280) {
			// correct the angle to 270 degrees since we snapped to line
			corrected[0] = 270.0;

			// y remains the same
			corrected[2] = pos[2];

			// now correct x
			tempX = pos[1];
			tempX = Math.round(tempX / tileLength);

			tempX = tempX * tileLength;
			corrected[1] = tempX;
		}
		return corrected;
	}
}