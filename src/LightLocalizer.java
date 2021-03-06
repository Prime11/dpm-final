/* LightLocalizer.java
 * This class is used to allow the robot to localize itself to (0, 0, 0.0) using the light sensor
 * 
 * Written by:
 * Hadi Sayar, Student ID: 260531679 
 * Antonio D'Aversa, Student ID: 260234498
 */

import java.util.Stack;

import lejos.nxt.ColorSensor;
import lejos.nxt.LCD;
import lejos.nxt.Sound;

public class LightLocalizer {
	private Odometer odo;
	private TwoWheeledRobot robot;
	private ColorSensor ls;
	private Navigation nav;
	private ColorSensor rls;
	private Stack<Point> inputStack = new Stack<Point>(); // dummy input

	private int fieldSize = 12; // need to pass this another way

	private int firstLight;
	private int count = 0; // counter
	private double[] angles = { 0, 0, 0, 0 }; // initialize the angles to 0
	private double d = 11.0; // adjust this
	private double x = 0;
	private double y = 0;
	private double deltaThetaY, deltaThetaX;

	public LightLocalizer(Odometer odo, ColorSensor ls, ColorSensor rls,
			Navigation nav) {
		this.odo = odo;
		this.robot = odo.getTwoWheeledRobot();
		this.ls = ls;
		this.rls = rls;
		this.nav = nav;
		// this.nav = new Navigation(this.odo, null, null);
		// turn on the light
		ls.setFloodlight(0);
		rls.setFloodlight(0);
	}

	public void doLocalization() {
		// start rotating and clock all 4 gridlines
		// we will rotate counter-clockwise so that we will measure the y first
		// and third and
		// x will be second and last.

		// set the turn speed higher to not detect the same line twice then
		// rotate
		
		//this.nav.turnTo(Math.toRadians(330));
		robot.setSpeeds(350, 350);
		robot.rotateCounterClockwise();

		while (count < 4) {
			firstLight = this.ls.getNormalizedLightValue();

			// if the normalize light value is below a certain threshold we have
			// detected a line
			if (firstLight < 440) {
				// beep whenever a line is detected.
				Sound.beep();
				// sleep here to not detect the same line twice (may be useless)
				try {
					Thread.sleep(500);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				angles[count] = this.odo.getAng();
				count++;
				LCD.clear(5);
			}
			// stop the robot after the last line is detected.
			if (count == 4) {
				robot.stop(0);
			}
		}

		// reset speeds of the motors
		robot.getLeftMotor().setSpeed(200);
		robot.getRightMotor().setSpeed(200);

		// do trig to compute (0,0) and 0 degrees

		// Math taken from the tutorial slides.
		// deltaTheta's are in degrees
		deltaThetaX = angles[1] - angles[3];
		deltaThetaY = angles[0] - angles[2];

		x = -1 * d * Math.cos(Math.toRadians(deltaThetaY) / 2);
		y = -1 * d * Math.cos(Math.toRadians(deltaThetaX) / 2);

		double theta = 0;

		theta = 180 - angles[0] + deltaThetaY / 2;

		LCD.drawInt((int) (deltaThetaY), 0, 4);
		LCD.drawInt((int) (angles[0]), 0, 5);

		theta = -theta + odo.getAng();

		//IF BATTERIES ARE TOO LOW THE Y-VALUE HAS TO BE NEGATIVE...OTHERWISE POSITIVE - CHECKLIST
		double[] pos = { x, -y, theta};
		boolean[] poop = { true, true, true };

		this.odo.setPosition(pos, poop);

		// ADDED CODE STARTS HERE - IF IT DOESN'T WORK REMOVE THIS.
		// travel to the negative values of x and y to get to the actual origin
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		this.nav.travelTo(true, 0, 0, 0, inputStack, inputStack);
		// adjust the angle so that the the robot facing the y direction
		this.nav.turnTo(Math.toRadians(0));

		this.robot.setSpeeds(75, 75);
		try {
			Thread.sleep(200);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		this.robot.rotateCounterClockwise();
		Sound.buzz();
	

		boolean right = false, left = false;
		int i = 0;
		while (i != 2) {
			if (this.ls.getNormalizedLightValue() < 420 && left == false) {
				this.robot.getLeftMotor().stop(true);
				left = true;
				i++;
			}
			if (this.rls.getNormalizedLightValue() < 440 && right == false) {
				this.robot.getRightMotor().stop(true);
				right = true;
				i++;
			}
		}
		
		
		//sleep maybe

		//this.robot.getLeftMotor().forward();

		// localization ends here

		// set the current position of the robot
		/*double[] position = computePosition(1, fieldSize);
		boolean[] updateOdo = { true, true, true };

		// the final updated position of the odometer with respect to the
		// initial starting tile
		this.odo.setPosition(position, updateOdo);*/
	}

}
