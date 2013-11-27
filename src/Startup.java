/* Startup.java 
 * This class will contain the main method of the program to be run on the robot during the competition
 */

import lejos.nxt.*;

import java.util.Arrays;
import java.util.Stack;

import bluetooth.PlayerRole;
import bluetooth.StartCorner;

public class Startup {
	//Constants - change them here instead of changing them EVERYWHERE else!
	public final static double WHEELRADIUS = 1.978;
	public final static double WHEELWIDTH = 17.85;
	
	//Note that the cage motor must be rotated only 7 degrees from a position where it is closed.
	//And the door must be opened enouhg so that the black peice does not interfere with the blocks when dropping them off
	
	
	/**
	 * 
	 * @param args
	 */
	public static void main(String[] args) {
		
		//Set if it uses Bluetooth Program
		boolean useBluetooth = true;
		
		double finalX = 6, finalY = 7;
		//Fix Start Coords WITH STARTID
		double startX = 0, startY = 0;
		int startID = 1;
		double redX1 = 0, redY1 = 7, redX2 = 8, redY2 = 8;
		//TODO: GreenZone avoidance
		double greenX1 = 2, greenY1 = 2, greenX2 = 3, greenY2 = 3;
		//Before it localizes
		double robotWheelRadii = Startup.WHEELRADIUS, robotWidth= Startup.WHEELWIDTH;
		//After it localizes
		double robotWheelRadiiSecond = Startup.WHEELRADIUS, robotWidthSecond = Startup.WHEELWIDTH;
		//Size of the field
		int widthX = 10, widthY = 10;
		boolean [] update = {true, true, true};
		
		if (useBluetooth) {
			int safeZ[];
			int dangerZ[];
			int role;
			int startingCorner;
			//TODO Set ODOMETER For Starting Corner
			bluetooth.BluetoothConnection blue = new bluetooth.BluetoothConnection();
			startID = blue.getTransmission().startingCorner.getId();
			startX = blue.getTransmission().startingCorner.getX();
			startY = blue.getTransmission().startingCorner.getY();
			role = blue.getTransmission().role.getId();
			startingCorner = blue.getTransmission().startingCorner.getId();
			
			if (role == 1) /*Builder*/{
				safeZ = blue.getTransmission().greenZone;
				dangerZ = blue.getTransmission().redZone;
			} else if (role == 2) /*Garbage Collector*/{
				safeZ = blue.getTransmission().redZone;
				dangerZ = blue.getTransmission().greenZone;
			} else {
				safeZ = new int[4];
				dangerZ = new int[4];
			}
			
			LCD.drawString("I'm a " + PlayerRole.lookupRole(role) + "!", 5, 0);
			Sound.systemSound(false, 2);
			greenX1 = safeZ[0];
			greenY1 = safeZ[1];
			greenX2 = safeZ[2];
			greenY2 = safeZ[3];
			redX1 = dangerZ[0];
			redY1 = dangerZ[1];
			redX2 = dangerZ[2];
			redY2 = dangerZ[3];
			startX = StartCorner.lookupCorner(startingCorner).getX();
			startY = StartCorner.lookupCorner(startingCorner).getY();
		}

		LCD.clear();
		//Button.waitForAnyPress();

		TwoWheeledRobot newBot = new TwoWheeledRobot(Motor.A, Motor.B);
		newBot.setRobotParts(robotWheelRadii, robotWidth);
		NXTRegulatedMotor gateMotor = Motor.C;
		// Retrieve the bluetooth signal and identify if the robot will be a
		// builder or a garbage collector

		// initiate the two sensors that will be taken in as input for the
		// odometry correction
		final ColorSensor leftLS = new ColorSensor(SensorPort.S1);
		final ColorSensor rightLS = new ColorSensor(SensorPort.S2);

		// ultrasonic sensor
		UltrasonicSensor us = new UltrasonicSensor(SensorPort.S3);
		final ColorSensor detectionLS = new ColorSensor(SensorPort.S4);
		// sensor to detect objects

		// Initiate the objects for the odometer, the display and odometry
		// correction
		Odometer odometer = new Odometer(newBot, true);
		odometer.setRobotParts(robotWheelRadii, robotWidth);

		LCDInfo lcd = new LCDInfo(odometer);

		//new object detection class to fit our robot
		// initiate the localization object and perform the localization.
		USLocalizer usl = new USLocalizer(odometer, us, USLocalizer.LocalizationType.FALLING_EDGE); 
		usl.doLocalization();
	
		//UltrasonicSensor us2 = new UltrasonicSensor(SensorPort.S3);
		UltrasonicSensor us2 = new UltrasonicSensor(SensorPort.S3);
		UltraSensor usd = new UltraSensor(us2);
		usd.start();
		
		Navigation nav = new Navigation(odometer, usd);
		LightLocalizer lsl = new LightLocalizer(odometer, leftLS, rightLS, nav);
		lsl.doLocalization();
		odometer.setPosition(computePosition(startID, (int) widthX+2), update);
		
		//code for opening gate motor
		//this first rotation to be done right after localization
		gateMotor.rotate(22);
		
		
		newBot.setRobotParts(robotWheelRadiiSecond, robotWidthSecond);
		odometer.setRobotParts(robotWheelRadiiSecond, robotWidthSecond);
		
		//set the current position of the robot
		
		//the final updated position of the odometer with respect to the initial starting tile
		Detection detect = new Detection(detectionLS);
		detect.start();
		Wrangler pathfinder = new Wrangler(odometer, newBot, detect, usd, gateMotor);
	
		//pathfinder.setFinal(finalX, finalY);
		pathfinder.setArenaSize(widthX, widthY);
		pathfinder.setStarter(startX, startY);
		pathfinder.setRedZone(redX1, redY1, redX2, redY2);
		pathfinder.setGreenZone(greenX1, greenY1, greenX2, greenY2);
		pathfinder.insertCornersIntoRedZone(startID);
		pathfinder.setGreenMid();
		pathfinder.createDangerList();
		pathfinder.backPedalStack.push(new Point(startX, startY, false));
		pathfinder.generateSetPath();
		pathfinder.runSimpleCourse();
		
		//the 2 rotates below are to get the blocks into the safe zone due to the offset of the cage
				
		newBot.getLeftMotor().rotate(nav.convertDistance(WHEELRADIUS, 10.0), true);
		newBot.getRightMotor().rotate(nav.convertDistance(WHEELRADIUS, 10.0), false);
		
		//open the door
		gateMotor.rotate(32);
		
		
		//set this up tomorrow
		//navigate away to drop off the bricks
		newBot.getLeftMotor().rotate(nav.convertDistance(WHEELRADIUS, 20.0), true);
		newBot.getRightMotor().rotate(nav.convertDistance(WHEELRADIUS, 20.0), false);
		
	}

	// the assumption is that the field will be a square
	/**
	 * ComputePosition
	 * @param tile
	 * @param gridsize
	 * @return
	 */
	public static double[] computePosition(int tile, int gridsize) {
		// pos has the format {x, y, theta}
		double[] pos = new double[3];
		double tileLength = 30.48;

		// Tile numbers reflect the specifications that were provided in the
		// specification document.
		if (tile == 1) {
			// set theta first
			pos[2] = 0;
			// set y position
			pos[1] = 0 * tileLength;
			pos[0] = 0;
		} else if (tile == 2) {
			pos[2] = 270;
			pos[1] = 0;
			// x will be different
			pos[0] = (gridsize - 2) * tileLength;

		} else if (tile == 3) {
			pos[2] = 180;
			pos[1] = (gridsize - 2) * tileLength;
			pos[0] = pos[1];
		} else if (tile == 4) {
			pos[2] = 90;
			pos[1] = (gridsize - 2) * tileLength;
			pos[0] = 0;
		}

		return pos;
	}
}
