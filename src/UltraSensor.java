/*
 * OdometryDisplay.java
 */

import lejos.nxt.UltrasonicSensor;

public class UltraSensor extends Thread {
	private static final long DISPLAY_PERIOD = 20;
	private UltrasonicSensor us ;//= new UltrasonicSensor(SensorPort.S2);
	private int distance;

	// constructor
	 /**
	  * Constructor
	  * @param us
	  */
	public UltraSensor (UltrasonicSensor us) {
		this.us = us;
		//this.distance = distance;
	}

	// run method (required for Thread)

	 /**
	  * Runs Ultrasonic + Display
	  */
	public void run() {
		long displayStart, displayEnd;
	
		// clear the display once
		//LCD.clearDisplay();

		while (true) {
			displayStart = System.currentTimeMillis();
			
			//get the distance
			this.distance = us.getDistance();
			
			// clear the lines for displaying odometry information
			//LCD.clear(6);
			//LCD.drawString("US Sensor:", 0, 6);
			//LCD.drawString(String.valueOf(distance), 10, 6);

			
			// throttle the OdometryDisplay
			displayEnd = System.currentTimeMillis();
			if (displayEnd - displayStart < DISPLAY_PERIOD) {
				try {
					Thread.sleep(DISPLAY_PERIOD - (displayEnd - displayStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that OdometryDisplay will be interrupted
					// by another thread
				}
			}
		}
	}

	 /**
	  * Returns Distance
	  * @return
	  */
	public int getDist() {
		return this.distance;
	}
	

}
