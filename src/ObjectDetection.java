/*ObjectDetection.java
 * Written by: Group 11
 * 
 * This class is an object detection class that differentiates between wood blocks and styrofoam blocks.
 * It also detects obstacles from the filtered data that is received.
 * 
 * The identify method only works for objects that are in a specific range. 
 * In our case its calibrated for when the object is a at distance of 6 detected by the US.
*/

import lejos.nxt.ColorSensor;
import lejos.nxt.LCD;
//import lejos.nxt.SensorPort;
//import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;

public class ObjectDetection {
	private ColorSensor cs;
	//private UltrasonicSensor us;
	private double redValue, blueValue;
	//private double wallThreshold = 60;
	//private int objectCount;
	//private boolean sameObject;
	//private double x1, x2;
	// constructor

	/**
	 * Constructor
	 * @param cs
	 * @param us
	 */
	public ObjectDetection(ColorSensor cs, UltrasonicSensor us) {
		this.cs = cs;
		//this.us = us;
		//this.redValue = redValue;
		//this.blueValue = blueValue;
		//this.objectCount = 0;
		//this.x1 = 0;
		//this.x2 = 0;
		//this.sameObject = false;
	}
	
	// Method must be modified according to the testing data

	/**
	 * Identifies block or blue palette
	 * @return
	 */
	public int identify() {
		LCD.clear();
		int returnValue = 2;

		// set the floodlight to red
		cs.setFloodlight(0);
		// read the normalized light value with the red floodlight
		redValue = cs.getNormalizedLightValue();

		// LCD.drawString(String.valueOf(colorValue), 3, 2);

		// set the floodlight to red
		cs.setFloodlight(2);
		// read the normalized light value with the red floodlight
		blueValue = cs.getNormalizedLightValue();

		// Values found through experimental data comparing the ratio of red
		// light to blue light, which was more accurate
		// than simply computing the difference between the two light values.
		//We found that using ratios was more effective for determining whether a block is wood or styrofoam.
		
		// NOTE: we will have to return something when we call this function
		// later in the code.
		
		//if statements below compute th ratios and determine what kind of block it is and return a value depending on what it is.
		if ((redValue / blueValue) > 1.09) {
			LCD.drawString("OBJECT DETECTED!", 0, 5);
			LCD.drawString("NOT BLOCK!!!", 0, 6);
			returnValue = 0;
		}
		if ((redValue / blueValue) <= 1.09 && (redValue / blueValue) > 1.01) {
			LCD.drawString("OBJECT DETECTED!", 0, 5);
			LCD.drawString("BLOCK!!!", 0, 6);
			returnValue = 1;
			
		}
		if ((redValue / blueValue) <= 1.01) {
			LCD.drawString("NO OBJECT FOUND!", 0, 5);
			returnValue = 2;
		}
		return returnValue;
		
	}
	

}
