import lejos.nxt.ColorSensor;
import lejos.nxt.LCD;
import lejos.nxt.Sound;
import lejos.nxt.ColorSensor.Color;
import lejos.nxt.SensorPort;


public class Detection extends Thread {
	private final int COLOR_THRESHOLD = 278; //verify this threshold 
	private int numberOfBlocks = 0; 
	final ColorSensor blockDetector;
	
	/** Detection constructor
	 * 
	 */
	public Detection(ColorSensor colorSensor) {
		this.blockDetector = colorSensor;
		this.blockDetector.setFloodlight(Color.BLUE);
	}
	/** Starts a new thread that increases the count of blocks everytime the threshold of the colorsensor gets
	 * passed
	 * 
	 */
	public void run() {
		int a = 0;
		while (!hasThreeBlocks()) {
			a = blockDetector.getNormalizedLightValue();
			LCD.clear(3);
			LCD.drawInt(a, 0, 3);
			if(a > 278){
				//beeps twice and increments the numberOfBlocks
				Sound.twoBeeps();
				numberOfBlocks++;
				try {
					//Sleeps for 3 seconds after each increment to avoid double incrementation
					Thread.sleep(3000);
					
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}			
	}
	
	public boolean hasThreeBlocks() {
		//returns true if there is more than 1 block in the stack
		if (numberOfBlocks >= 1) {
			return true; 
		}
		return false; 
	}

	public void resetNumberOfBlocks() {
		//resets the number of blocks to 1
		numberOfBlocks = 0; 
	}
}
