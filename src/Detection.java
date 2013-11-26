import lejos.nxt.ColorSensor;
import lejos.nxt.SensorPort;


public class Detection extends Thread {
	private final int COLOR_THRESHOLD = 278; //verify this threshold 
	private int numberOfBlocks = 0; 
	final ColorSensor blockDetector;
	
	public Detection(ColorSensor colorSensor) {
		this.blockDetector = colorSensor; 
	}
	
	public void run() {
		while(true) {
			if (blockDetector.getNormalizedLightValue() > COLOR_THRESHOLD) { //if there is a block in front of the color sensor
				/*waits 4 seconds so block can pass (might need to edit sleep time) 
				 * this ensures that block is not accounted for twice
				 */
				numberOfBlocks++;
				try { 
					Thread.sleep(4000); //test the amount of time it sleeps 
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					//does nothing 
				}
			}
		}
	}
	
	public boolean hasThreeBlocks() {
		if (numberOfBlocks == 3) {
			return true; 
		}
		return false; 
	}

	public void resetNumberOfBlocks() {
		numberOfBlocks = 0; 
	}
}
