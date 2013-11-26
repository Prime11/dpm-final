import lejos.nxt.ColorSensor;
import lejos.nxt.SensorPort;


public class Detection {
	private final int COLOR_THRESHOLD = 278; //verify this threshold 
	private int numberOfBlocks = 0; 
	private boolean hasThreeBlocks = false; 
	final ColorSensor blockDetector;
	
	public Detection(ColorSensor colorSensor) {
		this.blockDetector = colorSensor; 
	}
	
	//when this method returns true, the robot should drive to the respective dropoff zone 
	public boolean obtainedBlock() {
		while (numberOfBlocks < 3) {
			if (blockDetector.getNormalizedLightValue() > COLOR_THRESHOLD) { //if there is a block in front of the color sensor
				/*waits 4 seconds so block can pass (might need to edit sleep time) 
				 * this ensures that block is not accounted for twice
				 */
				try { 
					Thread.sleep(4000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					//does nothing 
				}
				numberOfBlocks++; 
				return hasThreeBlocks; 
			}
		}
		hasThreeBlocks = true; 
		return hasThreeBlocks; 
	}
}
