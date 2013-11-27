import lejos.nxt.ColorSensor;
import lejos.nxt.Sound;
import lejos.nxt.ColorSensor.Color;
import lejos.nxt.SensorPort;


public class Detection extends Thread {
	private final int COLOR_THRESHOLD = 278; //verify this threshold 
	private int numberOfBlocks = 0; 
	final ColorSensor blockDetector;
	
	public Detection(ColorSensor colorSensor) {
		this.blockDetector = colorSensor;
		this.blockDetector.setFloodlight(Color.BLUE);
	}
	
	public void run() {
		int a = 0;
		int b = 0;
		while (true) {
			a = blockDetector.getNormalizedLightValue();
			if (a > COLOR_THRESHOLD) {
				//Sound.buzz();
				if (b - a > 25 && b - a < 40) { // if there is a block in front
												// of the color sensor
					b = a;
					numberOfBlocks++;
					Sound.setVolume(Sound.VOL_MAX);
					Sound.twoBeeps();
					/*
					 * try { waits 4 seconds so block can pass (might need to
					 * edit sleep time) this ensures that block is not accounted
					 * for twice
					 * 
					 * Thread.sleep(4000); //test the amount of time it sleeps }
					 * catch (InterruptedException e) { // TODO Auto-generated
					 * catch block //does nothing }
					 */
				}
			}
			b = a;
		}
	}
	
	public boolean hasThreeBlocks() {
		if (numberOfBlocks >= 2) {
			return true; 
		}
		return false; 
	}

	public void resetNumberOfBlocks() {
		numberOfBlocks = 0; 
	}
}
