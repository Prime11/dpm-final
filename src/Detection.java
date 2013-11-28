import lejos.nxt.ColorSensor;
import lejos.nxt.LCD;
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
		while (!hasThreeBlocks()) {
			a = blockDetector.getNormalizedLightValue();
			LCD.clear(3);
			LCD.drawInt(a, 0, 3);
			if(a > 278){
				Sound.twoBeeps();
				numberOfBlocks++;
				try {
					Thread.sleep(3000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
			
			/*
			 * if (a > COLOR_THRESHOLD) { //Sound.buzz(); if (b - a > 25 && b -
			 * a < 40) { // if there is a block in front // of the color sensor
			 * b = a; numberOfBlocks++; Sound.setVolume(Sound.VOL_MAX);
			 * Sound.twoBeeps(); } } b = a;
			 */
		}
	}
	
	public boolean hasThreeBlocks() {
		if (numberOfBlocks >= 1) {
			return true; 
		}
		return false; 
	}

	public void resetNumberOfBlocks() {
		numberOfBlocks = 0; 
	}
}
