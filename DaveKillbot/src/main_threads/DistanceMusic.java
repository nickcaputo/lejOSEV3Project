package main_threads;


import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class DistanceMusic {

	public static void main(String[] args) {
		EV3UltrasonicSensor sensor = new EV3UltrasonicSensor(SensorPort.S1);
		sensor.enable();
		
		SampleProvider sp = sensor.getDistanceMode();
		
		float[] sample = new float[sp.sampleSize()];

		while(!Button.ENTER.isDown()){
			LCD.clear();
			sp.fetchSample(sample, 0);
			float distance = sample[0];
			LCD.drawString(String.format("Dist: %.2f", distance), 0, 4);
			if (distance <= 0.5) {
				Sound.playNote(Sound.XYLOPHONE, 500, 100);
				Button.LEDPattern(1);				
			} else if (distance > 1.5 && distance < 2) {				
				Sound.playNote(Sound.FLUTE, 700, 100);
				Button.LEDPattern(2);
			} else {
				Button.LEDPattern(0);
			}
			Delay.msDelay(100);
		}
		sensor.close();		
	}

}
