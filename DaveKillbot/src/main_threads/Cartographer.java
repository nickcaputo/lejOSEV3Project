package main_threads;

import java.util.BitSet;

import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.Color;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.utility.Delay;

/**
 * Tests mapmaking by roving about a maze.
 * 
 * @author CGreen
 *
 */
public class Cartographer {
	static DifferentialPilot robotPilot;
	static RegulatedMotor frontNeckMotor = Motor.A;
	static EV3UltrasonicSensor distanceSensor;
	static EV3TouchSensor bumpSensor; 
	static EV3ColorSensor colorSensor;
	static SampleProvider bumpSampleProvider;
	static SampleProvider distanceSampleProvider;
	static SampleProvider colorSampleProvider;
	static GPS gps;
	static GraphicsLCD mazeScreen, infoScreen;
	
	static long startTimeMillis;
	
	static final int STARTING_X = 0;
	static final int STARTING_Y = 3;
	static final double MAX_DISTANCE = 1.23;
	
	public static void main(String[] args) {
		// Sets up Pilot for robot and sensors for robot
		robotPilot = new DifferentialPilot(5.4, 14.5, Motor.C, Motor.B, false);
		distanceSensor = new EV3UltrasonicSensor(SensorPort.S1);
		bumpSensor = new EV3TouchSensor(SensorPort.S2);
		colorSensor = new EV3ColorSensor(SensorPort.S4);
		distanceSampleProvider = distanceSensor.getDistanceMode();

		robotPilot.setTravelSpeed(30);
		robotPilot.setAcceleration(60);
		robotPilot.setRotateSpeed(60);
		robotPilot.reset();
		mazeScreen = BrickFinder.getDefault().getGraphicsLCD();
		gps = new GPS(mazeScreen, STARTING_X, STARTING_Y, 0, 4, 4);
		
		LCD.clear();
		LCD.drawString("Press Enter to", 0, 0);
		LCD.drawString("begin Dave's", 0, 1);
		LCD.drawString("mission", 0,2);
		Button.waitForAnyPress();
		LCD.clear();
		Delay.msDelay(2000);
		gps.prepGPS();
		run();
		LCD.clear();
	}
	
	public static void run() {
		bumpSampleProvider = bumpSensor.getTouchMode();
		float[] bumpSample = new float[bumpSampleProvider.sampleSize()];
		float[] distanceScanData = new float[4];

		bumpSampleProvider.fetchSample(bumpSample, 0);

		//looking for white
		while(colorSensor.getColorID() != Color.WHITE && !Button.ENTER.isDown()){
			if (!gps.getVisited(gps.getX(), gps.getY())) {
				scanCell(distanceScanData);
			}
			robotPilot.travel(112, true);
									// the "true" argument means that this call returns immediately (instead of 
									// waiting/blocking), and the code below runs while the robot travels
			gps.updatePosition(1);
			while (robotPilot.isMoving()) {
				bumpSampleProvider.fetchSample(bumpSample, 0);
				if(bumpSample[0] == 1){ // is the touch sensor currently pushed in?
					robotPilot.stop();
					robotPilot.travel(-10);
					robotPilot.rotate(10); // always turn 90 degrees when you bump into something? NO WAY MAN
				}
				if (colorSensor.getColorID() == Color.WHITE){ // found the GOAL
					break;
				}
			}
		}
		robotPilot.stop();
		Button.LEDPattern(4); // victory celebration!
		playVictoryTuneOne();
		Button.waitForAnyPress();
	}

	public static void scanCell(float[] distanceScanData) {
		int[] coords = gps.getCoordinates();
		int count = 0;
		while (count < 4){
			if (Button.ENTER.isDown()) { 
				return; 
			} // break out, if someone is holding down the ENTER button.
			
			distanceScanData[count] = getDistanceMeasurement();
			frontNeckMotor.rotate(90);
			count++;
		}
		BitSet wallInfo = new BitSet(4);
		for (int direction = 0; direction < 4; direction++) {
			if (distanceScanData[direction] < MAX_DISTANCE) {
				wallInfo.set(direction);
			}
		}
		frontNeckMotor.rotate(-360); // rotate neck back (otherwise cords will tighten/tangle).
		gps.setWalls(coords[0], coords[1], wallInfo);
		gps.setVisited(coords[0], coords[1]);
	}
	
	public static float getDistanceMeasurement(){
		float[] distanceSample = new float[distanceSampleProvider.sampleSize()];
		distanceSampleProvider.fetchSample(distanceSample, 0);
		// provide instant feedback on whether a wall was detected
		if (distanceSample[0] < MAX_DISTANCE) {
			Sound.playTone(66, 100);
		} else {
			Sound.playTone(523, 100);
		}
		return distanceSample[0];
	}
	
	public static void playVictoryTuneOne() {
		Sound.playTone(880, 252);
		Sound.playTone(466*2, 252);
		Sound.playTone(494*2, 252);
		Sound.playTone(523*2, 750);
	}
	
}
