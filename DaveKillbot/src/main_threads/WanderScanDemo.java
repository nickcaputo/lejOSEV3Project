package main_threads;



import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.robotics.Color;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.utility.Delay;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;


public class WanderScanDemo {
	static DifferentialPilot robotPilot;
	static RegulatedMotor frontNeckMotor = Motor.A;
	static EV3UltrasonicSensor distanceSensor;
	static EV3TouchSensor bumpSensor; 
	static EV3ColorSensor colorSensor;
	static SampleProvider bumpSampleProvider;
	static SampleProvider distanceSampleProvider;
	static SampleProvider colorSampleProvider;

	static long startTimeMillis;

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

		LCD.clear();
		LCD.drawString("ENTER to run!", 0,0);
		Button.waitForAnyPress();
		LCD.clear();
		
		run();
	}

	public static void run(){
		bumpSampleProvider = bumpSensor.getTouchMode();
		float[] bumpSample = new float[bumpSampleProvider.sampleSize()];
		float[] distanceScanData = new float[12];

		bumpSampleProvider.fetchSample(bumpSample, 0);

		//looking for white
		while(colorSensor.getColorID() != Color.WHITE && !Button.ENTER.isDown()){
			scanSurroundings(distanceScanData);
			int angle = chooseDirectionAngle(distanceScanData);

			robotPilot.rotate(angle);
			robotPilot.travel(40, true); // travel 40 CENTIMETERS forward.
									// the "true" argument means that this call returns immediately (instead of 
									// waiting/blocking), and the code below runs while the robot travels
			while (robotPilot.isMoving()) {
				bumpSampleProvider.fetchSample(bumpSample, 0);
				if(bumpSample[0] == 1){ // is the touch sensor currently pushed in?
					robotPilot.stop();
					robotPilot.travel(-10);
					robotPilot.rotate(90); // always turn 90 degrees when you bump into something?
				}
				if (colorSensor.getColorID() == Color.WHITE){ // found the GOAL
					break;
				}
			}
		}
		robotPilot.stop();
		Button.LEDPattern(4); // victory celebration!
		Sound.beepSequenceUp();
		Sound.beepSequence();
		Button.waitForAnyPress();
	}

	/** Robots looks around at 12 evenly spaced intervals, measuring the 
	 *  distance to an object at each of those angles, and storing those distances
	 * into the distanceScanData array parameter that was passed in.
	 */
	public static void scanSurroundings(float[] distanceScanData ){
		int count = 0;
		while(count < 12){
			if (Button.ENTER.isDown()) { return; } // break out, if someone is holding down the ENTER button.

			distanceScanData[count] = getDistanceMeasurement();
			frontNeckMotor.rotate(30);  // 30 degrees is 1/12 of a full circle...
			count++;
		}
		frontNeckMotor.rotate(-360); // rotate neck back (otherwise cords will tighten/tangle). 
	}

	/**
	 *  Robot chooses an angle to turn, based on the distance readings in each direction.
	 *  In this case, it goes straight if the path in front is clear, but otherwise
	 *   it chooses to go in the direction that has the longest line-of-sight distance. 
	 * @param distanceScanData - array of distance measurements at each angle.
	 * @return an angle (in degrees) for the robot to turn.
	 */
	public static int chooseDirectionAngle(float[] distanceScanData){

		float maxDistance = 0;
		if(distanceScanData[0]>1){ // if the path seems clear up to 1 meter ahead, go straight ahead
			return 0; // angle 0, no turn
		}
		int maxIndex = 0;

		// this loop finds the direction that had the farthest distance reading
		for(int i = 0; i < distanceScanData.length; i++){
			if(Double.isInfinite(distanceScanData[i])){  // anything beyond 2.5m reads as infinity from the sensor
				maxIndex = i;
				break;
			}else if(distanceScanData[i] >= maxDistance){
				maxDistance = distanceScanData[i];
				maxIndex = i;
			}
		}	
		int angle = (maxIndex* -30);   // goes towards that index
		if (angle < -180) { angle += 360; }

		return angle;
	}

	/** Convenience method to get ONE distance measurement from the robot.
	 * (Technically it's a bit inefficient to keep re-allocating this small float[] array every time,
	 *   when we could be directly fetching the value into the scan array, but using this method
	 *   probably promotes more readable code...)
	 * 
	 * @return the distance reading (in meters) in the direction the EV3's ultra-sonic sensor is currently facing.
	 */
	public static float getDistanceMeasurement(){
		float[] distanceSample = new float[distanceSampleProvider.sampleSize()];
		distanceSampleProvider.fetchSample(distanceSample, 0);

		return distanceSample[0];
	}



}
