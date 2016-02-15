package main_threads;

import java.util.ArrayList;
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
	static ArrayList<Integer[]> cellHistory;
	
	
	static long startTimeMillis;
	
	static final int STARTING_X = 0;
	static final int STARTING_Y = 0;
	static final double MAX_DISTANCE = 1.23;
	static final double TRAVEL_DISTANCE = 117;
	
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
		infoScreen = BrickFinder.getDefault().getGraphicsLCD(); // if this is possible.
		// The idea would be to switch between map and info screens. Display map at the end
		// of a run. If this is impossible, I'll scrap the info screen and aim to draw stats
		// such as current position and goal position next to the map.
		gps = new GPS(mazeScreen, STARTING_X, STARTING_Y, 0, 4, 4);
		cellHistory = new ArrayList<Integer[]>();
		
		LCD.clear();
		LCD.drawString("Press Enter to", 0, 0);
		LCD.drawString("begin Dave's", 0, 1);
		LCD.drawString("mission", 0,2);
		Button.waitForAnyPress();
		LCD.clear();
		Delay.msDelay(2000);
		int[] goalLocation = findGoal();
		// TODO: Indicate on the map the location of the goal?
		Button.waitForAnyPress();
		Delay.msDelay(2000);
		goHome();
		LCD.clear();
	}
	/**
	 * Rove the map in search of the goal, documenting the maze layout along the way.
	 * 
	 * @return the coordinates of the cell containing the goal.
	 */
	public static int[] findGoal() {
		gps.prepGPS();
		int orientation = gps.getOrientation();
		int newOrientation;
		int[] currentPosition = gps.getCoordinates();
		bumpSampleProvider = bumpSensor.getTouchMode();
		float[] bumpSample = new float[bumpSampleProvider.sampleSize()];
		float[] distanceScanData = new float[4];

		bumpSampleProvider.fetchSample(bumpSample, 0);

		//looking for white
		while(colorSensor.getColorID() != Color.WHITE && !Button.ENTER.isDown()) {
			// Keep track that the robot came from the reverse direction
			gps.setDirectionTaken(currentPosition[0], currentPosition[1], 2);
			// Gather data if cell is unfamiliar
			if (!gps.getVisited(currentPosition[0], currentPosition[1])) {
				scanCell(distanceScanData);
			}
			// Determine new direction of travel
			newOrientation = getNewHeading(currentPosition, orientation);
			
			// Rotate if necessary
			if (newOrientation != orientation) {
				int rotationAngle = (newOrientation-orientation) % 4 * 90;
				if (Math.abs(rotationAngle) == 270) {
					if (rotationAngle < 0) {
						rotationAngle += 360;
					} else {
						rotationAngle -= 360;
					}
				}
				robotPilot.rotate(rotationAngle);
				orientation = newOrientation;
				gps.updateOrientation(orientation);
			}
			
			// If this is the first visit to this cell, make note that it has been here.
			if (!gps.getVisited(currentPosition[0], currentPosition[1])) {
				gps.setVisited(currentPosition[0], currentPosition[1]);
			}
			
			// Add this cell to the history.
			cellHistory.add(new Integer[]{currentPosition[0], currentPosition[1]});
			
			// Make yo' move.
			robotPilot.travel(TRAVEL_DISTANCE, true);
									// the "true" argument means that this call returns immediately (instead of 
									// waiting/blocking), and the code below runs while the robot travels
			// Make note of what direction was chosen.
			gps.setDirectionTaken(currentPosition[0], currentPosition[1], orientation);
			
			// Take note that the robot moved.
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
		
		// TODO: Figure out how much make-up distance is required (set to 18 for now)
		robotPilot.travel(18);
		// Scan the cell containing the goal
		scanCell(distanceScanData);
		gps.updatePosition(1);
		int[] goalCoords = gps.getCoordinates();
		gps.setVisited(goalCoords[0], goalCoords[1]);
		robotPilot.rotate(180);
		gps.updateOrientation(orientation+2);
		return goalCoords;
	}
	
	/**
	 * Scan the current cell to detect walls and update the GPS. 
	 * 
	 * @param distanceScanData
	 */
	public static void scanCell(float[] distanceScanData) {
		// TODO: Remove redundant backwards scan.
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
	}
	
	/**
	 * Detects the distance of obstacles (if any) in a cardinal direction.
	 * 
	 * @return the distance between a perceived object, if any
	 */
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
	
	/**
	 * Determine the next direction to travel based on data from scans.
	 * 
	 * @return the new direction of travel
	 */
	public static int getNewHeading(int[] coordinates, int currentOrientation) {
		//TODO: Polish this method, debug as necessary, etc.
		
		/* 1. Get wall data.
		 * 2. Prioritize robot's left turn: check whether open. If open, check whether
		 *    the adjoining cell has been visited. If not, proceed. If it has,
		 *    make same checks in the next direction (up, then right).
		 *    If all cells (besides reverse) have already been visited, or if all other paths
		 *    are closed, revert to a previous cell.
		 * 3. UPDATE GPS WITH NEW POSITION AND ORIENTATION.
		 * 4. Drive on, you sexy robot, you.
		 * 
		 * 
		 * POTENTIAL PROBLEM: If robot is faced with two consecutive corridors, it
		 * might go back and forth between them in an infinite loop. Could be remedied
		 * by choosing a non-backwards direction if the ONLY other option is a U-turn.
		 * 
		 */
		
		int newOrientation = currentOrientation;
		BitSet wallInfo = gps.getWalls(coordinates[0], coordinates[1]);
		BitSet pathsTaken = gps.getDirectionsTaken(coordinates[0], coordinates[1]);
		
		// Check directions starting with the direction TO THE LEFT of where the robot's 
		// body is facing. For example, if the robot is facing east, the robot will 
		// check the data in index 0 first. (0=N, 1=E, 2=S, 3=W)
		
		int direction = (currentOrientation - 1) % 4;
		for (int counter = 0; counter < 3; counter++) {
			if (!wallInfo.get(direction)) {
				
				// If the robot has been here before, we will try and travel
				// in a direction not previously taken, still (hopefully) following DFS.
				if (gps.getVisited(coordinates[0], coordinates[1])) {
					if (!pathsTaken.get(direction)) {
						newOrientation = direction;
						return newOrientation;
					}
				// Otherwise, we check for a new cell to visit.
				} else {
					if (!gps.getVisitedNeighbor(direction)) {
						newOrientation = direction;
						return newOrientation;
					}
				}
			}
			direction = (direction++) % 4;
		}
		// If no good options, goes back the way it came.
		return (currentOrientation + 2) % 4;
	}
	
	// TODO: Method to return to starting cell (STARTING_X, STARTING_Y)
	/**
	 * Go back to the starting cell.
	 */
	public static void goHome() {
		
	}
	public static void playVictoryTuneOne() { // LoZ treasure chest item theme
		Sound.playTone(880, 252);
		Sound.playTone(932, 252);
		Sound.playTone(988, 252);
		Sound.playTone(1046, 750);
	}
	
	public static void playVictoryTuneTwo() { // change this one to something else (maybe FF?)
		Sound.playTone(880, 252);
		Sound.playTone(932, 252);
		Sound.playTone(988, 252);
		Sound.playTone(1046, 750);
	}
	
}
