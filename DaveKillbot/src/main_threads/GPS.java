package main_threads;

import java.util.BitSet;

import lejos.hardware.lcd.GraphicsLCD;

public class GPS {
	// orientation is: 0=N, 1=E, 2=S, 3=W
	private int x, y, orientation, numRows, numColumns;
	private BitSet[][] wallData; // [N, E, S, W]
	private MazeRevealMap mazeMap;
	/**
	 * Class constructor
	 * @param startX - starting x position
	 * @param startY - starting y position 
	 * @param startOrientation - starting orientation [0=N, 1=E, 2=S, 3=W]
	 * @param numRows - number of rows of cells in the maze
	 * @param numColumns - number of columns of cells in the maze
	 * @param screen - the leJOS robot's lcd screen
	 */
	public GPS(int startX, int startY, int startOrientation, int numRows, int numColumns, GraphicsLCD screen) {
		x = startX;
		y = startY;
		orientation = startOrientation%4;
		this.numRows = numRows;
		this.numColumns = numColumns;
		wallData = new BitSet[numRows][numColumns];
		mazeMap = new MazeRevealMap(screen, numRows, numColumns);
		for (int i = 0; i < wallData.length; i++) {
			for (int j = 0; j < wallData[i].length; j++) {
				wallData[i][j] = new BitSet(4);
			}
		}
		
	}
	// debug method to learn how bitsets work
	/*public void showMeTheWalls() {
		for (int i = 0; i < wallData.length; i++) {
			for (int j = 0; j < wallData[i].length; j++) {
				System.out.print(wallData[i][j].get(0));
				System.out.print(wallData[i][j].get(1));
				System.out.print(wallData[i][j].get(2));
				System.out.print(wallData[i][j].get(3));
				System.out.print(" ");
			}
			System.out.println();
		}
	}*/
	/**
	 * Updates the GPS with the new orientation.
	 * 
	 * The orientation should be updated each time after
	 * the robot has performed a 90 degree rotation.
	 * 
	 * @param newOrientation - the new orientation
	 * @return the new orientation
	 */
	public int updateOrientation(int newOrientation) {
		orientation = newOrientation%4;
		return orientation;
	}
	
	/**
	 * Updates the GPS with new coordinates based on how many cells
	 * were traversed by the robot in the forwards direction.
	 * 
	 * The current position should be updated after each time the robot
	 * moves successfully from one cell to another.
	 * 
	 * @param step - the number of cells traversed. Negative values
	 * denote movement in the reverse direction (specifically, the 
	 * robot moving in the opposite direction of that which it is facing in).
	 * 
	 * @return the new coordinates
	 */
	public int[] updatePosition(int step) {
		if (orientation%2 == 1) {
			x+=step;
		} else {
			y+=step;
		}
		return getCoordinates();
	}
	/**
	 * Resets the GPS, blanking out walls on the map and placing the robot in a
	 * new starting position.
	 * 
	 * @param startX
	 * @param startY
	 * @param orientation
	 */
	public void resetGPS(int startX, int startY, int orientation) {
		mazeMap.reset();
		x = startX;
		y = startY;
		this.orientation = orientation;
	}
	public int[] getCoordinates() {
		int[] coords = new int[2];
		coords[0] = x;
		coords[1] = y;
		return coords;
	}
	public int getOrientation() {
		return orientation;
	}
	public int getX() {
		return x;
	}
	public int getY() {
		return y;
	}
	
}
