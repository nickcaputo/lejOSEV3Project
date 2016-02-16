package main_threads;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * Tests a way of taking a list of cells visited and removing dead end routes.
 * 
 * @author CGreen
 *
 */
public class RemoveDupesTest {
	public static void main(String[] args) {
		ArrayList<Integer[]> cellHistory = new ArrayList<Integer[]>();
		cellHistory.add(new Integer[]{0,0});
		cellHistory.add(new Integer[]{0,1});
		cellHistory.add(new Integer[]{1,1});
		cellHistory.add(new Integer[]{1,2});
		cellHistory.add(new Integer[]{0,2});
		cellHistory.add(new Integer[]{1,2});
		cellHistory.add(new Integer[]{1,3});
		cellHistory.add(new Integer[]{0,3});
		cellHistory.add(new Integer[]{-1,3}); // I know it's negative -- it's only a test.
		cellHistory.add(new Integer[]{0,3});
		cellHistory.add(new Integer[]{1,3});
		cellHistory.add(new Integer[]{2,3});
		cellHistory.add(new Integer[]{3,3});
		
		ArrayList<Integer[]> stepsHome = new ArrayList<Integer[]>();
		/* PSEUDOCODE
		 * 
		 * for each cell in the history:
		 * 		if it is already in the list of cells to visit:
		 * 			- Create a variable equal to that cell's coords
		 * 			- Add it to the list of cells to visit
		 * 			- Find the first instance of the duplicate cell in the steps
		 * 			- Remove the first instance of the current cell from the steps
		 * 			- While the cell at that same index (they've been shifted left) does not
		 * 				equal the duplicate, remove the cell at that index.
		 * 		if it is not:
		 * 			- Add it to the list of cells to visit
		 */
		for (int index = 0; index < cellHistory.size(); index++) {
			if (isInArrayList(cellHistory.get(index), stepsHome)) {
				Integer[] duplicate = cellHistory.get(index);
				stepsHome.add(0, duplicate);
				int indexOfDupe = findFirstInstance(duplicate, stepsHome);
				if (indexOfDupe == -1) {
					throw new ArrayIndexOutOfBoundsException("You done broke it, fool.");
				}
				stepsHome.remove(indexOfDupe);
				while (!Arrays.equals(duplicate, stepsHome.get(indexOfDupe))) {
					stepsHome.remove(indexOfDupe);
				}
			} else {
				stepsHome.add(0, cellHistory.get(index));
			}
		}
		for (Integer[] cell : stepsHome) {
			System.out.println("(" + cell[0] + ", " + cell[1] + ")");
		}
		System.out.println();
		Integer[] cell1 = {1,2};
		Integer[] cell2 = {1,2};
		System.out.println(cell1.equals(cell2));
		System.out.println(Arrays.equals(cell1, cell2));
	}
	/**
	 * Because .equals() and .contains() give inaccurate readings for the Integer[] objects.
	 * 
	 * @param cellCoords
	 * @param stepsHome
	 * @return
	 */
	public static boolean isInArrayList(Integer[] cellCoords, ArrayList<Integer[]> stepsHome) {
		for (int index = 0; index < stepsHome.size(); index++) {
			if (Arrays.equals(cellCoords, stepsHome.get(index))) {
				return true;
			}
		}
		return false;
	}
	
	/**
	 * Same reason as above.
	 * 
	 * @param cellCoords
	 * @param stepsHome
	 * @return
	 */
	public static int findFirstInstance(Integer[] cellCoords, ArrayList<Integer[]> stepsHome) {
		for (int index = 0; index < stepsHome.size(); index++) {
			if (Arrays.equals(cellCoords, stepsHome.get(index))) {
				return index;
			}
		}
		return -1;
	}
}
