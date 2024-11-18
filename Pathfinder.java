import java.util.*;
import java.io.FileWriter;
import java.io.IOException;

public class Pathfinder {
    public static void main(String[] args) {
        // Number of runs
        int instances = 10; // You can adjust this as needed


        double[] percentages = {0.0, 0.2, 0.4, 0.6, 0.8, 0.9};
        // Grid sizes
        int[] gridSizes = {200, 400, 600, 800, 1000};
        

        try {
            FileWriter csvWriter = new FileWriter("pathfinding_result.csv");
            csvWriter.append("Instance#, Grid Size, Pit Percentage, Algorithm, Time (ms), Memory (bytes), Path?, Explored-set\n");

        for (int gridSize : gridSizes) {
            for (double percentage : percentages) {
                for (int instance = 0; instance < instances; instance++) {

                    // Declaring variables and setting up the environment of the grid
                    int xo = 0;
                    int yo = 0;
                    int goalRow = gridSize - 1;
                    int goalCol = gridSize - 1;

                    // Generate grid for each for loop
                    char[][] gridSetup = new char[gridSize][gridSize];
                    char[][] grid = generateGrid(gridSetup, gridSize, gridSize, xo, yo, goalRow, goalCol, percentage);
                    //printGrid(grid);

                    // Run Garbage Collection before each run
                    System.gc();

                    // Measure time and memory usage for DFS
                    long startTimeDFS = System.currentTimeMillis();
                    boolean[][] visitedDFS = new boolean[gridSize][gridSize];
                    SearchResult pathExistsDFS = depthFirstSearch(grid, visitedDFS, xo, yo, goalRow, goalCol);
                    long endTimeDFS = System.currentTimeMillis();
                    long executionTimesDFS = endTimeDFS - startTimeDFS;
                    long memoryUsedDFS = Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory();
                    writeResults(csvWriter, instance, gridSize, percentage, "DFS", pathExistsDFS, executionTimesDFS, memoryUsedDFS);

                    // Run Garbage Collection before each run
                    System.gc();
                    // Measure time and memory usage for BFS
                    long startTimeBFS = System.currentTimeMillis();
                    boolean[][] visitedBFS = new boolean[gridSize][gridSize];
                    SearchResult pathExistsBFS = breadthFirstSearch(grid, visitedBFS, xo, yo, goalRow, goalCol);
                    long endTimeBFS = System.currentTimeMillis();
                    long executionTimesBFS = endTimeBFS - startTimeBFS;
                    long memoryUsedBFS = Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory();
                    writeResults(csvWriter, instance, gridSize, percentage, "BFS",  pathExistsBFS, executionTimesBFS, memoryUsedBFS);

                    // Run Garbage Collection before each run
                    System.gc();
                    // Measure time and memory usage for Astar
                    long startTimeAstar = System.currentTimeMillis();
                    boolean[][] visitedAstar = new boolean[gridSize][gridSize];
                    SearchResult pathExistsAstar = AstarSearch(grid, visitedAstar, xo, yo, goalRow, goalCol);
                    long endTimeAstar = System.currentTimeMillis();
                    long executionTimesAstar = endTimeAstar - startTimeAstar;
                    long memoryUsedAstar = Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory();
                    writeResults(csvWriter, instance, gridSize, percentage, "Astar", pathExistsAstar, executionTimesAstar, memoryUsedAstar);
                }
            }
        }
            csvWriter.flush();
            csvWriter.close();
        }

        catch (IOException e) {
            e.printStackTrace();
        }
    }
    // Helper method to write results to CSV
    private static void writeResults(FileWriter csvWriter, int instance, int gridSize, double percentage, String algorithm, SearchResult pathFound, long time, long memory) throws IOException {
        csvWriter.append(String.valueOf(instance + 1)).append(",    ");
        csvWriter.append(String.valueOf(gridSize)).append(",    ");
        csvWriter.append(String.valueOf(percentage)).append(",    ");
        csvWriter.append(algorithm).append(",    ");
        csvWriter.append(String.valueOf(time)).append(",    ");
        csvWriter.append(String.valueOf(memory)).append(",");
        csvWriter.append(pathFound.isPathFound() ? "Path Found" : "Path Not Found").append(",    ");
        csvWriter.append(String.valueOf(pathFound.getExploredSetSize())).append("\n");
        csvWriter.append("\n"); // Empty line for spacing
    }


    // DFS Implementation
    public static SearchResult depthFirstSearch(char[][] grid, boolean[][] visitedDFS, int xFront, int yFront, int goalRow, int goalCol) {
        Stack<Node> frontier = new Stack<>(); //As DFS uses LIFO queue, Stack add last item at its top (front of queue)
        //Stack<String> frontierSet = new Stack<>();//For us to see in console how frontier is being set
        Stack<String> exploredSetDFS = new Stack<>(); //For us to see in console the explored(visited) nodes

        frontier.push(new Node(xFront, yFront));//Pushing onto top of stack (frontier) from first to last node generated from actions. Leaving the last items on top

        //frontierSet.push("(" + xFront + "," + yFront + ")");//updating frontier with nodes in every state

        while (!frontier.isEmpty()) {//Frontier must have initially at least one item, which is the one from initial state at (0,0)
            Node ndTExp = frontier.pop();//removes(to use node to be expanded, and pop it out from queue)
            int ndTExpX = ndTExp.getX();//the first elements from queue (last in) with pop method
            int ndTExpY = ndTExp.getY();

            if (ndTExpX == goalRow && ndTExpY == goalCol) {

                //System.out.println("Frontier Set: " + /*frontierSet.toString()*/frontier.toString());//printing last updated Frontier set
                //System.out.println("Explored Set: " + exploredSet.toString());//printing last explored nodes
                //System.out.println();
                //System.out.println("Path Found!");
                //System.out.println("Explored Set DFS: " + exploredSetDFS.size());
                return new SearchResult(true, exploredSetDFS.size());

            }

            if (!visitedDFS[ndTExpX][ndTExpY] && grid[ndTExpX][ndTExpY] != 'o') {//first filter with 2 main features of dfs algorithm (redundant paths and obstacles)
                visitedDFS[ndTExpX][ndTExpY] = true; //boolean value to array marking no visited cells and without

                exploredSetDFS.push("(" + ndTExpX + "," + ndTExpY + ")");// pits as visited

                for (int[] dir : DIRECTIONS) {//expanding the ndTExp applying all possible actions from the set of actions
                    int newNdTExpX = ndTExpX + dir[0];
                    int newNdTExpY = ndTExpY + dir[1];
                    if (isValidMove(grid, newNdTExpX, newNdTExpY) && !visitedDFS[newNdTExpX][newNdTExpY]) {//to ensure not to have redundant
                        frontier.push(new Node(newNdTExpX, newNdTExpY));//paths or being expanding out of grid. Only adding valid nodes to Frontier
                        //frontierSet.push("(" + newNdTExpX + "," + newNdTExpY + ")");}
                    }

                    //System.out.println("Frontier Set: " + /*frontierSet.toString()*/frontier.toString());
                    //System.out.println("Explored Set: " + exploredSet.toString());
                    //System.out.println();
                }
            }
        }
        //System.out.println();
        //System.out.println("Path Not Found!");
        //System.out.println("Explored Set DFS: " + exploredSetDFS.size());
        return new SearchResult(false, exploredSetDFS.size());
    }

    // BFS Implementation
    public static SearchResult breadthFirstSearch(char[][] grid, boolean[][] visitedBFS, int xFront, int yFront, int goalRow, int goalCol) {
        ArrayDeque<Node> queue = new ArrayDeque<>();//To have a FIFO queue, Array Dequeue allows to add last items at the end of queue
        ArrayDeque<String> exploredSetBFS = new ArrayDeque<>();
        queue.offer(new Node(xFront, yFront));//with offer method, leaving the first items to go in queue at the front

        while (!queue.isEmpty()) {//Frontier must have initially at least one item, which is the one from initial state at (0,0)
            Node ndTExp = queue.poll();//removes(to use it and pop it out from queue) the first element from queue with poll method()
            int ndTExpX = ndTExp.getX();
            int ndTExpY = ndTExp.getY();

            if (ndTExpX == goalRow && ndTExpY == goalCol) {

                //System.out.println("Explored Set BFS: " + exploredSetBFS.size());

                return new SearchResult(true, exploredSetBFS.size());
            }
            if (!visitedBFS[ndTExpX][ndTExpY] && grid[ndTExpX][ndTExpY] != 'o') {//first filter with 2 main features of dfs algorithm (redundant paths and obstacles)
                visitedBFS[ndTExpX][ndTExpY] = true; //boolean value to array marking no visited cells and without
                //exploredSet.push("(" + ndTExpX + "," + ndTExpY + ")");// pits as visited

                exploredSetBFS.offer("(" + ndTExpX + "," + ndTExpY + ")");// pits as visited

                for (int[] dir : DIRECTIONS) {//expanding the ndTExp applying all possible actions from the set of actions
                    int newNdTExpX = ndTExpX + dir[0];
                    int newNdTExpY = ndTExpY + dir[1];
                    if (isValidMove(grid, newNdTExpX, newNdTExpY) && !visitedBFS[newNdTExpX][newNdTExpY]) {//to ensure not to have redundant
                        queue.offer(new Node(newNdTExpX, newNdTExpY));//paths or being expanding out of grid. Only adding valid nodes to Frontier
                        //frontierSet.push("(" + newNdTExpX + "," + newNdTExpY + ")");}
                    }
                    //System.out.println("Frontier Set: " + frontierSet.toString()frontier.toString());
                    //System.out.println("Explored Set: " + exploredSet.toString());
                    //System.out.println();
                }
            }
        }
        //System.out.println("Path Not Found!");
        return new SearchResult(false, exploredSetBFS.size());
    }

    // A* Implementation
    public static SearchResult AstarSearch(char[][] grid, boolean[][] visitedAstar, int xFront, int yFront, int goalRow, int goalCol) {
        PriorityQueue<Node> queue = new PriorityQueue<>(); // Using the natural ordering (compareTo) of Node objects
        //PriorityQueue<String> frontierSet = new PriorityQueue<>();//For us to see in console how frontier is being set
        Stack<String> exploredSetAstar = new Stack<>(); //For us to see in console the explored(visited) nodes

        // Add the initial node to the priority queue
        queue.offer(new Node(xFront, yFront, goalRow, goalCol, 0));
        //int g = 0;
        //frontierSet.offer("(" + xFront + "," + yFront + ")");//updating frontier with nodes in every state

        while (!queue.isEmpty()) {
            // Poll the node with the lowest F value
            Node ndTExp = queue.poll();
            int ndTExpX = ndTExp.getX();
            int ndTExpY = ndTExp.getY();

            // Check if the goal has been reached
            if (ndTExpX == goalRow && ndTExpY == goalCol) {
                //System.out.println("Frontier Set: " + frontierSet.toString());//printing last updated Frontier
                //System.out.println("Explored Set: " + exploredSetAstar.toString());//printing last explored nodes
                //System.out.println("Explored Set Astar: " + exploredSetAstar.size());
                //System.out.println();

                return new SearchResult(true, exploredSetAstar.size());
            }

            // Mark the current node as visited
            if (!visitedAstar[ndTExpX][ndTExpY] && grid[ndTExpX][ndTExpY] != 'o') {
                visitedAstar[ndTExpX][ndTExpY] = true;

                exploredSetAstar.push("(" + ndTExpX + "," + ndTExpY + ")");// pits as visited

                // Explore neighbors
                for (int[] dir : DIRECTIONS) {
                    int newNdTExpX = ndTExpX + dir[0];
                    int newNdTExpY = ndTExpY + dir[1];

                    // Check if the neighbor is within the grid boundaries and not visited
                    if (isValidMove(grid, newNdTExpX, newNdTExpY) && !visitedAstar[newNdTExpX][newNdTExpY]) {
                        // Calculate the new G value for the neighbor
                        int newG = ndTExp.getG() + 1;

                        // Create a new node for the neighbor and add it to the priority queue
                        queue.offer(new Node(newNdTExpX, newNdTExpY, goalRow, goalCol, newG));

                        //frontierSet.offer("(" + newNdTExpX + "," + newNdTExpY + "," + newG + ")");}
                    }
                    //System.out.println("Frontier Set: " + frontierSet.toString());
                    //System.out.println("Explored Set: " + exploredSetAstar.toString());
                }
            }
        }


        // Path not found
        //System.out.println("Path Not Found!");
        return new SearchResult(false, exploredSetAstar.size());
    }

    public static class SearchResult {
        private final boolean pathFound;
        private final int exploredSetSize;

        public SearchResult(boolean pathFound, int exploredSetSize) {
            this.pathFound = pathFound;
            this.exploredSetSize = exploredSetSize;
        }

        public boolean isPathFound() {
            return pathFound;
        }

        public int getExploredSetSize() {
            return exploredSetSize;
        }
    }

    // Helper methods for grid operations
    public static boolean isValidMove(char[][] grid, int rows, int cols) {
        return rows >= 0 && rows < grid.length && cols >= 0 && cols < grid[0].length;
    }

    public static char[][] generateGrid(char[][] gridSetup, int gridSizeX, int gridSizeY, int xo, int yo, int goalRow, int goalCol, double percentage) {
        char[][] toGeneGrid = new char[gridSizeX][gridSizeY]; //local variable to prepare to generate grid and print it in the printGrid()
        Random random = new Random();

        for (int i = 0; i < gridSetup.length; i++) {
            for (int j = 0; j < gridSetup[0].length; j++) {
                toGeneGrid[i][j] = (random.nextDouble() < percentage) ? 'o' : '.';
                //toGeneGrid[1][1] = 'o';
                //toGeneGrid[2][0] = 'o';
                //toGeneGrid[i][j] = '.';
            }
        }

        toGeneGrid[xo][yo] = '#';
        toGeneGrid[goalRow][goalCol] = 'G';

        return toGeneGrid;
    }

    public static void printGrid(char[][] toGeneGrid) {
        for (int i = 0; i < toGeneGrid.length; i++) {
            for (int j = 0; j < toGeneGrid[0].length; j++) {
                System.out.print(toGeneGrid[i][j] + " ");
            }
            System.out.println();
        }
    }

    public static final int[][] DIRECTIONS = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

}