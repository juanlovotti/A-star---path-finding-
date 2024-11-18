class Node implements Comparable<Node> {

    private final int x, y, goalRow, goalCol, g;

    public Node(int x, int y) {
        this.x = x;
        this.y = y;
        this.goalRow = 0;
        this.goalCol = 0;
        this.g = 0;
    }

    public Node(int x, int y, int goalRow, int goalCol, int g) {
        this.x = x;
        this.y = y;
        this.goalRow = goalRow;
        this.goalCol = goalCol;
        this.g = g;
    }

    int getX() {
        return this.x;
    }

    int getY() {
        return this.y;
    }

    @Override
    public int compareTo(Node nodenxt) {
        if (this.getF() < nodenxt.getF()){
            return -1;
        } else if (this.getF() > nodenxt.getF()){
            return  1;
        } else {
            return 0;
        }
    }

    int getH() {
        return Math.abs(goalRow - x) + Math.abs(goalCol - y);
    }

    int getF() {
        return getG() + getH();
    }

    int getG() {
        return g;
    }
}