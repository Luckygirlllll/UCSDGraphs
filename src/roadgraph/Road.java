package roadgraph;

public class Road {

    private String roadName;
    private String roadType;
    private Intersection destination;
    private double length;

    public Road(String roadName, String roadType, Intersection destination, double length) {
        this.roadName = roadName;
        this.roadType = roadType;
        this.destination = destination;
        this.length = length;
    }

    public Intersection getDestination() {
        return destination;
    }

    public double getLength() {
        return length;
    }

    @Override
    public String toString() {
        return "Road [" + roadName + ", " + length + "]";
    }
}
