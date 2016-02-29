package roadgraph;

import geography.GeographicPoint;

import java.util.ArrayList;
import java.util.List;

public class Intersection extends GeographicPoint {

    private List<Road> roads;
    private double distanceFromStart = java.lang.Double.POSITIVE_INFINITY;


    public Intersection(GeographicPoint location) {
        super(location.getX(), location.getY());
        roads = new ArrayList<>();
    }

    public List<Road> getRoads() {
        return roads;
    }

    public void addEdge(Intersection destination, String roadName, String roadType, double length) {
        roads.add(new Road(roadName, roadType, destination, length));
    }

    public void setDistanceFromStart(double distanceFromStart) {
        this.distanceFromStart = distanceFromStart;
    }

    public double getDistanceFromStart() {
        return distanceFromStart;
    }

    public GeographicPoint getGeographicPoint() {
        return this;
    }
}
