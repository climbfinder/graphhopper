/*
 *  Licensed to GraphHopper GmbH under one or more contributor
 *  license agreements. See the NOTICE file distributed with this work for
 *  additional information regarding copyright ownership.
 *
 *  GraphHopper GmbH licenses this file to you under the Apache License,
 *  Version 2.0 (the "License"); you may not use this file except in
 *  compliance with the License. You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
package com.graphhopper.matching;

import com.graphhopper.util.PointList;

import java.util.*;

/**
 * How closely one line follows another. Line A is densified so that no two points are more than a step apart, and
 * each point is measured against line B. The measure is not symmetric: a stretch of B that A skips only shows up when
 * B is measured against A, so callers report both directions.
 * <p>
 * Lengths and distances use a local planar projection on a sphere of radius 6371008.8 m, so that the numbers can be
 * compared with an external benchmark that uses the same definitions.
 */
public final class RouteDeviation {

    private static final double EARTH_RADIUS_M = 6371008.8;
    private static final double DEG = Math.PI / 180;
    private static final double CELL_DEG = 0.002;

    public static final class Coverage {
        private final int points;
        private final double withinPercent;
        private final double offMeters;
        private final double maxMeters;

        Coverage(int points, double withinPercent, double offMeters, double maxMeters) {
            this.points = points;
            this.withinPercent = withinPercent;
            this.offMeters = offMeters;
            this.maxMeters = maxMeters;
        }

        /** Points of A after densifying. */
        public int getPoints() {
            return points;
        }

        /** Share of A's points within the tolerance of B, in percent. */
        public double getWithinPercent() {
            return withinPercent;
        }

        /** Off points times the densify step: the length of A further than the tolerance from B, in metres. */
        public double getOffMeters() {
            return offMeters;
        }

        /** Largest distance from a point of A to B, in metres. */
        public double getMaxMeters() {
            return maxMeters;
        }
    }

    private RouteDeviation() {
    }

    /** Length of a polyline in metres. */
    public static double length(PointList line) {
        double total = 0;
        for (int i = 1; i < line.size(); i++)
            total += segmentMeters(line.getLat(i - 1), line.getLon(i - 1), line.getLat(i), line.getLon(i));
        return total;
    }

    public static Coverage coverage(PointList a, PointList b, double toleranceMeters, double stepMeters) {
        if (a.size() < 2 || b.size() < 2)
            throw new IllegalArgumentException("coverage needs two lines of at least two points, got " + a.size() + " and " + b.size());
        LineIndex index = new LineIndex(b);
        List<Double> distances = new ArrayList<>();
        distances.add(index.distance(a.getLat(0), a.getLon(0)));
        for (int i = 1; i < a.size(); i++) {
            double lat0 = a.getLat(i - 1), lon0 = a.getLon(i - 1), lat1 = a.getLat(i), lon1 = a.getLon(i);
            // cut segment i-1 -> i into equal pieces of at most stepMeters; point i is the last cut
            int pieces = Math.max(1, (int) Math.ceil(segmentMeters(lat0, lon0, lat1, lon1) / stepMeters));
            for (int k = 1; k <= pieces; k++)
                distances.add(index.distance(lat0 + (lat1 - lat0) * k / pieces, lon0 + (lon1 - lon0) * k / pieces));
        }
        int off = 0;
        double max = 0;
        for (double d : distances) {
            if (d > toleranceMeters)
                off++;
            max = Math.max(max, d);
        }
        int points = distances.size();
        return new Coverage(points, 100.0 * (points - off) / points, off * stepMeters, max);
    }

    private static double segmentMeters(double lat0, double lon0, double lat1, double lon1) {
        double cosLat = Math.cos((lat0 + lat1) / 2 * DEG);
        double dx = (lon1 - lon0) * DEG * EARTH_RADIUS_M * cosLat;
        double dy = (lat1 - lat0) * DEG * EARTH_RADIUS_M;
        return Math.hypot(dx, dy);
    }

    private static double pointToSegmentMeters(double lat, double lon, double latA, double lonA, double latB, double lonB) {
        double scaleX = DEG * EARTH_RADIUS_M * Math.cos(lat * DEG);
        double scaleY = DEG * EARTH_RADIUS_M;
        double ax = (lonA - lon) * scaleX, ay = (latA - lat) * scaleY;
        double bx = (lonB - lon) * scaleX, by = (latB - lat) * scaleY;
        double dx = bx - ax, dy = by - ay;
        double lengthSq = dx * dx + dy * dy;
        double t = lengthSq == 0 ? 0 : Math.max(0, Math.min(1, -(ax * dx + ay * dy) / lengthSq));
        return Math.hypot(ax + t * dx, ay + t * dy);
    }

    /** The segments of a line in a grid, for exact nearest-segment distances. */
    private static final class LineIndex {
        private final PointList line;
        private final Map<Long, List<Integer>> cells = new HashMap<>();
        private final double minCellMeters;
        private int minX = Integer.MAX_VALUE, minY = Integer.MAX_VALUE, maxX = Integer.MIN_VALUE, maxY = Integer.MIN_VALUE;

        LineIndex(PointList line) {
            this.line = line;
            double maxAbsLat = 0;
            for (int i = 0; i < line.size() - 1; i++) {
                int x0 = cell(Math.min(line.getLon(i), line.getLon(i + 1)));
                int x1 = cell(Math.max(line.getLon(i), line.getLon(i + 1)));
                int y0 = cell(Math.min(line.getLat(i), line.getLat(i + 1)));
                int y1 = cell(Math.max(line.getLat(i), line.getLat(i + 1)));
                for (int x = x0; x <= x1; x++)
                    for (int y = y0; y <= y1; y++)
                        cells.computeIfAbsent(key(x, y), k -> new ArrayList<>()).add(i);
                minX = Math.min(minX, x0);
                minY = Math.min(minY, y0);
                maxX = Math.max(maxX, x1);
                maxY = Math.max(maxY, y1);
                maxAbsLat = Math.max(maxAbsLat, Math.max(Math.abs(line.getLat(i)), Math.abs(line.getLat(i + 1))));
            }
            minCellMeters = CELL_DEG * DEG * EARTH_RADIUS_M * Math.cos(Math.min(maxAbsLat, 89) * DEG);
        }

        private static int cell(double degrees) {
            return (int) Math.floor(degrees / CELL_DEG);
        }

        private static long key(int x, int y) {
            return ((long) x << 32) | (y & 0xffffffffL);
        }

        /** Searches square rings of cells around the point until no further ring can hold anything nearer. */
        double distance(double lat, double lon) {
            int px = cell(lon), py = cell(lat);
            int reach = Math.max(0, Math.max(Math.max(px - minX, maxX - px), Math.max(py - minY, maxY - py)));
            Set<Integer> seen = new HashSet<>();
            double best = Double.POSITIVE_INFINITY;
            for (int ring = 0; ring <= reach; ring++) {
                if (ring == 0) {
                    best = visit(px, py, lat, lon, seen, best);
                } else {
                    for (int k = -ring; k <= ring; k++) {
                        best = visit(px + k, py - ring, lat, lon, seen, best);
                        best = visit(px + k, py + ring, lat, lon, seen, best);
                        if (Math.abs(k) != ring) {
                            best = visit(px - ring, py + k, lat, lon, seen, best);
                            best = visit(px + ring, py + k, lat, lon, seen, best);
                        }
                    }
                }
                if (best <= ring * minCellMeters)
                    break;
            }
            return best;
        }

        private double visit(int x, int y, double lat, double lon, Set<Integer> seen, double best) {
            List<Integer> segments = cells.get(key(x, y));
            if (segments == null)
                return best;
            for (int i : segments) {
                if (seen.add(i))
                    best = Math.min(best, pointToSegmentMeters(lat, lon, line.getLat(i), line.getLon(i), line.getLat(i + 1), line.getLon(i + 1)));
            }
            return best;
        }
    }
}
