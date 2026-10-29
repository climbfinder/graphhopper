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
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * Synthetic matched paths around a main road running east along latitude 50 from longitude 4.000 to 4.002, with a side
 * street going north from longitude 4.001. Edge keys: 2 and 4 are the main road, 10 and 12 the side street northwards
 * (11 and 13 southwards).
 */
public class SpurTrimmerTest {

    private static final double LAT_PER_METER = 1 / (6371008.8 * Math.PI / 180);

    private static PointList line(double... latLons) {
        PointList points = new PointList();
        for (int i = 0; i < latLons.length; i += 2)
            points.add(latLons[i], latLons[i + 1]);
        return points;
    }

    private static MatchWaypoints.Stretch stretch(int key, double lat0, double lon0, double lat1, double lon1) {
        return new MatchWaypoints.Stretch(key, -1, true, line(lat0, lon0, lat1, lon1));
    }

    private static double north(double meters) {
        return 50.0 + meters * LAT_PER_METER;
    }

    private static int[] keys(List<MatchWaypoints.Stretch> stretches) {
        return stretches.stream().mapToInt(s -> s.edgeKey).toArray();
    }

    /** Main road, side street out to {@code reach} metres and back (in pieces ending at {@code splits}), main road. */
    private static List<MatchWaypoints.Stretch> pathWithSpur(double... splits) {
        List<MatchWaypoints.Stretch> path = new ArrayList<>();
        path.add(stretch(2, 50.0, 4.000, 50.0, 4.001));
        double from = 0;
        for (int i = 0; i < splits.length; i++) {
            path.add(stretch(10 + 2 * i, north(from), 4.001, north(splits[i]), 4.001));
            from = splits[i];
        }
        for (int i = splits.length - 1; i >= 0; i--) {
            double to = i == 0 ? 0 : splits[i - 1];
            path.add(stretch(11 + 2 * i, north(splits[i]), 4.001, north(to), 4.001));
        }
        path.add(stretch(4, 50.0, 4.001, 50.0, 4.002));
        return path;
    }

    private static RouteDeviation.LineIndex gpx(PointList line) {
        return new RouteDeviation.LineIndex(line);
    }

    private static final PointList GPX_ALONG_MAIN_ROAD = line(50.0, 4.000, 50.0, 4.002);

    @Test
    public void spurOffTheGpxIsTrimmed() {
        List<MatchWaypoints.Stretch> path = pathWithSpur(100);
        List<SpurTrimmer.TrimmedSpur> trimmed = SpurTrimmer.trim(path, gpx(GPX_ALONG_MAIN_ROAD));
        assertEquals(List.of(2, 4), List.of(keys(path)[0], keys(path)[1]));
        assertEquals(2, path.size());
        assertEquals(1, trimmed.size());
        assertEquals(200, trimmed.get(0).length, 0.5);
        assertEquals(100, trimmed.get(0).apexDistance, 0.5);
        assertEquals(north(100), trimmed.get(0).apex.lat, 1e-9);
    }

    @Test
    public void outAndBackThatTheGpxRidesIsKept() {
        // the ride itself goes up the side street and comes back, like an out-and-back climb
        PointList gpx = line(50.0, 4.000, 50.0, 4.001, north(100), 4.001, 50.0, 4.001, 50.0, 4.002);
        List<MatchWaypoints.Stretch> path = pathWithSpur(100);
        assertEquals(0, SpurTrimmer.trim(path, gpx(gpx)).size());
        assertEquals(4, path.size());
    }

    @Test
    public void spurWithItsApexWithinToleranceIsKept() {
        List<MatchWaypoints.Stretch> path = pathWithSpur(12);
        assertEquals(0, SpurTrimmer.trim(path, gpx(GPX_ALONG_MAIN_ROAD)).size());
        assertEquals(4, path.size());
    }

    @Test
    public void onlyThePartBeyondWhatTheGpxRidesIsTrimmed() {
        // the ride goes 100 m up the side street and back, the match goes on to 200 m before it turns
        PointList gpx = line(50.0, 4.000, 50.0, 4.001, north(100), 4.001, 50.0, 4.001, 50.0, 4.002);
        List<MatchWaypoints.Stretch> path = pathWithSpur(100, 200);
        List<SpurTrimmer.TrimmedSpur> trimmed = SpurTrimmer.trim(path, gpx(gpx));
        assertEquals(1, trimmed.size());
        assertEquals(200, trimmed.get(0).length, 0.5);
        int[] keys = keys(path);
        assertEquals(4, keys.length);
        assertEquals(List.of(2, 10, 11, 4), List.of(keys[0], keys[1], keys[2], keys[3]));
    }

    @Test
    public void anEdgeWhoseFarEndIsOffIsRemovedWhole() {
        // the side street has a bend 10 m from the main road, on the GPX, but the path can only be cut at nodes: the
        // edge goes because its far end is 100 m off, including the 10 m the GPX might ride
        List<MatchWaypoints.Stretch> path = new ArrayList<>();
        path.add(stretch(2, 50.0, 4.000, 50.0, 4.001));
        path.add(new MatchWaypoints.Stretch(10, -1, true, line(50.0, 4.001, north(10), 4.001, north(100), 4.001)));
        path.add(new MatchWaypoints.Stretch(11, -1, true, line(north(100), 4.001, north(10), 4.001, 50.0, 4.001)));
        path.add(stretch(4, 50.0, 4.001, 50.0, 4.002));
        assertEquals(1, SpurTrimmer.trim(path, gpx(GPX_ALONG_MAIN_ROAD)).size());
        assertEquals(2, path.size());
    }

    @Test
    public void theStartAndEndOfThePathAreNeverTrimmed() {
        // the path starts by going up the side street and back: there is nothing before it to keep it connected to
        List<MatchWaypoints.Stretch> path = new ArrayList<>();
        path.add(stretch(10, 50.0, 4.001, north(100), 4.001));
        path.add(stretch(11, north(100), 4.001, 50.0, 4.001));
        path.add(stretch(4, 50.0, 4.001, 50.0, 4.002));
        assertEquals(0, SpurTrimmer.trim(path, gpx(GPX_ALONG_MAIN_ROAD)).size());
        assertEquals(3, path.size());
    }
}
