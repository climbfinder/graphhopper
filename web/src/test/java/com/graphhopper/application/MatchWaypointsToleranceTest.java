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
package com.graphhopper.application;

import com.graphhopper.util.PointList;
import org.junit.jupiter.api.Test;

import static com.graphhopper.matching.MatchWaypoints.withinTolerance;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class MatchWaypointsToleranceTest {

    // longitude degrees per metre at latitude 50
    private static final double LON_PER_METER = 1 / (6371008.8 * Math.PI / 180 * Math.cos(Math.toRadians(50)));

    private static PointList line(double... latLons) {
        PointList points = new PointList();
        for (int i = 0; i < latLons.length; i += 2)
            points.add(latLons[i], latLons[i + 1]);
        return points;
    }

    /** A line of about 1.1 km north along longitude 4 plus an eastward offset. */
    private static PointList northbound(double eastMeters) {
        return line(50.000, 4.0 + eastMeters * LON_PER_METER, 50.010, 4.0 + eastMeters * LON_PER_METER);
    }

    @Test
    public void theOtherCarriagewayIsWithinTolerance() {
        assertTrue(withinTolerance(northbound(10), northbound(0)));
    }

    @Test
    public void aParallelRoadFurtherAwayIsNot() {
        assertFalse(withinTolerance(northbound(20), northbound(0)));
    }

    @Test
    public void aShortDetourWithinTheCorridorIsNotWhenItAddsLength() {
        // zigzag within 12 m of the line: every point is close, but it is about 25 m (2 %) longer
        PointList zigzag = new PointList();
        for (int i = 0; i <= 20; i++)
            zigzag.add(50.000 + 0.0005 * i, 4.0 + (i % 2 == 0 ? 0 : 12 * LON_PER_METER));
        assertFalse(withinTolerance(zigzag, northbound(0)));
    }

    @Test
    public void aLegThatStopsShortIsNot() {
        assertFalse(withinTolerance(line(50.000, 4.0, 50.009, 4.0), northbound(0)));
    }
}
