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

import com.graphhopper.matching.RouteDeviation;
import com.graphhopper.util.PointList;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class RouteDeviationTest {

    // 0.001 degrees of latitude on a sphere of radius 6371008.8 m
    private static final double METERS_PER_MILLIDEGREE = 6371008.8 * Math.PI / 180 / 1000;

    private static PointList line(double... latLons) {
        PointList points = new PointList();
        for (int i = 0; i < latLons.length; i += 2)
            points.add(latLons[i], latLons[i + 1]);
        return points;
    }

    @Test
    public void lengthUsesThePlanarProjection() {
        assertEquals(METERS_PER_MILLIDEGREE, RouteDeviation.length(line(50.000, 4.0, 50.001, 4.0)), 1e-6);
    }

    @Test
    public void identicalLinesAreFullyCovered() {
        PointList a = line(50.000, 4.0, 50.001, 4.0, 50.001, 4.002);
        RouteDeviation.Coverage coverage = RouteDeviation.coverage(a, a, 15, 10);
        assertEquals(100, coverage.getWithinPercent(), 1e-9);
        assertEquals(0, coverage.getOffMeters(), 1e-9);
        assertEquals(0, coverage.getMaxMeters(), 1e-6);
    }

    @Test
    public void aDetourShowsInBothDirections() {
        // b goes straight north for ~333 m, a takes a 0.001 degree (~71 m) detour east over its middle third
        PointList b = line(50.000, 4.0, 50.003, 4.0);
        PointList a = line(50.000, 4.0, 50.001, 4.0, 50.001, 4.001, 50.002, 4.001, 50.002, 4.0, 50.003, 4.0);
        RouteDeviation.Coverage aOnB = RouteDeviation.coverage(a, b, 15, 10);
        RouteDeviation.Coverage bOnA = RouteDeviation.coverage(b, a, 15, 10);
        // the detour leg lies 0.001 degrees of longitude east of b
        assertEquals(METERS_PER_MILLIDEGREE * Math.cos(Math.toRadians(50.001)), aOnB.getMaxMeters(), 0.01);
        assertTrue(aOnB.getWithinPercent() < 75, "" + aOnB.getWithinPercent());
        // the middle of b is 0.0005 degrees of latitude from the nearest part of a
        assertEquals(METERS_PER_MILLIDEGREE / 2, bOnA.getMaxMeters(), 0.01);
        assertTrue(bOnA.getWithinPercent() < 100 && bOnA.getWithinPercent() > 50, "" + bOnA.getWithinPercent());
    }
}
