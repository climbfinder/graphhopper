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

import com.graphhopper.util.GHUtility;
import com.graphhopper.util.shapes.GHPoint;

import java.util.ArrayList;
import java.util.List;

/**
 * Removes out-and-back spurs that the matcher adds but the GPX does not ride.
 * <p>
 * Where GPX points lie far from any road (a track drawn through buildings, a path missing in OSM), every point still
 * gets a candidate, only on the nearest road around, and the matcher visits each of them: it rides out along some edges
 * and back over the same ones. Such a retrace is trimmed from its turning point outwards, one pair of edges (out and
 * back) at a time, while the pair's end towards the turning point lies more than {@link #TOLERANCE_METERS} from the
 * GPX. The trim thus stops at the first node, going outwards from the turning point, that lies within the tolerance.
 * The part of a retrace that the GPX itself rides up to that node is kept, so an out-and-back of the ride itself (up a
 * climb and back down the same road) stays whole. The path can only be cut at nodes, which keeps it connected so that
 * routing can reproduce it: it turns back at the node where the trim stopped, or goes straight on where the whole spur
 * was removed. The price is that an edge whose far end is off the GPX is removed whole, also when the GPX rides its
 * first part.
 */
final class SpurTrimmer {

    static final double TOLERANCE_METERS = 15;

    static final class TrimmedSpur {
        /** Where the matched path turned back. */
        final GHPoint apex;
        /** Distance from the turning point to the GPX, in metres. */
        final double apexDistance;
        /** Length of the removed part, out and back, in metres. */
        final double length;

        TrimmedSpur(GHPoint apex, double apexDistance, double length) {
            this.apex = apex;
            this.apexDistance = apexDistance;
            this.length = length;
        }
    }

    private SpurTrimmer() {
    }

    /**
     * Removes the off-GPX part of every retrace from {@code stretches} (in place) and returns what was removed. The
     * first and last stretch are never removed, so the path keeps its start and end.
     */
    static List<TrimmedSpur> trim(List<MatchWaypoints.Stretch> stretches, RouteDeviation.LineIndex gpx) {
        List<TrimmedSpur> trimmed = new ArrayList<>();
        boolean changed = true;
        while (changed) {
            changed = false;
            for (int k = 1; k < stretches.size(); k++) {
                if (stretches.get(k).edgeKey != GHUtility.reverseEdgeKey(stretches.get(k - 1).edgeKey))
                    continue;
                GHPoint apex = new GHPoint(stretches.get(k).geometry.getLat(0), stretches.get(k).geometry.getLon(0));
                double apexDistance = gpx.distance(apex.lat, apex.lon);
                if (apexDistance <= TOLERANCE_METERS)
                    continue;
                // pair j is the outgoing stretch k-1-j and the returning stretch k+j
                int pairs = 0;
                while (k - 1 - pairs >= 1 && k + pairs <= stretches.size() - 2
                        && stretches.get(k + pairs).edgeKey == GHUtility.reverseEdgeKey(stretches.get(k - 1 - pairs).edgeKey)
                        && isOff(stretches.get(k - 1 - pairs), gpx))
                    pairs++;
                if (pairs == 0)
                    continue;
                List<MatchWaypoints.Stretch> removed = stretches.subList(k - pairs, k + pairs);
                double length = 0;
                for (MatchWaypoints.Stretch stretch : removed)
                    length += stretch.length;
                removed.clear();
                trimmed.add(new TrimmedSpur(apex, apexDistance, length));
                changed = true;
                break;
            }
        }
        return trimmed;
    }

    /** Whether the end of the stretch (towards the turning point) lies more than the tolerance from the GPX. */
    private static boolean isOff(MatchWaypoints.Stretch stretch, RouteDeviation.LineIndex gpx) {
        int last = stretch.geometry.size() - 1;
        return gpx.distance(stretch.geometry.getLat(last), stretch.geometry.getLon(last)) > TOLERANCE_METERS;
    }
}
