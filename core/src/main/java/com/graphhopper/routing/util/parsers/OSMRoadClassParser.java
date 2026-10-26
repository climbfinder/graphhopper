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
package com.graphhopper.routing.util.parsers;

import com.graphhopper.reader.ReaderWay;
import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.routing.ev.EdgeIntAccess;
import com.graphhopper.routing.ev.RoadClass;
import com.graphhopper.storage.IntsRef;

import static com.graphhopper.routing.ev.RoadClass.CYCLEWAY;
import static com.graphhopper.routing.ev.RoadClass.OTHER;

public class OSMRoadClassParser implements TagParser {

    protected final EnumEncodedValue<RoadClass> roadClassEnc;

    public OSMRoadClassParser(EnumEncodedValue<RoadClass> roadClassEnc) {
        this.roadClassEnc = roadClassEnc;
    }

    @Override
    public void handleWayTags(int edgeId, EdgeIntAccess edgeIntAccess, ReaderWay readerWay, IntsRef relationFlags) {
        String roadClassTag = readerWay.getTag("highway");
        if (roadClassTag == null)
            return;
        // climbfinder: roads with a cycle lane or track count as cycleway, so a custom model can prefer them
        if (isCycleInfrastructure(readerWay)) {
            roadClassEnc.setEnum(false, edgeId, edgeIntAccess, CYCLEWAY);
            return;
        }
        RoadClass roadClass = RoadClass.find(roadClassTag);
        if (roadClass == OTHER && roadClassTag.endsWith("_link"))
            roadClass = RoadClass.find(roadClassTag.substring(0, roadClassTag.length() - 5));

        if (roadClass != OTHER)
            roadClassEnc.setEnum(false, edgeId, edgeIntAccess, roadClass);
    }

    static boolean isCycleInfrastructure(ReaderWay way) {
        boolean hasCycleway = way.hasTag("cycleway") || way.hasTag("cycleway:left")
                || way.hasTag("cycleway:right") || way.hasTag("cycleway:both");
        // cycleway=opposite only allows contraflow cycling on a oneway street, it is no cycle infrastructure
        return hasCycleway && !way.hasTag("cycleway", "opposite") || way.hasTag("bicycle", "designated");
    }
}
