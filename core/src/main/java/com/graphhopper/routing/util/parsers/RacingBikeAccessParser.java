package com.graphhopper.routing.util.parsers;

import com.graphhopper.routing.ev.BooleanEncodedValue;
import com.graphhopper.routing.ev.EncodedValueLookup;
import com.graphhopper.routing.ev.Roundabout;
import com.graphhopper.routing.ev.VehicleAccess;
import com.graphhopper.util.PMap;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class RacingBikeAccessParser extends BikeCommonAccessParser {

    // climbfinder: only the bicycle tag restricts a racing bike, access and vehicle do not
    private static final List<String> RESTRICTIONS = Collections.singletonList("bicycle");

    public RacingBikeAccessParser(EncodedValueLookup lookup, PMap properties) {
        this(lookup.getBooleanEncodedValue(VehicleAccess.key("racingbike")),
                lookup.getBooleanEncodedValue(Roundabout.KEY));
        blockPrivate(properties.getBool("block_private", true));
        blockFords(properties.getBool("block_fords", false));
    }

    protected RacingBikeAccessParser(BooleanEncodedValue accessEnc, BooleanEncodedValue roundaboutEnc) {
        super(accessEnc, roundaboutEnc, RESTRICTIONS);

        barriers.add("kissing_gate");
        barriers.add("stile");
        barriers.add("turnstile");

        // climbfinder: bicycle=restricted does not block a racing bike
        restrictedValues.remove("restricted");
        // climbfinder: neither do agricultural, forestry and delivery
        restrictedValues.removeAll(Arrays.asList("agricultural", "forestry", "delivery"));
    }
}
