package com.graphhopper.routing.util.parsers;

import com.graphhopper.reader.ReaderWay;
import com.graphhopper.routing.ev.BooleanEncodedValue;
import com.graphhopper.routing.ev.EncodedValueLookup;
import com.graphhopper.routing.ev.Roundabout;
import com.graphhopper.routing.ev.VehicleAccess;
import com.graphhopper.routing.util.WayAccess;
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
        // climbfinder: a racing bike gets through these barriers
        allowedBarriers.addAll(Arrays.asList("lift_gate", "gate", "sally_port", "toll_booth", "block", "swing_gate",
                "cattle_grid", "cycle_barrier", "kissing_gate"));
    }

    @Override
    public WayAccess getAccess(ReaderWay way) {
        WayAccess access = super.getAccess(way);
        if (!access.canSkip() && way.getTag("highway") != null && isBlockedBySacScale(way))
            return WayAccess.CAN_SKIP;
        if (!access.canSkip() && way.hasTag("highway", "bridleway") && !isTaggedForBicycles(way))
            return WayAccess.CAN_SKIP;
        return access;
    }

    /**
     * climbfinder: a racing bike cannot ride a bridleway unless the way is explicitly tagged for bicycles. Upstream
     * dropped this rule from the parser (graphhopper/graphhopper#3067) and blocks bridleways in its custom model for
     * Germany only.
     */
    private boolean isTaggedForBicycles(ReaderWay way) {
        if (way.hasTag("bicycle", "dismount"))
            return true;
        for (String value : way.getTag("bicycle", "").split(";"))
            if (allowedValues.contains(value))
                return true;
        return false;
    }

    /**
     * climbfinder: upstream moved sac_scale into the custom models (hike_rating), which cannot express our exception
     * for ways that carry a bicycle tag, so it stays an access rule for racing bikes.
     */
    static boolean isBlockedBySacScale(ReaderWay way) {
        String sacScale = way.getTag("sac_scale");
        // scales above hiking are nearly impossible on a bike, unless the way is tagged for bicycles or paved
        return sacScale != null && !"hiking".equals(sacScale)
                && !way.hasTag("bicycle") && !way.hasTag("surface", "asphalt");
    }
}
