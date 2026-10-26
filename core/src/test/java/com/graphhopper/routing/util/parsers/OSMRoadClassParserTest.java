package com.graphhopper.routing.util.parsers;

import com.graphhopper.reader.ReaderWay;
import com.graphhopper.routing.ev.*;
import com.graphhopper.storage.IntsRef;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class OSMRoadClassParserTest {
    private EnumEncodedValue<RoadClass> rcEnc;
    private OSMRoadClassParser parser;
    private IntsRef relFlags;

    @BeforeEach
    public void setUp() {
        rcEnc = RoadClass.create();
        rcEnc.init(new EncodedValue.InitializerConfig());
        parser = new OSMRoadClassParser(rcEnc);
        relFlags = new IntsRef(2);
    }

    @Test
    public void testSimpleTags() {
        ReaderWay readerWay = new ReaderWay(1);
        ArrayEdgeIntAccess intAccess = new ArrayEdgeIntAccess(1);
        int edgeId = 0;
        readerWay.setTag("highway", "primary");
        parser.handleWayTags(edgeId, intAccess, readerWay, relFlags);
        assertEquals(RoadClass.PRIMARY, rcEnc.getEnum(false, edgeId, intAccess));

        intAccess = new ArrayEdgeIntAccess(1);
        readerWay.setTag("highway", "unknownstuff");
        parser.handleWayTags(edgeId, intAccess, readerWay, relFlags);
        assertEquals(RoadClass.OTHER, rcEnc.getEnum(false, edgeId, intAccess));

        intAccess = new ArrayEdgeIntAccess(1);
        readerWay.setTag("highway", "motorway_link");
        parser.handleWayTags(edgeId, intAccess, readerWay, relFlags);
        assertEquals(RoadClass.MOTORWAY, rcEnc.getEnum(false, edgeId, intAccess));

        readerWay = new ReaderWay(1);
        readerWay.setTag("highway", "cycleway");
        intAccess = new ArrayEdgeIntAccess(1);
        parser.handleWayTags(edgeId, intAccess, readerWay, relFlags);
        assertEquals(RoadClass.CYCLEWAY, rcEnc.getEnum(false, edgeId, intAccess));
    }

    @Test
    public void climbfinderCycleLanesAndDesignatedWaysAreCycleway() {
        assertEquals(RoadClass.CYCLEWAY, parse("secondary", "cycleway:right", "lane"));
        assertEquals(RoadClass.CYCLEWAY, parse("secondary", "cycleway:left", "track"));
        assertEquals(RoadClass.CYCLEWAY, parse("tertiary", "cycleway:both", "lane"));
        assertEquals(RoadClass.CYCLEWAY, parse("residential", "cycleway", "track"));
        assertEquals(RoadClass.CYCLEWAY, parse("footway", "bicycle", "designated"));
        // cycleway=opposite only allows contraflow cycling on a oneway street
        assertEquals(RoadClass.RESIDENTIAL, parse("residential", "cycleway", "opposite"));
        // ported unchanged from cf-0.3: every other cycleway value counts, "no" included
        assertEquals(RoadClass.CYCLEWAY, parse("secondary", "cycleway", "no"));
        assertEquals(RoadClass.OTHER, parse(null, "cycleway", "lane"));
    }

    private RoadClass parse(String highway, String key, String value) {
        ReaderWay readerWay = new ReaderWay(1);
        if (highway != null)
            readerWay.setTag("highway", highway);
        readerWay.setTag(key, value);
        ArrayEdgeIntAccess intAccess = new ArrayEdgeIntAccess(1);
        parser.handleWayTags(0, intAccess, readerWay, relFlags);
        return rcEnc.getEnum(false, 0, intAccess);
    }

    @Test
    public void testIgnore() {
        ReaderWay readerWay = new ReaderWay(1);
        EdgeIntAccess edgeIntAccess = new ArrayEdgeIntAccess(1);
        int edgeId = 0;
        readerWay.setTag("route", "ferry");
        parser.handleWayTags(edgeId, edgeIntAccess, readerWay, relFlags);
        assertEquals(RoadClass.OTHER, rcEnc.getEnum(false, edgeId, edgeIntAccess));
    }

    @Test
    public void testNoNPE() {
        ReaderWay readerWay = new ReaderWay(1);
        ArrayEdgeIntAccess intAccess = new ArrayEdgeIntAccess(1);
        int edgeId = 0;
        parser.handleWayTags(edgeId, intAccess, readerWay, relFlags);
        assertEquals(RoadClass.OTHER, rcEnc.getEnum(false, edgeId, intAccess));
    }
}