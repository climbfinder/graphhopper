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
package com.graphhopper.application.resources;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.graphhopper.application.GraphHopperApplication;
import com.graphhopper.application.GraphHopperServerConfiguration;
import com.graphhopper.jackson.Jackson;
import com.graphhopper.routing.TestProfiles;
import com.graphhopper.util.Helper;
import io.dropwizard.testing.junit5.DropwizardAppExtension;
import io.dropwizard.testing.junit5.DropwizardExtensionsSupport;
import jakarta.ws.rs.client.Entity;
import jakarta.ws.rs.core.Response;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static com.graphhopper.application.util.TestUtils.clientTarget;
import static org.junit.jupiter.api.Assertions.*;

@ExtendWith(DropwizardExtensionsSupport.class)
public class MatchWaypointsResourceTest {

    private static final String DIR = "../target/match-waypoints-test";
    private static final ObjectMapper MAPPER = Jackson.newObjectMapper();
    public static final DropwizardAppExtension<GraphHopperServerConfiguration> app = new DropwizardAppExtension<>(GraphHopperApplication.class, createConfig());

    private static GraphHopperServerConfiguration createConfig() {
        GraphHopperServerConfiguration config = new GraphHopperServerConfiguration();
        config.getGraphHopperConfiguration().
                putObject("datareader.file", "../map-matching/files/leipzig_germany.osm.pbf").
                putObject("import.osm.ignored_highways", "").
                putObject("graph.location", DIR).
                putObject("graph.encoded_values", "bike_access, bike_priority, bike_average_speed, road_class, road_environment, surface").
                putObject("routing.snap_preventions_default", "tunnel, bridge, ferry").
                setProfiles(List.of(TestProfiles.accessSpeedAndPriority("bike", "bike")));
        return config;
    }

    @AfterAll
    public static void cleanUp() {
        Helper.removeDir(new File(DIR));
    }

    /** The routing options the route planner sends, with the planner's custom model. */
    private static ObjectNode plannerOptions() {
        ObjectNode options = MAPPER.createObjectNode();
        options.put("profile", "bike");
        options.put("ch.disable", true);
        options.put("pass_through", true);
        options.put("heading_penalty", 120);
        options.put("points_encoded", false);
        options.put("instructions", false);
        options.putArray("details").add("road_class").add("surface").add("road_environment");
        ObjectNode customModel = options.putObject("custom_model");
        ArrayNode priority = customModel.putArray("priority");
        String[][] roadClasses = {{"CYCLEWAY", "0.8"}, {"PATH", "0.25"}, {"TRACK", "0.25"}, {"STEPS", "0.005"},
                {"FOOTWAY", "0.1"}, {"RESIDENTIAL", "0.8"}, {"LIVING_STREET", "0.6"}, {"SERVICE", "0.5"}};
        for (String[] rc : roadClasses)
            priority.addObject().put("if", "road_class == " + rc[0]).put("multiply_by", rc[1]);
        priority.addObject().put("if", "road_environment == FERRY").put("multiply_by", "0.5");
        customModel.put("distance_influence", 120);
        return options;
    }

    private static List<double[]> route(double... latLons) {
        ObjectNode request = plannerOptions();
        ArrayNode points = request.putArray("points");
        for (int i = 0; i < latLons.length; i += 2)
            points.addArray().add(latLons[i + 1]).add(latLons[i]);
        JsonNode json = clientTarget(app, "/route").request().post(Entity.json(request), JsonNode.class);
        return lonLats(json.get("paths").get(0).get("points").get("coordinates"));
    }

    private static List<double[]> lonLats(JsonNode coordinates) {
        List<double[]> result = new ArrayList<>();
        for (JsonNode c : coordinates)
            result.add(new double[]{c.get(0).asDouble(), c.get(1).asDouble()});
        return result;
    }

    /** A GPX track along the line, with a track point about every 10 m and one second apart. */
    private static String gpx(List<double[]> lonLats) {
        StringBuilder sb = new StringBuilder("<?xml version=\"1.0\" encoding=\"UTF-8\"?><gpx version=\"1.1\" xmlns=\"http://www.topografix.com/GPX/1/1\"><trk><trkseg>");
        long time = 1_700_000_000_000L;
        for (int i = 0; i < lonLats.size(); i++) {
            double[] b = lonLats.get(i);
            int pieces = 1;
            if (i > 0) {
                double[] a = lonLats.get(i - 1);
                double meters = Math.hypot((b[0] - a[0]) * 111_320 * Math.cos(Math.toRadians(b[1])), (b[1] - a[1]) * 110_540);
                pieces = Math.max(1, (int) Math.ceil(meters / 10));
            }
            for (int k = (i == 0 ? pieces : 1); k <= pieces; k++) {
                double lon = i == 0 ? b[0] : lonLats.get(i - 1)[0] + (b[0] - lonLats.get(i - 1)[0]) * k / pieces;
                double lat = i == 0 ? b[1] : lonLats.get(i - 1)[1] + (b[1] - lonLats.get(i - 1)[1]) * k / pieces;
                sb.append(String.format(Locale.ROOT, "<trkpt lat=\"%.6f\" lon=\"%.6f\"><time>%s</time></trkpt>", lat, lon, java.time.Instant.ofEpochMilli(time)));
                time += 1000;
            }
        }
        return sb.append("</trkseg></trk></gpx>").toString();
    }

    private static Response post(ObjectNode body) {
        return clientTarget(app, "/match/waypoints").request().buildPost(Entity.json(body)).invoke();
    }

    private static JsonNode matchWaypoints(String gpx) {
        ObjectNode body = plannerOptions();
        body.put("gpx", gpx);
        try (Response response = post(body)) {
            JsonNode json = response.readEntity(JsonNode.class);
            assertEquals(200, response.getStatus(), json.toString());
            return json;
        }
    }

    private static JsonNode diagnostics(JsonNode json) {
        return json.get("diagnostics");
    }

    private static void assertReproducesMatch(JsonNode json) {
        JsonNode deviation = diagnostics(json).get("deviation");
        assertEquals(100.0, deviation.get("route_on_match").get("within_percent").asDouble(), json.toString());
        assertEquals(100.0, deviation.get("match_on_route").get("within_percent").asDouble(), json.toString());
        assertEquals(0, deviation.get("length_difference_percent").asDouble(), 0.1, json.toString());
    }

    private static List<double[]> waypoints(JsonNode json) {
        List<double[]> result = new ArrayList<>();
        for (JsonNode w : json.get("waypoints"))
            result.add(new double[]{w.get("lon").asDouble(), w.get("lat").asDouble()});
        return result;
    }

    @Test
    public void routeOfTheRouterNeedsOnlyStartAndEnd() {
        JsonNode json = matchWaypoints(gpx(route(51.343324, 12.359187, 51.330, 12.380)));
        assertEquals(2, json.get("waypoints").size(), json.toString());
        assertEquals(0, diagnostics(json).get("forced_count").asInt());
        assertTrue(diagnostics(json).get("route_evaluations").asInt() > 0);
        assertReproducesMatch(json);
    }

    @Test
    public void detourGetsOneWaypointAndTheRouteHasTheRouteShape() {
        // a route via 51.3375,12.3720 that the router would not take from start to end
        JsonNode json = matchWaypoints(gpx(route(51.343324, 12.359187, 51.3375, 12.3720, 51.330, 12.380)));
        assertEquals(3, json.get("waypoints").size(), json.toString());
        assertEquals(0, diagnostics(json).get("forced_count").asInt());
        assertReproducesMatch(json);

        JsonNode path = json.get("route").get("paths").get(0);
        assertEquals(3, path.get("snapped_waypoints").get("coordinates").size());
        assertTrue(path.get("details").has("road_class"));
        assertTrue(path.get("distance").asDouble() > 2000);
        assertTrue(json.get("match").get("points").get("coordinates").size() > 2);
    }

    @Test
    public void feedingTheRouteBackGivesTheSameWaypoints() {
        JsonNode first = matchWaypoints(gpx(route(51.343324, 12.359187, 51.3375, 12.3720, 51.330, 12.380)));
        List<double[]> routed = lonLats(first.get("route").get("paths").get(0).get("points").get("coordinates"));
        JsonNode second = matchWaypoints(gpx(routed));
        List<double[]> a = waypoints(first), b = waypoints(second);
        assertEquals(a.size(), b.size(), first.get("waypoints") + " vs " + second.get("waypoints"));
        for (int i = 0; i < a.size(); i++) {
            assertEquals(a.get(i)[0], b.get(i)[0], 1e-5, "lon of waypoint " + i);
            assertEquals(a.get(i)[1], b.get(i)[1], 1e-5, "lat of waypoint " + i);
        }
        assertReproducesMatch(second);
    }

    @Test
    public void turningBackOnAStreetGivesForcedWaypointsWithReason() throws Exception {
        // the track rides into a street and back; with pass_through the router will not turn back on it
        String gpx = new String(getClass().getResourceAsStream("/tour4-with-uturn.gpx").readAllBytes(), java.nio.charset.StandardCharsets.UTF_8);
        JsonNode json = matchWaypoints(gpx);
        JsonNode forced = diagnostics(json).get("forced_waypoints");
        assertTrue(forced.size() > 0, json.toString());
        assertEquals(forced.size(), diagnostics(json).get("forced_count").asInt());
        for (JsonNode f : forced) {
            assertEquals("u_turn", f.get("reason").asText(), f.toString());
            assertFalse(f.get("message").asText().isEmpty());
            assertTrue(json.get("waypoints").get(f.get("index").asInt()).get("forced").asBoolean());
            assertEquals(json.get("waypoints").get(f.get("index").asInt()).get("lat").asDouble(), f.get("lat").asDouble(), 1e-9);
        }
        // the deviation of the final route from the match is reported, not hidden
        assertTrue(diagnostics(json).get("deviation").get("route_on_match").get("within_percent").asDouble() < 100, json.toString());
    }

    @Test
    public void offNetworkStretchIsMatchedOntoTheNetworkAndReproduced() {
        // the middle third of the track cuts straight across blocks, away from the roads
        List<double[]> line = route(51.343324, 12.359187, 51.330, 12.380);
        List<double[]> offNetwork = new ArrayList<>(line.subList(0, line.size() / 3));
        offNetwork.addAll(line.subList(2 * line.size() / 3, line.size()));
        JsonNode json = matchWaypoints(gpx(offNetwork));
        // the matcher connects the gap over roads; the waypoints reproduce that match, so nothing is forced
        assertEquals(0, diagnostics(json).get("forced_count").asInt(), json.toString());
        assertTrue(json.get("waypoints").size() > 2);
        assertReproducesMatch(json);
    }

    private static void assertBadRequest(ObjectNode body, String expectedMessagePart) {
        try (Response response = post(body)) {
            JsonNode json = response.readEntity(JsonNode.class);
            assertEquals(400, response.getStatus(), json.toString());
            assertTrue(json.get("message").asText().contains(expectedMessagePart), json.toString());
        }
    }

    private static ObjectNode validBody() {
        ObjectNode body = plannerOptions();
        body.put("gpx", gpx(route(51.343324, 12.359187, 51.330, 12.380)));
        return body;
    }

    @Test
    public void missingCustomModelIsABadRequest() {
        ObjectNode body = validBody();
        body.remove("custom_model");
        assertBadRequest(body, "'custom_model' field is required");
    }

    @Test
    public void invalidCustomModelIsABadRequest() {
        ObjectNode body = validBody();
        body.put("custom_model", "racingbike");
        assertBadRequest(body, "'custom_model' must be a JSON object");

        body = validBody();
        ((ArrayNode) body.get("custom_model").get("priority")).addObject().put("if", "no_such_value == 3").put("multiply_by", "0.5");
        assertBadRequest(body, "no_such_value");
    }

    @Test
    public void missingOrEmptyGpxIsABadRequest() {
        ObjectNode body = validBody();
        body.remove("gpx");
        assertBadRequest(body, "'gpx' field is required");

        body = validBody();
        body.put("gpx", "this is not xml");
        assertBadRequest(body, "Cannot read the GPX document");

        body = validBody();
        body.put("gpx", "<?xml version=\"1.0\"?><gpx version=\"1.1\" xmlns=\"http://www.topografix.com/GPX/1/1\"><wpt lat=\"51.34\" lon=\"12.36\"/></gpx>");
        assertBadRequest(body, "No tracks found");

        body = validBody();
        body.put("gpx", "<?xml version=\"1.0\"?><gpx version=\"1.1\" xmlns=\"http://www.topografix.com/GPX/1/1\"><trk><trkseg></trkseg></trk></gpx>");
        assertBadRequest(body, "at least two track points");
    }

    @Test
    public void pointsAreABadRequest() {
        ObjectNode body = validBody();
        body.putArray("points").addArray().add(12.36).add(51.34);
        assertBadRequest(body, "Do not send 'points'");
    }

    @Test
    public void missingProfileIsABadRequest() {
        ObjectNode body = validBody();
        body.remove("profile");
        assertBadRequest(body, "'profile' parameter is required");
    }
}
