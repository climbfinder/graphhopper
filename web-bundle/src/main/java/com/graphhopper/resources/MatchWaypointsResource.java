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
package com.graphhopper.resources;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.fasterxml.jackson.dataformat.xml.XmlMapper;
import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.GraphHopper;
import com.graphhopper.GraphHopperConfig;
import com.graphhopper.http.GHRequestTransformer;
import com.graphhopper.http.ProfileResolver;
import com.graphhopper.jackson.Gpx;
import com.graphhopper.jackson.Jackson;
import com.graphhopper.jackson.ResponsePathSerializer;
import com.graphhopper.matching.*;
import com.graphhopper.util.*;
import com.graphhopper.gpx.GpxConversions;
import com.graphhopper.jackson.MultiException;
import com.graphhopper.util.shapes.GHPoint;
import jakarta.inject.Inject;
import jakarta.ws.rs.Consumes;
import jakarta.ws.rs.POST;
import jakarta.ws.rs.Produces;
import jakarta.ws.rs.core.MediaType;
import jakarta.ws.rs.core.Response;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

import static com.graphhopper.resources.RouteResource.removeLegacyParameters;
import static com.graphhopper.util.Parameters.Routing.CALC_POINTS;
import static com.graphhopper.util.Parameters.Routing.INSTRUCTIONS;

/**
 * Turns a GPX track into the fewest waypoints whose route reproduces the track as snapped to the road network, see
 * {@link MatchWaypoints}.
 * <p>
 * The request is the JSON body of {@code POST /route} without {@code points}, plus the GPX document in {@code gpx}
 * and optionally {@code gps_accuracy} (metres, default 10). Its {@code profile} and {@code custom_model} are required
 * and give the one weighting that both the map matching and the waypoint search use. The response holds the route
 * through the waypoints exactly as {@code POST /route} returns it for the same options ({@code route}), the matched
 * path ({@code match}), the waypoints and diagnostics.
 */
@jakarta.ws.rs.Path("match/waypoints")
public class MatchWaypointsResource {

    /** Distance within which a point of one line counts as on the other, for the deviation diagnostics. */
    static final double DEVIATION_TOLERANCE_METERS = 15;
    static final double DEVIATION_STEP_METERS = 10;

    private static final Logger logger = LoggerFactory.getLogger(MatchWaypointsResource.class);
    private static final List<String> UNSUPPORTED_FIELDS = Arrays.asList("points", "headings", "curbsides", "point_hints");

    private final GraphHopperConfig config;
    private final GraphHopper graphHopper;
    private final ProfileResolver profileResolver;
    private final GHRequestTransformer ghRequestTransformer;
    private final List<String> snapPreventionsDefault;
    private final String osmDate;
    private final ObjectMapper objectMapper = Jackson.newObjectMapper();
    private final XmlMapper xmlMapper = new XmlMapper();

    @Inject
    public MatchWaypointsResource(GraphHopperConfig config, GraphHopper graphHopper, ProfileResolver profileResolver, GHRequestTransformer ghRequestTransformer) {
        this.config = config;
        this.graphHopper = graphHopper;
        this.profileResolver = profileResolver;
        this.ghRequestTransformer = ghRequestTransformer;
        this.snapPreventionsDefault = Arrays.stream(config.getString("routing.snap_preventions_default", "")
                .split(",")).map(String::trim).filter(s -> !s.isEmpty()).toList();
        this.osmDate = graphHopper.getProperties().getAll().get("datareader.data.date");
    }

    @POST
    @Consumes(MediaType.APPLICATION_JSON)
    @Produces(MediaType.APPLICATION_JSON)
    public Response doPost(ObjectNode body) {
        StopWatch total = new StopWatch().start();
        if (body == null)
            throw new IllegalArgumentException("Send a JSON object with the GPX document in 'gpx' and the routing options of POST /route");
        ObjectNode options = body.deepCopy();
        List<Observation> observations = readObservations(options.remove("gpx"));
        double gpsAccuracy = readGpsAccuracy(options.remove("gps_accuracy"));
        GHRequest request = readRoutingOptions(options);

        StopWatch matchWatch = new StopWatch().start();
        MatchWaypoints matchWaypoints = new MatchWaypoints(graphHopper, request);
        MatchResult match = matchWaypoints.match(observations, gpsAccuracy);
        matchWatch.stop();

        StopWatch reconstructWatch = new StopWatch().start();
        MatchWaypoints.Result waypoints = matchWaypoints.reconstruct(match);
        reconstructWatch.stop();

        StopWatch routeWatch = new StopWatch().start();
        GHResponse route = graphHopper.route(withPoints(request, waypoints.getPoints()));
        routeWatch.stop();
        if (route.hasErrors())
            throw new MultiException(route.getErrors());

        PMap hints = request.getHints();
        boolean instructions = hints.getBool(INSTRUCTIONS, true);
        boolean calcPoints = hints.getBool(CALC_POINTS, true);
        boolean elevation = hints.getBool("elevation", false);
        boolean pointsEncoded = hints.getBool("points_encoded", true);
        double pointsEncodedMultiplier = hints.getDouble("points_encoded_multiplier", 1e5);
        PointList matchPoints = match.getMergedPath().calcPoints();
        total.stop();

        ObjectNode json = objectMapper.createObjectNode();
        json.set("route", ResponsePathSerializer.jsonObject(route, new ResponsePathSerializer.Info(config.getCopyrights(), Math.round(total.getMillisDouble()), osmDate),
                instructions, calcPoints, elevation, pointsEncoded, pointsEncodedMultiplier));
        ObjectNode matchJson = json.putObject("match");
        matchJson.put("distance", match.getMatchLength());
        matchJson.put("gpx_distance", match.getGpxEntriesLength());
        matchJson.put("points_encoded", pointsEncoded);
        if (pointsEncoded)
            matchJson.put("points", ResponsePathSerializer.encodePolyline(matchPoints, elevation, pointsEncodedMultiplier));
        else
            matchJson.putPOJO("points", matchPoints.toLineString(elevation));
        ArrayNode waypointsJson = json.putArray("waypoints");
        for (MatchWaypoints.Waypoint waypoint : waypoints.getWaypoints())
            waypointsJson.addObject().put("lat", waypoint.getPoint().lat).put("lon", waypoint.getPoint().lon).put("forced", waypoint.isForced());
        json.set("diagnostics", diagnostics(waypoints, route, matchPoints, matchWatch, reconstructWatch, routeWatch, total));

        logger.info(objectMapper.createObjectNode()
                .put("duration_ms", Math.round(total.getMillisDouble()))
                .put("profile", request.getProfile())
                .put("observations", observations.size())
                .put("waypoints", waypoints.getWaypoints().size())
                .put("route_evaluations", waypoints.getRouteEvaluations()).toString());
        return Response.ok(json).header("X-GH-Took", "" + Math.round(total.getMillisDouble())).build();
    }

    private List<Observation> readObservations(JsonNode gpxNode) {
        if (gpxNode == null || !gpxNode.isTextual() || gpxNode.asText().isBlank())
            throw new IllegalArgumentException("The 'gpx' field is required: the GPX document as a string");
        Gpx gpx;
        try {
            gpx = xmlMapper.readValue(gpxNode.asText(), Gpx.class);
        } catch (IOException e) {
            throw new IllegalArgumentException("Cannot read the GPX document in 'gpx': " + e.getMessage());
        }
        if (gpx.trk == null || gpx.trk.isEmpty())
            throw new IllegalArgumentException("No tracks found in GPX document. Are you using waypoints or routes instead?");
        if (gpx.trk.size() > 1)
            throw new IllegalArgumentException("GPX documents with multiple tracks not supported yet.");
        List<Observation> observations = GpxConversions.getEntries(gpx.trk.get(0));
        if (observations.size() < 2)
            throw new IllegalArgumentException("The GPX track needs at least two track points, got " + observations.size());
        return observations;
    }

    private static double readGpsAccuracy(JsonNode node) {
        if (node == null || node.isNull())
            return 10;
        if (!node.isNumber() || node.asDouble() <= 0)
            throw new IllegalArgumentException("'gps_accuracy' must be a positive number of metres, got " + node);
        return node.asDouble();
    }

    private GHRequest readRoutingOptions(ObjectNode options) {
        for (String field : UNSUPPORTED_FIELDS)
            if (options.has(field))
                throw new IllegalArgumentException("Do not send '" + field + "': the waypoints come from the GPX track");
        JsonNode customModel = options.get("custom_model");
        if (customModel == null || customModel.isNull())
            throw new IllegalArgumentException("The 'custom_model' field is required: the planner's custom model, used for both the map matching and the waypoints");
        if (!customModel.isObject())
            throw new IllegalArgumentException("'custom_model' must be a JSON object, got " + customModel.getNodeType());
        GHRequest request;
        try {
            request = objectMapper.treeToValue(options, GHRequest.class);
        } catch (JsonProcessingException e) {
            throw new IllegalArgumentException("Invalid routing options: " + e.getOriginalMessage());
        }
        if (!request.hasSnapPreventions())
            request.setSnapPreventions(snapPreventionsDefault);
        request = ghRequestTransformer.transformRequest(request);
        if (Helper.isEmpty(request.getProfile()))
            throw new IllegalArgumentException("The 'profile' parameter is required when you use the `custom_model` parameter");
        // a request-time custom model needs the flexible mode, as in POST /route
        request.getHints().putObject(Parameters.CH.DISABLE, true);
        PMap profileResolverHints = new PMap(request.getHints());
        profileResolverHints.putObject("profile", request.getProfile());
        request.setProfile(profileResolver.resolveProfile(profileResolverHints));
        removeLegacyParameters(request.getHints());
        return request;
    }

    /** The routing options of the request with the waypoints as its points. */
    static GHRequest withPoints(GHRequest options, List<GHPoint> points) {
        GHRequest request = new GHRequest(points)
                .setProfile(options.getProfile())
                .setCustomModel(options.getCustomModel())
                .setAlgorithm(options.getAlgorithm())
                .setLocale(options.getLocale())
                .setSnapPreventions(options.getSnapPreventions())
                .setPathDetails(options.getPathDetails());
        request.getHints().putAll(options.getHints());
        return request;
    }

    private static ObjectNode diagnostics(MatchWaypoints.Result waypoints, GHResponse route, PointList matchPoints,
                                          StopWatch matchWatch, StopWatch reconstructWatch, StopWatch routeWatch, StopWatch total) {
        ObjectNode diagnostics = Jackson.newObjectMapper().createObjectNode();
        diagnostics.put("route_evaluations", waypoints.getRouteEvaluations());
        diagnostics.put("candidates", waypoints.getCandidates());
        diagnostics.put("skipped_candidates", waypoints.getSkippedCandidates());
        diagnostics.put("waypoints", waypoints.getWaypoints().size());
        diagnostics.put("removed_waypoints", waypoints.getRemovedWaypoints());
        diagnostics.put("legs_within_tolerance", waypoints.getLegsWithinTolerance());
        ArrayNode forced = diagnostics.putArray("forced_waypoints");
        List<MatchWaypoints.Waypoint> list = waypoints.getWaypoints();
        for (int i = 0; i < list.size(); i++) {
            MatchWaypoints.Waypoint waypoint = list.get(i);
            if (!waypoint.isForced())
                continue;
            forced.addObject()
                    .put("index", i)
                    .put("lat", waypoint.getPoint().lat)
                    .put("lon", waypoint.getPoint().lon)
                    .put("reason", waypoint.getForcedReason().name().toLowerCase(Locale.ROOT))
                    .put("message", message(waypoint.getForcedReason()));
        }
        diagnostics.put("forced_count", forced.size());

        PointList routePoints = route.getBest().getPoints();
        ObjectNode deviation = diagnostics.putObject("deviation");
        deviation.put("tolerance_m", DEVIATION_TOLERANCE_METERS);
        double routeLength = RouteDeviation.length(routePoints);
        double matchLength = RouteDeviation.length(matchPoints);
        deviation.put("route_length_m", round(routeLength, 1));
        deviation.put("match_length_m", round(matchLength, 1));
        deviation.put("length_difference_percent", matchLength > 0 ? round(100 * (routeLength - matchLength) / matchLength, 3) : 0);
        if (routePoints.size() >= 2 && matchPoints.size() >= 2) {
            putCoverage(deviation.putObject("route_on_match"), RouteDeviation.coverage(routePoints, matchPoints, DEVIATION_TOLERANCE_METERS, DEVIATION_STEP_METERS));
            putCoverage(deviation.putObject("match_on_route"), RouteDeviation.coverage(matchPoints, routePoints, DEVIATION_TOLERANCE_METERS, DEVIATION_STEP_METERS));
        }

        ObjectNode took = diagnostics.putObject("took_ms");
        took.put("match", Math.round(matchWatch.getMillisDouble()));
        took.put("waypoints", Math.round(reconstructWatch.getMillisDouble()));
        took.put("route", Math.round(routeWatch.getMillisDouble()));
        took.put("total", Math.round(total.getMillisDouble()));
        return diagnostics;
    }

    private static void putCoverage(ObjectNode node, RouteDeviation.Coverage coverage) {
        node.put("within_percent", round(coverage.getWithinPercent(), 3));
        node.put("off_m", round(coverage.getOffMeters(), 1));
        node.put("max_m", round(coverage.getMaxMeters(), 1));
    }

    private static String message(MatchWaypoints.ForcedReason reason) {
        switch (reason) {
            case NO_ROUTE:
                return "No route from the previous waypoint to this one";
            case NOT_ALLOWED:
                return "The match uses a road, or a direction of it, that the profile and custom model do not allow";
            case U_TURN:
                return "The match turns back between the previous waypoint and this one and the router does not";
            case ROUTER_DEVIATES:
                return "The router takes other roads than the match, even to the nearest candidate";
            default:
                throw new IllegalStateException("Unknown reason " + reason);
        }
    }

    private static double round(double value, int decimals) {
        double factor = Math.pow(10, decimals);
        return Math.round(value * factor) / factor;
    }
}
