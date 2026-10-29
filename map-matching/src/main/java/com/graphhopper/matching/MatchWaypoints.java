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

import com.carrotsearch.hppc.IntArrayList;
import com.carrotsearch.hppc.IntIntHashMap;
import com.graphhopper.GHRequest;
import com.graphhopper.GraphHopper;
import com.graphhopper.config.Profile;
import com.graphhopper.routing.AlgorithmOptions;
import com.graphhopper.routing.EdgeRestrictions;
import com.graphhopper.routing.FlexiblePathCalculator;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.RoutingAlgorithmFactorySimple;
import com.graphhopper.routing.ev.BooleanEncodedValue;
import com.graphhopper.routing.ev.RoadClass;
import com.graphhopper.routing.ev.RoadEnvironment;
import com.graphhopper.routing.ev.Subnetwork;
import com.graphhopper.routing.querygraph.QueryGraph;
import com.graphhopper.routing.querygraph.VirtualEdgeIteratorState;
import com.graphhopper.routing.util.DefaultSnapFilter;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.routing.util.SnapPreventionEdgeFilter;
import com.graphhopper.routing.util.TraversalMode;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.storage.NodeAccess;
import com.graphhopper.storage.index.LocationIndexTree;
import com.graphhopper.storage.index.Snap;
import com.graphhopper.util.*;
import com.graphhopper.util.exceptions.MaximumNodesExceededException;
import com.graphhopper.util.shapes.GHPoint;

import java.util.*;

import static com.graphhopper.util.EdgeIterator.NO_EDGE;

/**
 * Turns a GPX track into the fewest waypoints whose route reproduces the track as snapped to the road network.
 * <p>
 * The track is map-matched first. Candidate waypoints are the start, the middle of every matched road stretch of at
 * least {@link #MIN_CANDIDATE_STRETCH_METERS}, every place where the match turns back on itself, and the end. From the
 * current waypoint an exponential and then a binary search finds the farthest candidate whose leg follows the match:
 * along exactly the matched edges (same edges, same directions, twins counted as one), or otherwise within
 * {@link #LEG_TOLERANCE_METERS} of it, see {@link #withinTolerance}. When not even the next candidate can be reached
 * that way, that candidate becomes a forced waypoint. A removal pass then drops every waypoint whose neighbours
 * already route along the match.
 * <p>
 * Match and reconstruction use one weighting: the request's profile merged with its custom model, created the same
 * way {@code /route} creates it. Each leg is evaluated the way {@code /route} computes a leg of a via route with the
 * request's options: the same snapping (including snap preventions), the same flexible path calculation and, with
 * {@code pass_through}, the same penalty on turning back over the edge the previous leg arrived on.
 */
public class MatchWaypoints {

    /**
     * Matched stretches (consecutive edges with the same original edge and direction) shorter than this get no
     * candidate in their middle.
     */
    public static final double MIN_CANDIDATE_STRETCH_METERS = 15;

    /**
     * A leg whose edges differ from the match still counts as following it when every point of the leg lies within
     * this distance of the matched stretch and vice versa (the criterion of the prototype), ...
     */
    public static final double LEG_TOLERANCE_METERS = 15;
    /** ... and its length differs from the matched stretch by at most this many metres or ... */
    public static final double LEG_LENGTH_TOLERANCE_METERS = 20;
    /** ... this fraction of the matched stretch, whichever is larger. */
    public static final double LEG_LENGTH_TOLERANCE_FRACTION = 0.01;

    public enum ForcedReason {
        /** The router finds no path at all from the previous waypoint to this one. */
        NO_ROUTE,
        /**
         * The match uses a road, or a direction of it, that the weighting does not allow between the previous
         * waypoint and this one. The matcher can do this where an observation lies on a one-way road.
         */
        NOT_ALLOWED,
        /** The match turns back on itself between the previous waypoint and this one, and the router does not. */
        U_TURN,
        /** The router takes other roads than the match, even to the nearest candidate. */
        ROUTER_DEVIATES
    }

    public static final class Waypoint {
        private final GHPoint point;
        private final ForcedReason forcedReason;

        Waypoint(GHPoint point, ForcedReason forcedReason) {
            this.point = point;
            this.forcedReason = forcedReason;
        }

        public GHPoint getPoint() {
            return point;
        }

        public boolean isForced() {
            return forcedReason != null;
        }

        /**
         * Why the leg that ends at this waypoint does not follow the match, or null when it does.
         */
        public ForcedReason getForcedReason() {
            return forcedReason;
        }
    }

    /** An out-and-back spur removed from the match. */
    public static final class TrimmedSpur {
        private final GHPoint apex;
        private final double apexDistance;
        private final double length;

        TrimmedSpur(GHPoint apex, double apexDistance, double length) {
            this.apex = apex;
            this.apexDistance = apexDistance;
            this.length = length;
        }

        /** Where the matched path turned back. */
        public GHPoint getApex() {
            return apex;
        }

        /** Distance from the turning point to the GPX, in metres. */
        public double getApexDistance() {
            return apexDistance;
        }

        /** Length of the removed part, out and back, in metres. */
        public double getLength() {
            return length;
        }
    }

    public static final class Result {
        private final List<Waypoint> waypoints;
        private final int routeEvaluations;
        private final int candidates;
        private final int skippedCandidates;
        private final int removedWaypoints;
        private final int legsWithinTolerance;
        private final List<TrimmedSpur> trimmedSpurs;
        private final double matchLengthBeforeTrim;
        private final PointList matchPoints;

        Result(List<Waypoint> waypoints, int routeEvaluations, int candidates, int skippedCandidates, int removedWaypoints,
               int legsWithinTolerance, List<TrimmedSpur> trimmedSpurs, double matchLengthBeforeTrim, PointList matchPoints) {
            this.legsWithinTolerance = legsWithinTolerance;
            this.trimmedSpurs = trimmedSpurs;
            this.matchLengthBeforeTrim = matchLengthBeforeTrim;
            this.matchPoints = matchPoints;
            this.waypoints = waypoints;
            this.routeEvaluations = routeEvaluations;
            this.candidates = candidates;
            this.skippedCandidates = skippedCandidates;
            this.removedWaypoints = removedWaypoints;
        }

        public List<Waypoint> getWaypoints() {
            return waypoints;
        }

        public List<GHPoint> getPoints() {
            List<GHPoint> points = new ArrayList<>(waypoints.size());
            for (Waypoint waypoint : waypoints)
                points.add(waypoint.getPoint());
            return points;
        }

        /** Number of path calculations made to choose and prune the waypoints. */
        public int getRouteEvaluations() {
            return routeEvaluations;
        }

        /** Candidates that could be used, i.e. whose point snaps back onto the matched edge. */
        public int getCandidates() {
            return candidates;
        }

        /** Candidates dropped because their point snaps to another edge (for example under a snap prevention). */
        public int getSkippedCandidates() {
            return skippedCandidates;
        }

        /**
         * Legs of the result that take other edges than the match but stay within the tolerance of it, for example
         * the other carriageway of a dual carriageway.
         */
        public int getLegsWithinTolerance() {
            return legsWithinTolerance;
        }

        /** The out-and-back spurs removed from the match before the waypoint search, see {@link SpurTrimmer}. */
        public List<TrimmedSpur> getTrimmedSpurs() {
            return trimmedSpurs;
        }

        /** Length of the matched path before the spurs were removed, in metres along its geometry. */
        public double getMatchLengthBeforeTrim() {
            return matchLengthBeforeTrim;
        }

        /** The matched path after removing the spurs: the path the waypoints reproduce. */
        public PointList getMatchPoints() {
            return matchPoints;
        }

        /** Waypoints the removal pass dropped after the greedy search. */
        public int getRemovedWaypoints() {
            return removedWaypoints;
        }
    }

    private final GraphHopper hopper;
    private final GHRequest options;
    private final Profile profile;
    private final PMap weightingHints;
    private final Weighting weighting;
    private final boolean passThrough;

    /**
     * @param options the routing options of the request: profile, custom model (both required), hints such as
     *                {@code pass_through} and {@code heading_penalty}, snap preventions and algorithm. Its points are
     *                ignored.
     */
    public MatchWaypoints(GraphHopper hopper, GHRequest options) {
        if (Helper.isEmpty(options.getProfile()))
            throw new IllegalArgumentException("A profile is required");
        if (options.getCustomModel() == null)
            throw new IllegalArgumentException("A custom_model is required: it is the one weighting used for both the match and the waypoints");
        this.hopper = hopper;
        this.options = options;
        this.profile = hopper.getProfile(options.getProfile());
        if (profile == null)
            throw new IllegalArgumentException("The requested profile '" + options.getProfile() + "' does not exist");
        // the limit /route enforces (Router.Solver.checkMaxVisitedNodes), checked before matching and the waypoint
        // search spend CPU under it, not only in the final route
        int maxVisitedNodes = hopper.getRouterConfig().getMaxVisitedNodes();
        if (options.getHints().getInt(Parameters.Routing.MAX_VISITED_NODES, maxVisitedNodes) > maxVisitedNodes)
            throw new IllegalArgumentException("The max_visited_nodes parameter has to be below or equal to:" + maxVisitedNodes);
        this.weightingHints = new PMap(options.getHints());
        weightingHints.putObject(CustomModel.KEY, options.getCustomModel());
        weightingHints.putObject("profile", profile.getName());
        this.weighting = hopper.createWeighting(profile, weightingHints);
        this.passThrough = options.getHints().getBool(Parameters.Routing.PASS_THROUGH, false);
    }

    /**
     * Map-matches the observations with the request's weighting (profile plus custom model).
     */
    public MatchResult match(List<Observation> observations, double gpsAccuracy) {
        PMap matchHints = new PMap(weightingHints);
        // map matching does not use CH; disabling it also makes the matcher use Dijkstra instead of LM
        matchHints.putObject(Parameters.CH.DISABLE, true);
        MapMatching mapMatching = MapMatching.fromGraphHopper(hopper, matchHints);
        mapMatching.setMeasurementErrorSigma(gpsAccuracy);
        return mapMatching.match(observations);
    }

    /**
     * @param gpx the track that was matched, to tell the spurs the matcher added from the ones the track rides
     */
    public Result reconstruct(MatchResult matchResult, PointList gpx) {
        Twins twins = new Twins();
        List<Stretch> stretches = toStretches(matchResult, twins);
        if (stretches.isEmpty())
            throw new IllegalArgumentException("The GPX track matches no road stretch; it needs at least two points that are further apart");
        double lengthBeforeTrim = RouteDeviation.length(new MatchLine(stretches).points);
        List<TrimmedSpur> trimmedSpurs = new ArrayList<>();
        if (gpx.size() >= 2)
            for (SpurTrimmer.TrimmedSpur spur : SpurTrimmer.trim(stretches, new RouteDeviation.LineIndex(gpx)))
                trimmedSpurs.add(new TrimmedSpur(spur.apex, spur.apexDistance, spur.length));
        int[] keys = new int[stretches.size()];
        boolean[] allowed = new boolean[stretches.size()];
        for (int i = 0; i < keys.length; i++) {
            keys[i] = stretches.get(i).edgeKey;
            allowed[i] = stretches.get(i).allowed;
        }

        MatchLine matchLine = new MatchLine(stretches);
        List<Candidate> rawCandidates = createCandidates(matchResult, stretches, matchLine);
        List<Candidate> candidates = new ArrayList<>();
        List<Snap> snaps = new ArrayList<>();
        int skipped = 0;
        for (int i = 0; i < rawCandidates.size(); i++) {
            Candidate candidate = rawCandidates.get(i);
            boolean isEnd = i == 0 || i == rawCandidates.size() - 1;
            Snap snap = snap(candidate.point);
            if (snap == null && isEnd)
                throw new IllegalArgumentException("Cannot snap the " + (i == 0 ? "start" : "end") + " of the match " + candidate.point);
            if (snap == null || (!isEnd && !candidate.isOnItsEdge(snap, twins))) {
                skipped++;
                continue;
            }
            candidate.snap = snap;
            candidates.add(candidate);
            snaps.add(snap);
        }

        QueryGraph queryGraph = QueryGraph.create(hopper.getBaseGraph(), snaps);
        Evaluator evaluator = new Evaluator(queryGraph, candidates, keys, allowed, twins, matchLine);
        List<Integer> greedy = greedy(evaluator, candidates.size());
        List<Integer> kept = removeRedundant(evaluator, greedy);

        List<Waypoint> waypoints = new ArrayList<>(kept.size());
        waypoints.add(new Waypoint(candidates.get(kept.get(0)).point, null));
        int incoming = NO_EDGE;
        int legsWithinTolerance = 0;
        for (int i = 1; i < kept.size(); i++) {
            Evaluation leg = evaluator.evaluate(kept.get(i - 1), kept.get(i), incoming);
            waypoints.add(new Waypoint(candidates.get(kept.get(i)).point, leg.followsMatch ? null : leg.reason));
            if (leg.withinTolerance)
                legsWithinTolerance++;
            incoming = leg.finalEdge;
        }
        return new Result(waypoints, evaluator.evaluations, candidates.size(), skipped, greedy.size() - kept.size(), legsWithinTolerance,
                trimmedSpurs, lengthBeforeTrim, matchLine.points);
    }

    /**
     * The greedy search of the prototype: from the current waypoint grow the step (1, 2, 4, ...) until a candidate
     * fails, then binary search between the last success and the first failure. Returns candidate indices.
     */
    private List<Integer> greedy(Evaluator evaluator, int candidateCount) {
        List<Integer> waypoints = new ArrayList<>();
        waypoints.add(0);
        int last = candidateCount - 1;
        int current = 0;
        int incoming = NO_EDGE;
        while (current < last) {
            int good = current;
            Evaluation goodEvaluation = null;
            int bad = -1;
            int step = 1;
            while (true) {
                int b = Math.min(current + step, last);
                Evaluation evaluation = evaluator.evaluate(current, b, incoming);
                if (evaluation.followsMatch) {
                    good = b;
                    goodEvaluation = evaluation;
                    if (b == last)
                        break;
                    step *= 2;
                } else {
                    bad = b;
                    break;
                }
            }
            if (bad >= 0) {
                int lo = good, hi = bad;
                while (hi - lo > 1) {
                    int mid = (lo + hi) >>> 1;
                    Evaluation evaluation = evaluator.evaluate(current, mid, incoming);
                    if (evaluation.followsMatch) {
                        lo = mid;
                        goodEvaluation = evaluation;
                    } else {
                        hi = mid;
                    }
                }
                good = lo;
            }
            if (good == current) {
                // forced: not even the next candidate is reached along the match
                good = current + 1;
                goodEvaluation = evaluator.evaluate(current, good, incoming);
            }
            waypoints.add(good);
            incoming = goodEvaluation.finalEdge;
            current = good;
        }
        return waypoints;
    }

    /**
     * Drops every waypoint whose predecessor (as kept so far) routes along the match to its successor. The greedy
     * search can miss these because a route can fail at one candidate and succeed further on.
     */
    private List<Integer> removeRedundant(Evaluator evaluator, List<Integer> waypoints) {
        List<Integer> kept = new ArrayList<>();
        kept.add(waypoints.get(0));
        int incoming = NO_EDGE;
        for (int i = 1; i < waypoints.size(); i++) {
            int previous = kept.get(kept.size() - 1);
            if (i < waypoints.size() - 1 && evaluator.evaluate(previous, waypoints.get(i + 1), incoming).followsMatch)
                continue;
            incoming = evaluator.evaluate(previous, waypoints.get(i), incoming).finalEdge;
            kept.add(waypoints.get(i));
        }
        return kept;
    }

    /**
     * Snaps a point the way {@code /route} snaps a waypoint without heading or point hint: with the snap preventions
     * first, then without. Returns null if no edge is found.
     */
    private Snap snap(GHPoint point) {
        LocationIndexTree locationIndex = (LocationIndexTree) hopper.getLocationIndex();
        BooleanEncodedValue inSubnetworkEnc = hopper.getEncodingManager().getBooleanEncodedValue(Subnetwork.key(profile.getName()));
        EdgeFilter snapFilter = new DefaultSnapFilter(weighting, inSubnetworkEnc);
        Snap snap = null;
        List<String> snapPreventions = options.getSnapPreventions();
        if (!snapPreventions.isEmpty()) {
            EdgeFilter strictFilter = new SnapPreventionEdgeFilter(snapFilter,
                    hopper.getEncodingManager().getEnumEncodedValue(RoadClass.KEY, RoadClass.class),
                    hopper.getEncodingManager().getEnumEncodedValue(RoadEnvironment.KEY, RoadEnvironment.class),
                    snapPreventions);
            snap = locationIndex.findClosest(point.lat, point.lon, strictFilter);
        }
        if (snap == null || !snap.isValid())
            snap = locationIndex.findClosest(point.lat, point.lon, snapFilter);
        return snap.isValid() ? snap : null;
    }

    /**
     * Candidates in path order: start, the middle of each long enough stretch, each turning point, end.
     */
    private List<Candidate> createCandidates(MatchResult matchResult, List<Stretch> stretches, MatchLine matchLine) {
        QueryGraph matchGraph = (QueryGraph) matchResult.getGraph();
        NodeAccess nodeAccess = matchGraph.getNodeAccess();
        List<Candidate> candidates = new ArrayList<>();
        int last = stretches.size() - 1;
        PointList first = stretches.get(0).geometry;
        candidates.add(Candidate.atStart(new GHPoint(first.getLat(0), first.getLon(0))));
        for (int i = 0; i <= last; i++) {
            Stretch stretch = stretches.get(i);
            if (i > 0 && stretch.edgeKey == GHUtility.reverseEdgeKey(stretches.get(i - 1).edgeKey)) {
                // the match turns back here; a waypoint on the turning node lets the router turn back too
                int node = stretch.baseNode;
                if (!matchGraph.isVirtualNode(node))
                    candidates.add(Candidate.atTurn(new GHPoint(nodeAccess.getLat(node), nodeAccess.getLon(node)), node, i, matchLine.stretchStart[i]));
            }
            if (stretch.length >= MIN_CANDIDATE_STRETCH_METERS)
                candidates.add(Candidate.onStretch(stretch.middle(), GHUtility.getEdgeFromEdgeKey(stretch.edgeKey), i, matchLine.stretchStart[i] + stretch.length / 2));
        }
        PointList end = stretches.get(last).geometry;
        candidates.add(Candidate.atEnd(new GHPoint(end.getLat(end.size() - 1), end.getLon(end.size() - 1)), last, matchLine.length()));
        return candidates;
    }

    /**
     * Groups the edges of the matched path into stretches of the same original edge (or its twin, see {@link Twins})
     * in the same direction.
     */
    private List<Stretch> toStretches(MatchResult matchResult, Twins twins) {
        QueryGraph matchGraph = (QueryGraph) matchResult.getGraph();
        List<Stretch> stretches = new ArrayList<>();
        Stretch current = null;
        for (EdgeIteratorState edge : matchResult.getMergedPath().calcEdges()) {
            int originalKey = originalEdgeKey(matchGraph, edge);
            int key = twins.canonicalKey(originalKey);
            PointList geometry = edge.fetchWayGeometry(FetchMode.ALL);
            if (current != null && current.edgeKey == key) {
                current.append(geometry);
            } else {
                // the matcher can enter an edge against its direction at an observation, so check the direction here
                boolean allowed = Double.isFinite(weighting.calcEdgeWeight(hopper.getBaseGraph().getEdgeIteratorStateForKey(originalKey), false));
                current = new Stretch(key, edge.getBaseNode(), allowed, geometry);
                stretches.add(current);
            }
        }
        return stretches;
    }

    static int originalEdgeKey(QueryGraph queryGraph, EdgeIteratorState edge) {
        if (queryGraph.isVirtualEdge(edge.getEdge()))
            return ((VirtualEdgeIteratorState) edge).getOriginalEdgeKey();
        return edge.getEdgeKey();
    }

    static final class Stretch {
        final int edgeKey;
        final int baseNode;
        /** Whether the weighting allows the matched edge in the matched direction. */
        final boolean allowed;
        final PointList geometry = new PointList();
        double length;

        Stretch(int edgeKey, int baseNode, boolean allowed, PointList geometry) {
            this.edgeKey = edgeKey;
            this.baseNode = baseNode;
            this.allowed = allowed;
            append(geometry);
        }

        void append(PointList points) {
            // consecutive edges share their junction point
            int from = geometry.isEmpty() ? 0 : 1;
            for (int i = from; i < points.size(); i++) {
                if (!geometry.isEmpty())
                    length += DistanceCalcEarth.DIST_EARTH.calcDist(geometry.getLat(geometry.size() - 1), geometry.getLon(geometry.size() - 1), points.getLat(i), points.getLon(i));
                geometry.add(points.getLat(i), points.getLon(i));
            }
        }

        /** The point halfway along the stretch. */
        GHPoint middle() {
            return MatchWaypoints.middle(geometry);
        }
    }

    /** The point halfway along a line. */
    static GHPoint middle(PointList geometry) {
        double length = 0;
        for (int i = 1; i < geometry.size(); i++)
            length += DistanceCalcEarth.DIST_EARTH.calcDist(geometry.getLat(i - 1), geometry.getLon(i - 1), geometry.getLat(i), geometry.getLon(i));
        double remaining = length / 2;
        for (int i = 1; i < geometry.size(); i++) {
            double lat0 = geometry.getLat(i - 1), lon0 = geometry.getLon(i - 1);
            double lat1 = geometry.getLat(i), lon1 = geometry.getLon(i);
            double d = DistanceCalcEarth.DIST_EARTH.calcDist(lat0, lon0, lat1, lon1);
            if (d >= remaining && d > 0) {
                double f = remaining / d;
                return new GHPoint(lat0 + f * (lat1 - lat0), lon0 + f * (lon1 - lon0));
            }
            remaining -= d;
        }
        return new GHPoint(geometry.getLat(geometry.size() - 1), geometry.getLon(geometry.size() - 1));
    }

    static final class Candidate {
        final GHPoint point;
        /** The stretch a leg leaving this candidate starts on. */
        final int firstStretch;
        /** The stretch a leg arriving at this candidate ends on. */
        final int lastStretch;
        /** The original edge the point must snap to, or -1. */
        final int edge;
        /** The tower node the point must snap to, or -1. */
        final int towerNode;
        /** Distance along the matched path, in metres. */
        final double along;
        Snap snap;

        private Candidate(GHPoint point, int firstStretch, int lastStretch, int edge, int towerNode, double along) {
            this.point = point;
            this.along = along;
            this.firstStretch = firstStretch;
            this.lastStretch = lastStretch;
            this.edge = edge;
            this.towerNode = towerNode;
        }

        static Candidate atStart(GHPoint point) {
            return new Candidate(point, 0, 0, -1, -1, 0);
        }

        static Candidate atEnd(GHPoint point, int lastStretch, double along) {
            return new Candidate(point, lastStretch, lastStretch, -1, -1, along);
        }

        static Candidate onStretch(GHPoint point, int edge, int stretch, double along) {
            return new Candidate(point, stretch, stretch, edge, -1, along);
        }

        static Candidate atTurn(GHPoint point, int towerNode, int stretchAfterTurn, double along) {
            return new Candidate(point, stretchAfterTurn, stretchAfterTurn - 1, -1, towerNode, along);
        }

        /**
         * Whether the snap of this candidate's point lands where the candidate was taken from. If not (another edge
         * is as close, or a snap prevention moved it) a waypoint there would not mean what the candidate means.
         */
        boolean isOnItsEdge(Snap snap, Twins twins) {
            if (towerNode >= 0)
                return snap.getSnappedPosition() == Snap.Position.TOWER && snap.getClosestNode() == towerNode;
            return snap.getSnappedPosition() != Snap.Position.TOWER
                    && twins.canonicalEdge(snap.getClosestEdge().getEdge()) == twins.canonicalEdge(edge);
        }
    }

    static final class Evaluation {
        final boolean followsMatch;
        /** The leg follows the match within the tolerance but not along exactly its edges. */
        final boolean withinTolerance;
        final ForcedReason reason;
        /** The last edge of the leg, which pass_through keeps the next leg from turning back over; NO_EDGE if none. */
        final int finalEdge;

        Evaluation(boolean followsMatch, ForcedReason reason, int finalEdge) {
            this(followsMatch, false, reason, finalEdge);
        }

        Evaluation(boolean followsMatch, boolean withinTolerance, ForcedReason reason, int finalEdge) {
            this.followsMatch = followsMatch;
            this.withinTolerance = withinTolerance;
            this.reason = reason;
            this.finalEdge = finalEdge;
        }
    }

    /**
     * Calculates legs between candidates on one query graph that holds every candidate, the way {@code /route} does
     * for one leg of a via route, and compares the leg's edges with the match. Results are cached.
     */
    private final class Evaluator {
        private final QueryGraph queryGraph;
        private final List<Candidate> candidates;
        private final int[] matchKeys;
        private final boolean[] matchAllowed;
        private final FlexiblePathCalculator pathCalculator;
        private final Twins twins;
        private final MatchLine matchLine;
        private final Map<Integer, Map<Long, Evaluation>> cacheByIncoming = new HashMap<>();
        int evaluations;

        Evaluator(QueryGraph queryGraph, List<Candidate> candidates, int[] matchKeys, boolean[] matchAllowed, Twins twins, MatchLine matchLine) {
            this.matchLine = matchLine;
            this.matchAllowed = matchAllowed;
            this.queryGraph = queryGraph;
            this.twins = twins;
            this.candidates = candidates;
            this.matchKeys = matchKeys;
            PMap hints = options.getHints();
            AlgorithmOptions algoOpts = new AlgorithmOptions().
                    setAlgorithm(options.getAlgorithm()).
                    setTraversalMode(profile.hasTurnCosts() ? TraversalMode.EDGE_BASED : TraversalMode.NODE_BASED).
                    setMaxVisitedNodes(hints.getInt(Parameters.Routing.MAX_VISITED_NODES, hopper.getRouterConfig().getMaxVisitedNodes())).
                    setTimeoutMillis(Math.min(hopper.getRouterConfig().getTimeoutMillis(), hints.getLong(Parameters.Routing.TIMEOUT_MS, hopper.getRouterConfig().getTimeoutMillis()))).
                    setHints(hints);
            this.pathCalculator = new FlexiblePathCalculator(queryGraph, new RoutingAlgorithmFactorySimple(), weighting, algoOpts);
        }

        Evaluation evaluate(int from, int to, int incomingEdge) {
            int unfavored = passThrough ? incomingEdge : NO_EDGE;
            Map<Long, Evaluation> byLeg = cacheByIncoming.computeIfAbsent(unfavored, k -> new HashMap<>());
            long leg = ((long) from << 32) | to;
            Evaluation cached = byLeg.get(leg);
            if (cached != null)
                return cached;
            Evaluation evaluation = calculate(candidates.get(from), candidates.get(to), unfavored);
            byLeg.put(leg, evaluation);
            return evaluation;
        }

        private Evaluation calculate(Candidate from, Candidate to, int unfavoredEdge) {
            evaluations++;
            int[] expected = Arrays.copyOfRange(matchKeys, from.firstStretch, to.lastStretch + 1);
            EdgeRestrictions restrictions = new EdgeRestrictions();
            if (unfavoredEdge != NO_EDGE)
                restrictions.getUnfavoredEdges().add(unfavoredEdge);
            Path path;
            try {
                path = pathCalculator.calcPaths(from.snap.getClosestNode(), to.snap.getClosestNode(), restrictions).get(0);
            } catch (MaximumNodesExceededException e) {
                return new Evaluation(false, ForcedReason.NO_ROUTE, NO_EDGE);
            }
            if (!path.isFound())
                return new Evaluation(false, ForcedReason.NO_ROUTE, NO_EDGE);
            int finalEdge = path.getEdgeCount() > 0 ? path.getFinalEdge().getEdge() : NO_EDGE;
            int[] actual = collapse(path.calcEdges());
            if (Arrays.equals(expected, actual))
                return new Evaluation(true, null, finalEdge);
            if (withinTolerance(path.calcPoints(), matchLine.between(from.along, to.along)))
                return new Evaluation(true, true, null, finalEdge);
            return new Evaluation(false, reason(from, to), finalEdge);
        }

        private ForcedReason reason(Candidate from, Candidate to) {
            for (int i = from.firstStretch; i <= to.lastStretch; i++)
                if (!matchAllowed[i])
                    return ForcedReason.NOT_ALLOWED;
            // include the stretch before the leg: a leg that starts where the match turns back cannot turn back
            // either when the previous leg arrived over a virtual edge, which pass_through then penalizes
            int first = Math.max(0, from.firstStretch - 1);
            return turnsBack(Arrays.copyOfRange(matchKeys, first, to.lastStretch + 1)) ? ForcedReason.U_TURN : ForcedReason.ROUTER_DEVIATES;
        }

        private int[] collapse(List<EdgeIteratorState> edges) {
            IntArrayList keys = new IntArrayList(edges.size());
            for (EdgeIteratorState edge : edges) {
                int key = twins.canonicalKey(originalEdgeKey(queryGraph, edge));
                if (keys.isEmpty() || keys.get(keys.size() - 1) != key)
                    keys.add(key);
            }
            return keys.toArray();
        }
    }

    private static boolean turnsBack(int[] keys) {
        for (int i = 1; i < keys.length; i++)
            if (keys[i] == GHUtility.reverseEdgeKey(keys[i - 1]))
                return true;
        return false;
    }

    /**
     * Edges that connect the same two nodes along (almost) the same geometry, such as a street that is mapped twice
     * on the same nodes. The matcher can put the match on either of them while the router takes the cheaper one; both
     * give the same route, so legs are compared by a canonical key: the smallest edge key among an edge and its twins
     * in the same direction.
     */
    private final class Twins {
        private static final double MAX_LENGTH_DIFFERENCE_METERS = 1;
        private static final double MAX_MIDDLE_DISTANCE_METERS = 5;
        private final BaseGraph graph = hopper.getBaseGraph();
        private final EdgeExplorer explorer = graph.createEdgeExplorer();
        private final IntIntHashMap canonicalKeys = new IntIntHashMap();

        int canonicalKey(int edgeKey) {
            int cached = canonicalKeys.getOrDefault(edgeKey, -1);
            if (cached >= 0)
                return cached;
            EdgeIteratorState edge = graph.getEdgeIteratorStateForKey(edgeKey);
            int canonical = edgeKey;
            if (edge.getBaseNode() != edge.getAdjNode()) {
                EdgeIterator iter = explorer.setBaseNode(edge.getBaseNode());
                while (iter.next()) {
                    if (iter.getEdge() != edge.getEdge() && iter.getAdjNode() == edge.getAdjNode() && sameGeometry(edge, iter))
                        canonical = Math.min(canonical, iter.getEdgeKey());
                }
            }
            canonicalKeys.put(edgeKey, canonical);
            return canonical;
        }

        int canonicalEdge(int edge) {
            return GHUtility.getEdgeFromEdgeKey(canonicalKey(GHUtility.createEdgeKey(edge, false)));
        }

        private boolean sameGeometry(EdgeIteratorState a, EdgeIteratorState b) {
            if (Math.abs(a.getDistance() - b.getDistance()) > MAX_LENGTH_DIFFERENCE_METERS)
                return false;
            GHPoint middleA = middle(a.fetchWayGeometry(FetchMode.ALL));
            GHPoint middleB = middle(b.fetchWayGeometry(FetchMode.ALL));
            return DistanceCalcEarth.DIST_EARTH.calcDist(middleA.lat, middleA.lon, middleB.lat, middleB.lon) <= MAX_MIDDLE_DISTANCE_METERS;
        }
    }

    /**
     * Whether a leg follows the matched stretch geometrically: lengths within the length tolerance and every point of
     * each line (densified to 5 m) within {@link #LEG_TOLERANCE_METERS} of the other.
     */
    public static boolean withinTolerance(PointList leg, PointList matched) {
        if (leg.size() < 2 || matched.size() < 2)
            return false;
        double legLength = RouteDeviation.length(leg), matchedLength = RouteDeviation.length(matched);
        if (Math.abs(legLength - matchedLength) > Math.max(LEG_LENGTH_TOLERANCE_METERS, LEG_LENGTH_TOLERANCE_FRACTION * matchedLength))
            return false;
        return RouteDeviation.coverage(leg, matched, LEG_TOLERANCE_METERS, 5).getMaxMeters() <= LEG_TOLERANCE_METERS
                && RouteDeviation.coverage(matched, leg, LEG_TOLERANCE_METERS, 5).getMaxMeters() <= LEG_TOLERANCE_METERS;
    }

    /** The matched path as one line, to cut out the stretch between two candidates. */
    static final class MatchLine {
        final PointList points = new PointList();
        /** Distance along the line at each point. */
        private double[] along = new double[16];
        /** Distance along the line where each stretch starts. */
        final double[] stretchStart;

        MatchLine(List<Stretch> stretches) {
            stretchStart = new double[stretches.size()];
            for (int s = 0; s < stretches.size(); s++) {
                PointList geometry = stretches.get(s).geometry;
                stretchStart[s] = points.isEmpty() ? 0 : along[points.size() - 1];
                // consecutive stretches share their junction point
                for (int i = points.isEmpty() ? 0 : 1; i < geometry.size(); i++)
                    add(geometry.getLat(i), geometry.getLon(i));
            }
        }

        private void add(double lat, double lon) {
            int n = points.size();
            double d = n == 0 ? 0 : along[n - 1] + DistanceCalcEarth.DIST_EARTH.calcDist(points.getLat(n - 1), points.getLon(n - 1), lat, lon);
            if (n == along.length)
                along = Arrays.copyOf(along, n * 2);
            along[n] = d;
            points.add(lat, lon);
        }

        double length() {
            return points.isEmpty() ? 0 : along[points.size() - 1];
        }

        /** The part of the line from one distance along it to another. */
        PointList between(double from, double to) {
            PointList result = new PointList();
            int n = points.size();
            int i = 1;
            while (i < n - 1 && along[i] < from)
                i++;
            addAt(result, from, i);
            for (; i < n && along[i] < to; i++)
                if (along[i] > from)
                    result.add(points.getLat(i), points.getLon(i));
            addAt(result, to, Math.min(Math.max(i, 1), n - 1));
            return result;
        }

        /** Adds the point at distance d, which lies on segment i-1 -> i. */
        private void addAt(PointList result, double d, int i) {
            double segment = along[i] - along[i - 1];
            double f = segment <= 0 ? 0 : Math.max(0, Math.min(1, (d - along[i - 1]) / segment));
            result.add(points.getLat(i - 1) + f * (points.getLat(i) - points.getLat(i - 1)),
                    points.getLon(i - 1) + f * (points.getLon(i) - points.getLon(i - 1)));
        }
    }
}
