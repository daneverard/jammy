// Traffic Simulator - Main Application
// MVP Implementation

// ============================================================================
// Configuration & Constants
// ============================================================================

const CONFIG = {
    // Simulation
    timestep: 0.016, // ~60 FPS
    minTimestep: 0.001,
    maxTimestep: 0.1,
    
    // Car Dynamics (from spec)
    desiredHeadway: 1.5, // seconds
    minGap: 2.0, // meters
    accelLimit: 1.6, // m/s?
    decelLimit: 2.5, // m/s?
    emergencyDecel: 6.0, // m/s?
    carLength: 4.3, // meters
    reactionTimeMean: 0.9, // seconds
    reactionTimeStd: 0.25,
    reactionTimeMin: 0.4,
    reactionTimeMax: 1.8,
    perceptionNoise: 0.05, // ?5%
    topSpeedMean: 50, // km/h
    topSpeedStd: 8,
    topSpeedMin: 30,
    topSpeedMax: 80,
    
    // Network
    snapRadius: 10, // pixels at 1? zoom
    gridSize: 20, // pixels
    
    // Visual
    roadWidth: 6,
    junctionRadius: 8,
    
    // Performance
    maxCars: 500,
    performanceModeThreshold: 30, // FPS
    
    // History
    maxHistory: 50,
};

// ============================================================================
// Utility Functions
// ============================================================================

function kmhToMs(kmh) {
    return kmh / 3.6;
}

function msToKmh(ms) {
    return ms * 3.6;
}

function randomNormal(mean, std, min, max) {
    // Box-Muller transform for normal distribution
    let u = 0, v = 0;
    while (u === 0) u = Math.random();
    while (v === 0) v = Math.random();
    let z = Math.sqrt(-2.0 * Math.log(u)) * Math.cos(2.0 * Math.PI * v);
    let value = mean + std * z;
    return Math.max(min, Math.min(max, value));
}

function distance(p1, p2) {
    const dx = p2.x - p1.x;
    const dy = p2.y - p1.y;
    return Math.sqrt(dx * dx + dy * dy);
}

function pointOnLine(p, lineStart, lineEnd) {
    const A = p.x - lineStart.x;
    const B = p.y - lineStart.y;
    const C = lineEnd.x - lineStart.x;
    const D = lineEnd.y - lineStart.y;
    
    const dot = A * C + B * D;
    const lenSq = C * C + D * D;
    if (lenSq === 0) return 0;
    
    let param = dot / lenSq;
    if (param < 0) param = 0;
    if (param > 1) param = 1;
    
    return param;
}

function distToLineSegment(p, lineStart, lineEnd) {
    const A = p.x - lineStart.x;
    const B = p.y - lineStart.y;
    const C = lineEnd.x - lineStart.x;
    const D = lineEnd.y - lineStart.y;
    
    const dot = A * C + B * D;
    const lenSq = C * C + D * D;
    if (lenSq === 0) return distance(p, lineStart);
    
    let param = dot / lenSq;
    if (param < 0) param = 0;
    if (param > 1) param = 1;
    
    const xx = lineStart.x + param * C;
    const yy = lineStart.y + param * D;
    
    return distance(p, { x: xx, y: yy });
}

// ============================================================================
// Road Network Data Structures
// ============================================================================

class RoadSegment {
    constructor(id, points, twoWay = true, speedLimit = null) {
        this.id = id;
        this.points = points; // Array of {x, y}
        this.twoWay = twoWay;
        this.speedLimit = speedLimit; // km/h, null means use global
        this.selected = false;
        this.cars = new Set(); // Cars currently on this segment
        
        this.updateLength();
    }
    
    updateLength() {
        this.length = 0;
        for (let i = 1; i < this.points.length; i++) {
            this.length += distance(this.points[i - 1], this.points[i]);
        }
    }
    
    getStartPoint() {
        return this.points[0];
    }
    
    getEndPoint() {
        return this.points[this.points.length - 1];
    }
    
    getPointAt(t) {
        // t: 0 to 1 along the polyline
        if (t <= 0) return this.points[0];
        if (t >= 1) return this.points[this.points.length - 1];
        
        let targetDist = t * this.length;
        let currentDist = 0;
        
        for (let i = 1; i < this.points.length; i++) {
            const segLength = distance(this.points[i - 1], this.points[i]);
            if (currentDist + segLength >= targetDist) {
                const localT = (targetDist - currentDist) / segLength;
                return {
                    x: this.points[i - 1].x + (this.points[i].x - this.points[i - 1].x) * localT,
                    y: this.points[i - 1].y + (this.points[i].y - this.points[i - 1].y) * localT
                };
            }
            currentDist += segLength;
        }
        
        return this.getEndPoint();
    }
    
    getDirectionAt(t) {
        // Returns normalized direction vector at t
        if (t <= 0) {
            const dx = this.points[1].x - this.points[0].x;
            const dy = this.points[1].y - this.points[0].y;
            const len = Math.sqrt(dx * dx + dy * dy);
            return len > 0 ? { x: dx / len, y: dy / len } : { x: 1, y: 0 };
        }
        if (t >= 1) {
            const n = this.points.length;
            const dx = this.points[n - 1].x - this.points[n - 2].x;
            const dy = this.points[n - 1].y - this.points[n - 2].y;
            const len = Math.sqrt(dx * dx + dy * dy);
            return len > 0 ? { x: dx / len, y: dy / len } : { x: 1, y: 0 };
        }
        
        let targetDist = t * this.length;
        let currentDist = 0;
        
        for (let i = 1; i < this.points.length; i++) {
            const segLength = distance(this.points[i - 1], this.points[i]);
            if (currentDist + segLength >= targetDist) {
                const dx = this.points[i].x - this.points[i - 1].x;
                const dy = this.points[i].y - this.points[i - 1].y;
                const len = Math.sqrt(dx * dx + dy * dy);
                return len > 0 ? { x: dx / len, y: dy / len } : { x: 1, y: 0 };
            }
            currentDist += segLength;
        }
        
        return { x: 1, y: 0 };
    }
    
    getNearestPoint(worldPos) {
        let minDist = Infinity;
        let bestT = 0;
        
        let currentDist = 0;
        for (let i = 1; i < this.points.length; i++) {
            const segStart = this.points[i - 1];
            const segEnd = this.points[i];
            const segLength = distance(segStart, segEnd);
            
            const t = pointOnLine(worldPos, segStart, segEnd);
            const point = {
                x: segStart.x + (segEnd.x - segStart.x) * t,
                y: segStart.y + (segEnd.y - segStart.y) * t
            };
            const dist = distance(worldPos, point);
            
            if (dist < minDist) {
                minDist = dist;
                const segT = currentDist / this.length;
                bestT = segT + (t * segLength) / this.length;
            }
            
            currentDist += segLength;
        }
        
        return { t: bestT, dist: minDist };
    }
    
    toJSON() {
        return {
            id: this.id,
            points: this.points,
            twoWay: this.twoWay,
            speedLimit: this.speedLimit
        };
    }
    
    static fromJSON(data) {
        return new RoadSegment(data.id, data.points, data.twoWay, data.speedLimit);
    }
}

class Junction {
    constructor(id, position) {
        this.id = id;
        this.position = position; // {x, y}
        this.connectedSegments = new Set();
        this.degree = 0;
    }
    
    addSegment(segment) {
        this.connectedSegments.add(segment);
        this.updateDegree();
    }
    
    removeSegment(segment) {
        this.connectedSegments.delete(segment);
        this.updateDegree();
    }
    
    updateDegree() {
        this.degree = 0;
        for (const seg of this.connectedSegments) {
            if (seg.twoWay) {
                this.degree += 2;
            } else {
                this.degree += 1;
            }
        }
    }
    
    getOutgoingSegments(fromSegment, direction) {
        // Returns valid outgoing segments from a given segment and direction
        const outgoing = [];
        for (const seg of this.connectedSegments) {
            if (seg === fromSegment) continue;
            
            const segStart = seg.getStartPoint();
            const segEnd = seg.getEndPoint();
            const atStart = distance(this.position, segStart) < CONFIG.snapRadius;
            const atEnd = distance(this.position, segEnd) < CONFIG.snapRadius;
            
            if (seg.twoWay) {
                // Can go either direction
                outgoing.push({ segment: seg, reverse: false });
                outgoing.push({ segment: seg, reverse: true });
            } else if (atStart) {
                // Can go forward
                outgoing.push({ segment: seg, reverse: false });
            } else if (atEnd) {
                // Can go backward (reverse direction)
                outgoing.push({ segment: seg, reverse: true });
            }
        }
        return outgoing;
    }
}

class RoadNetwork {
    constructor() {
        this.segments = new Map();
        this.junctions = new Map();
        this.nextSegmentId = 1;
        this.nextJunctionId = 1;
        this.graph = new Map(); // For routing
    }
    
    addSegment(points, twoWay = true, speedLimit = null) {
        if (points.length < 2) return null;
        
        // Check for zero-length segments
        let totalLength = 0;
        for (let i = 1; i < points.length; i++) {
            totalLength += distance(points[i - 1], points[i]);
        }
        if (totalLength < 1) return null;
        
        const segment = new RoadSegment(this.nextSegmentId++, points, twoWay, speedLimit);
        this.segments.set(segment.id, segment);
        
        this.updateJunctions();
        return segment;
    }
    
    removeSegment(segmentId) {
        const segment = this.segments.get(segmentId);
        if (!segment) return false;
        
        // Remove from junctions
        for (const junction of this.junctions.values()) {
            junction.removeSegment(segment);
            if (junction.connectedSegments.size === 0) {
                this.junctions.delete(junction.id);
            }
        }
        
        this.segments.delete(segmentId);
        this.updateJunctions();
        return true;
    }
    
    updateJunctions() {
        // Clear old junctions
        this.junctions.clear();
        
        // Find all segment endpoints
        const endpointMap = new Map(); // position string -> junction
        
        const getPosKey = (p) => `${Math.round(p.x)},${Math.round(p.y)}`;
        
        for (const segment of this.segments.values()) {
            const start = segment.getStartPoint();
            const end = segment.getEndPoint();
            
            // Check start point
            const startKey = getPosKey(start);
            let junction = endpointMap.get(startKey);
            if (!junction) {
                junction = new Junction(this.nextJunctionId++, start);
                endpointMap.set(startKey, junction);
                this.junctions.set(junction.id, junction);
            }
            junction.addSegment(segment);
            
            // Check end point
            const endKey = getPosKey(end);
            if (startKey !== endKey) {
                junction = endpointMap.get(endKey);
                if (!junction) {
                    junction = new Junction(this.nextJunctionId++, end);
                    endpointMap.set(endKey, junction);
                    this.junctions.set(junction.id, junction);
                }
                junction.addSegment(segment);
            }
            
            // Check for intersections (simplified: only check if endpoints are close)
            // For MVP, we'll just use endpoint snapping
        }
        
        // Snap nearby endpoints
        this.snapNearbyEndpoints();
    }
    
    snapNearbyEndpoints() {
        const snapRadius = CONFIG.snapRadius;
        const junctions = Array.from(this.junctions.values());
        
        for (let i = 0; i < junctions.length; i++) {
            for (let j = i + 1; j < junctions.length; j++) {
                const j1 = junctions[i];
                const j2 = junctions[j];
                const dist = distance(j1.position, j2.position);
                
                if (dist < snapRadius) {
                    // Merge j2 into j1
                    const avgPos = {
                        x: (j1.position.x + j2.position.x) / 2,
                        y: (j1.position.y + j2.position.y) / 2
                    };
                    
                    // Update all segments connected to j2
                    for (const seg of j2.connectedSegments) {
                        j1.addSegment(seg);
                        // Update segment endpoints to snap to merged position
                        const start = seg.getStartPoint();
                        const end = seg.getEndPoint();
                        if (distance(start, j2.position) < snapRadius) {
                            seg.points[0] = { ...avgPos };
                        }
                        if (distance(end, j2.position) < snapRadius) {
                            seg.points[seg.points.length - 1] = { ...avgPos };
                        }
                    }
                    
                    this.junctions.delete(j2.id);
                }
            }
        }
        
        // Update junction positions to match segment endpoints
        for (const junction of this.junctions.values()) {
            let totalX = 0, totalY = 0, count = 0;
            for (const seg of junction.connectedSegments) {
                const start = seg.getStartPoint();
                const end = seg.getEndPoint();
                if (distance(start, junction.position) < snapRadius) {
                    totalX += start.x;
                    totalY += start.y;
                    count++;
                }
                if (distance(end, junction.position) < snapRadius && distance(start, end) > snapRadius) {
                    totalX += end.x;
                    totalY += end.y;
                    count++;
                }
            }
            if (count > 0) {
                junction.position = { x: totalX / count, y: totalY / count };
            }
        }
    }
    
    findNearestSegment(worldPos, maxDist = 50) {
        let nearest = null;
        let minDist = maxDist;
        
        for (const segment of this.segments.values()) {
            const { dist } = segment.getNearestPoint(worldPos);
            if (dist < minDist) {
                minDist = dist;
                nearest = segment;
            }
        }
        
        return nearest ? { segment: nearest, dist: minDist } : null;
    }
    
    getRandomSpawnPoint() {
        const segments = Array.from(this.segments.values());
        if (segments.length === 0) return null;
        
        const segment = segments[Math.floor(Math.random() * segments.length)];
        const t = Math.random() * 0.8 + 0.1; // Avoid endpoints
        const point = segment.getPointAt(t);
        const direction = segment.getDirectionAt(t);
        
        return {
            segment: segment,
            t: t,
            position: point,
            direction: direction,
            reverse: false
        };
    }
    
    toJSON() {
        return {
            segments: Array.from(this.segments.values()).map(s => s.toJSON()),
            version: "1.0"
        };
    }
    
    static fromJSON(data) {
        const network = new RoadNetwork();
        for (const segData of data.segments || []) {
            const segment = RoadSegment.fromJSON(segData);
            network.segments.set(segment.id, segment);
            network.nextSegmentId = Math.max(network.nextSegmentId, segment.id + 1);
        }
        network.updateJunctions();
        return network;
    }
}

// ============================================================================
// Car Agent
// ============================================================================

class CarAgent {
    constructor(id, spawnPoint, network) {
        this.id = id;
        this.network = network;
        this.segment = spawnPoint.segment;
        this.t = spawnPoint.t; // Position along segment (0-1)
        this.reverse = spawnPoint.reverse || false;
        this.position = { ...spawnPoint.position };
        this.direction = spawnPoint.reverse ? 
            { x: -spawnPoint.direction.x, y: -spawnPoint.direction.y } : 
            { ...spawnPoint.direction };
        
        // Dynamics
        this.speed = 0; // m/s
        this.targetSpeed = 0;
        this.acceleration = 0;
        
        // Individual parameters (variability)
        this.topSpeed = kmhToMs(randomNormal(
            CONFIG.topSpeedMean, CONFIG.topSpeedStd,
            CONFIG.topSpeedMin, CONFIG.topSpeedMax
        ));
        this.reactionTime = randomNormal(
            CONFIG.reactionTimeMean, CONFIG.reactionTimeStd,
            CONFIG.reactionTimeMin, CONFIG.reactionTimeMax
        );
        this.reactionTimer = 0;
        this.pendingAcceleration = 0;
        
        // Following behavior
        this.leader = null;
        this.headway = CONFIG.desiredHeadway;
        this.minGap = CONFIG.minGap;
        
        // Navigation
        this.currentSegment = this.segment;
        this.path = []; // Future route
        this.turnaroundPending = false;
        
        // Visual
        this.selected = false;
    }
    
    update(dt, allCars) {
        if (this.turnaroundPending) {
            this.performTurnaround();
            return;
        }
        
        // Update position along segment
        const segment = this.currentSegment;
        if (!segment) return;
        
        const segmentSpeedLimit = segment.speedLimit ? 
            kmhToMs(segment.speedLimit) : 
            this.topSpeed;
        const effectiveTopSpeed = Math.min(this.topSpeed, segmentSpeedLimit);
        
        // Find leader
        this.findLeader(allCars);
        
        // Calculate desired speed
        if (this.leader) {
            this.calculateFollowingSpeed(dt);
        } else {
            this.targetSpeed = effectiveTopSpeed;
        }
        
        // Apply reaction time
        if (this.reactionTimer > 0) {
            this.reactionTimer -= dt;
        } else {
            // Apply pending acceleration
            this.acceleration = this.pendingAcceleration;
            this.pendingAcceleration = 0;
        }
        
        // Update speed with acceleration limits
        const speedDiff = this.targetSpeed - this.speed;
        const maxAccel = speedDiff > 0 ? CONFIG.accelLimit : 
            (Math.abs(speedDiff) > 5 ? CONFIG.emergencyDecel : CONFIG.decelLimit);
        
        const desiredAccel = Math.sign(speedDiff) * Math.min(Math.abs(speedDiff / dt), maxAccel);
        
        if (this.reactionTimer <= 0) {
            this.acceleration = desiredAccel;
        } else {
            this.pendingAcceleration = desiredAccel;
        }
        
        this.speed = Math.max(0, this.speed + this.acceleration * dt);
        this.speed = Math.min(this.speed, effectiveTopSpeed);
        
        // Move along segment
        const segmentLength = segment.length;
        const distanceToMove = this.speed * dt;
        const tIncrement = distanceToMove / segmentLength;
        
        if (!this.reverse) {
            this.t += tIncrement;
            if (this.t >= 1.0) {
                this.reachJunction();
            }
        } else {
            this.t -= tIncrement;
            if (this.t <= 0.0) {
                this.reachJunction();
            }
        }
        
        // Clamp t
        this.t = Math.max(0, Math.min(1, this.t));
        
        // Update position and direction
        this.position = segment.getPointAt(this.t);
        const dir = segment.getDirectionAt(this.t);
        this.direction = this.reverse ? 
            { x: -dir.x, y: -dir.y } : 
            { x: dir.x, y: dir.y };
    }
    
    findLeader(allCars) {
        this.leader = null;
        const segment = this.currentSegment;
        if (!segment) return;
        
        let minDistance = Infinity;
        let minCar = null;
        
        for (const car of allCars) {
            if (car === this || car.currentSegment !== segment) continue;
            
            // Check if on same direction
            if (car.reverse !== this.reverse) {
                if (!segment.twoWay) continue;
                // For two-way, check if ahead
                if (this.reverse) {
                    if (car.t > this.t) continue; // Behind us
                } else {
                    if (car.t < this.t) continue; // Behind us
                }
            } else {
                // Same direction
                let dist;
                if (this.reverse) {
                    if (car.t > this.t) continue; // Behind us
                    dist = (car.t - this.t) * segment.length;
                } else {
                    if (car.t < this.t) continue; // Behind us
                    dist = (car.t - this.t) * segment.length;
                }
                
                if (dist < minDistance) {
                    minDistance = dist;
                    minCar = car;
                }
            }
        }
        
        if (minCar && minDistance < segment.length * 0.9) {
            this.leader = minCar;
        }
    }
    
    calculateFollowingSpeed(dt) {
        if (!this.leader) {
            this.targetSpeed = this.topSpeed;
            return;
        }
        
        const segment = this.currentSegment;
        const leaderDist = Math.abs(this.leader.t - this.t) * segment.length;
        
        // Apply perception noise
        const noiseFactor = 1 + (Math.random() * 2 - 1) * CONFIG.perceptionNoise;
        const perceivedDist = leaderDist * noiseFactor;
        
        // Safe following distance
        const safeDist = this.minGap + this.speed * this.headway;
        
        // Leader speed (with noise)
        const leaderSpeed = this.leader.speed * (1 + (Math.random() * 2 - 1) * CONFIG.perceptionNoise * 0.1);
        
        if (perceivedDist < safeDist * 1.2) {
            // Too close - match or slow down
            this.targetSpeed = Math.min(leaderSpeed, this.speed);
            
            // Emergency braking if very close
            if (perceivedDist < safeDist * 0.5) {
                this.targetSpeed = Math.max(0, leaderSpeed - 2);
            }
        } else {
            // Room to accelerate, but don't exceed leader much
            this.targetSpeed = Math.min(this.topSpeed, leaderSpeed + 2);
        }
        
        // Schedule reaction
        if (Math.abs(this.targetSpeed - this.speed) > 0.1) {
            this.reactionTimer = this.reactionTime;
        }
    }
    
    reachJunction() {
        // Find junction at segment endpoint
        const segment = this.currentSegment;
        const endpoint = this.reverse ? segment.getStartPoint() : segment.getEndPoint();
        
        // Find junction
        let junction = null;
        for (const j of this.network.junctions.values()) {
            if (distance(j.position, endpoint) < CONFIG.snapRadius) {
                junction = j;
                break;
            }
        }
        
        if (!junction) {
            // Dead end - turn around
            this.turnaroundPending = true;
            return;
        }
        
        // Get outgoing segments
        const outgoing = junction.getOutgoingSegments(segment, this.reverse);
        
        if (outgoing.length === 0) {
            // Dead end
            this.turnaroundPending = true;
            return;
        }
        
        // Random choice
        const choice = outgoing[Math.floor(Math.random() * outgoing.length)];
        this.currentSegment = choice.segment;
        this.reverse = choice.reverse;
        this.t = this.reverse ? 0.99 : 0.01;
        
        // Update segment reference
        segment.cars.delete(this);
        this.currentSegment.cars.add(this);
    }
    
    performTurnaround() {
        // Simple turnaround: reverse direction on same segment
        if (this.speed > 0.1) {
            this.speed = Math.max(0, this.speed - CONFIG.decelLimit * 0.016);
            return;
        }
        
        // Turn around
        this.reverse = !this.reverse;
        this.turnaroundPending = false;
        this.speed = 0.1; // Small initial speed
    }
    
    getSpeed() {
        return msToKmh(this.speed);
    }
    
    getTopSpeed() {
        return msToKmh(this.topSpeed);
    }
}

// ============================================================================
// Simulation Engine
// ============================================================================

class SimulationEngine {
    constructor(network) {
        this.network = network;
        this.cars = [];
        this.running = false;
        this.paused = false;
        this.speed = 1.0;
        this.simTime = 0;
        this.nextCarId = 1;
        
        // Performance tracking
        this.frameCount = 0;
        this.lastFpsTime = 0;
        this.fps = 60;
        this.performanceMode = false;
    }
    
    start() {
        this.running = true;
        this.paused = false;
    }
    
    pause() {
        this.paused = !this.paused;
    }
    
    reset() {
        this.cars = [];
        this.simTime = 0;
        this.running = false;
        this.paused = false;
    }
    
    spawnCars(count, seed = null) {
        if (seed !== null) {
            // Simple seed for reproducibility (not full RNG seeding)
            Math.random = (() => {
                let value = seed;
                return () => {
                    value = (value * 9301 + 49297) % 233280;
                    return value / 233280;
                };
            })();
        }
        
        this.cars = [];
        for (let i = 0; i < count && i < CONFIG.maxCars; i++) {
            const spawnPoint = this.network.getRandomSpawnPoint();
            if (spawnPoint) {
                const car = new CarAgent(this.nextCarId++, spawnPoint, this.network);
                spawnPoint.segment.cars.add(car);
                this.cars.push(car);
            }
        }
    }
    
    addCar() {
        const spawnPoint = this.network.getRandomSpawnPoint();
        if (spawnPoint) {
            const car = new CarAgent(this.nextCarId++, spawnPoint, this.network);
            spawnPoint.segment.cars.add(car);
            this.cars.push(car);
            return car;
        }
        return null;
    }
    
    removeCar(carId) {
        const index = this.cars.findIndex(c => c.id === carId);
        if (index >= 0) {
            const car = this.cars[index];
            car.currentSegment.cars.delete(car);
            this.cars.splice(index, 1);
            return true;
        }
        return false;
    }
    
    update(dt) {
        if (!this.running || this.paused) return;
        
        const effectiveDt = dt * this.speed;
        this.simTime += effectiveDt;
        
        // Update all cars
        for (const car of this.cars) {
            car.update(effectiveDt, this.cars);
        }
        
        // Collision avoidance (corrective)
        this.resolveCollisions();
        
        // Remove cars that are stuck or off network
        this.cars = this.cars.filter(car => {
            if (!car.currentSegment || !this.network.segments.has(car.currentSegment.id)) {
                car.currentSegment?.cars.delete(car);
                return false;
            }
            return true;
        });
    }
    
    resolveCollisions() {
        // Simple collision resolution: maintain minimum distance
        for (let i = 0; i < this.cars.length; i++) {
            for (let j = i + 1; j < this.cars.length; j++) {
                const car1 = this.cars[i];
                const car2 = this.cars[j];
                
                if (car1.currentSegment !== car2.currentSegment) continue;
                
                const dist = Math.abs(car1.t - car2.t) * car1.currentSegment.length;
                const minDist = CONFIG.carLength * 1.1;
                
                if (dist < minDist) {
                    // Separate cars
                    const midpoint = (car1.t + car2.t) / 2;
                    const separation = (minDist - dist) / car1.currentSegment.length / 2;
                    
                    if (car1.t < car2.t) {
                        car1.t = Math.max(0, midpoint - separation);
                        car2.t = Math.min(1, midpoint + separation);
                    } else {
                        car2.t = Math.max(0, midpoint - separation);
                        car1.t = Math.min(1, midpoint + separation);
                    }
                    
                    // Slow down both
                    car1.speed = Math.min(car1.speed, car2.speed * 0.9);
                    car2.speed = Math.min(car2.speed, car1.speed * 0.9);
                }
            }
        }
    }
    
    getMetrics() {
        if (this.cars.length === 0) {
            return {
                count: 0,
                meanSpeed: 0,
                medianSpeed: 0,
                avgDelay: 0
            };
        }
        
        const speeds = this.cars.map(c => c.getSpeed()).sort((a, b) => a - b);
        const meanSpeed = speeds.reduce((a, b) => a + b, 0) / speeds.length;
        const medianSpeed = speeds[Math.floor(speeds.length / 2)];
        
        // Average delay (simplified: compare to top speed)
        const avgTopSpeed = this.cars.reduce((sum, c) => sum + c.getTopSpeed(), 0) / this.cars.length;
        const avgDelay = avgTopSpeed > 0 ? 
            ((avgTopSpeed - meanSpeed) / avgTopSpeed) * 100 : 0;
        
        return {
            count: this.cars.length,
            meanSpeed,
            medianSpeed,
            avgDelay
        };
    }
    
    updateFPS() {
        this.frameCount++;
        const now = performance.now();
        if (now - this.lastFpsTime >= 1000) {
            this.fps = this.frameCount;
            this.frameCount = 0;
            this.lastFpsTime = now;
            
            // Check performance mode
            if (this.fps < CONFIG.performanceModeThreshold && !this.performanceMode) {
                this.performanceMode = true;
                showToast("Performance mode enabled", "warning");
            } else if (this.fps >= CONFIG.performanceModeThreshold * 1.5 && this.performanceMode) {
                this.performanceMode = false;
            }
        }
    }
}

// ============================================================================
// Canvas Renderer
// ============================================================================

class CanvasRenderer {
    constructor(canvas) {
        this.canvas = canvas;
        this.ctx = canvas.getContext('2d');
        this.scale = 1.0;
        this.offsetX = 0;
        this.offsetY = 0;
        this.showGrid = false;
        this.showDirection = false;
    }
    
    resize() {
        const rect = this.canvas.getBoundingClientRect();
        this.canvas.width = rect.width * window.devicePixelRatio;
        this.canvas.height = rect.height * window.devicePixelRatio;
        this.ctx.scale(window.devicePixelRatio, window.devicePixelRatio);
    }
    
    worldToScreen(worldPos) {
        return {
            x: (worldPos.x - this.offsetX) * this.scale,
            y: (worldPos.y - this.offsetY) * this.scale
        };
    }
    
    screenToWorld(screenPos) {
        return {
            x: screenPos.x / this.scale + this.offsetX,
            y: screenPos.y / this.scale + this.offsetY
        };
    }
    
    zoom(factor, centerX, centerY) {
        const oldScale = this.scale;
        this.scale = Math.max(0.1, Math.min(10, this.scale * factor));
        
        const worldX = this.screenToWorld({ x: centerX, y: centerY }).x;
        const worldY = this.screenToWorld({ x: centerX, y: centerY }).y;
        
        this.offsetX = worldX - centerX / this.scale;
        this.offsetY = worldY - centerY / this.scale;
    }
    
    pan(dx, dy) {
        this.offsetX -= dx / this.scale;
        this.offsetY -= dy / this.scale;
    }
    
    clear() {
        this.ctx.clearRect(0, 0, this.canvas.width / window.devicePixelRatio, 
                          this.canvas.height / window.devicePixelRatio);
    }
    
    render(network, simulation, selectedItem = null) {
        this.clear();
        
        // Draw grid
        if (this.showGrid) {
            this.drawGrid();
        }
        
        // Draw roads
        for (const segment of network.segments.values()) {
            this.drawSegment(segment, selectedItem === segment);
        }
        
        // Draw junctions
        for (const junction of network.junctions.values()) {
            this.drawJunction(junction);
        }
        
        // Draw cars
        for (const car of simulation.cars) {
            this.drawCar(car, selectedItem === car, simulation.performanceMode);
        }
    }
    
    drawGrid() {
        const gridSize = CONFIG.gridSize / this.scale;
        const screenWidth = this.canvas.width / window.devicePixelRatio;
        const screenHeight = this.canvas.height / window.devicePixelRatio;
        
        const worldTopLeft = this.screenToWorld({ x: 0, y: 0 });
        const worldBottomRight = this.screenToWorld({ x: screenWidth, y: screenHeight });
        
        this.ctx.strokeStyle = '#333';
        this.ctx.lineWidth = 0.5;
        
        const startX = Math.floor(worldTopLeft.x / gridSize) * gridSize;
        const startY = Math.floor(worldTopLeft.y / gridSize) * gridSize;
        
        for (let x = startX; x <= worldBottomRight.x; x += gridSize) {
            const screen = this.worldToScreen({ x, y: worldTopLeft.y });
            const screenEnd = this.worldToScreen({ x, y: worldBottomRight.y });
            this.ctx.beginPath();
            this.ctx.moveTo(screen.x, screen.y);
            this.ctx.lineTo(screen.x, screenEnd.y);
            this.ctx.stroke();
        }
        
        for (let y = startY; y <= worldBottomRight.y; y += gridSize) {
            const screen = this.worldToScreen({ x: worldTopLeft.x, y });
            const screenEnd = this.worldToScreen({ x: worldBottomRight.x, y });
            this.ctx.beginPath();
            this.ctx.moveTo(screen.x, screen.y);
            this.ctx.lineTo(screenEnd.x, screen.y);
            this.ctx.stroke();
        }
    }
    
    drawSegment(segment, selected) {
        this.ctx.strokeStyle = selected ? '#00aaff' : '#888';
        this.ctx.lineWidth = (selected ? 4 : 3) / this.scale;
        this.ctx.lineCap = 'round';
        this.ctx.lineJoin = 'round';
        
        if (!segment.twoWay) {
            this.ctx.setLineDash([5 / this.scale, 5 / this.scale]);
        } else {
            this.ctx.setLineDash([]);
        }
        
        this.ctx.beginPath();
        const start = this.worldToScreen(segment.points[0]);
        this.ctx.moveTo(start.x, start.y);
        
        for (let i = 1; i < segment.points.length; i++) {
            const point = this.worldToScreen(segment.points[i]);
            this.ctx.lineTo(point.x, point.y);
        }
        
        this.ctx.stroke();
        this.ctx.setLineDash([]);
        
        // Draw direction arrow if needed
        if (this.showDirection || !segment.twoWay) {
            this.drawDirectionArrow(segment);
        }
        
        // Draw vertices if selected
        if (selected) {
            for (const point of segment.points) {
                const screen = this.worldToScreen(point);
                this.ctx.fillStyle = '#00aaff';
                this.ctx.beginPath();
                this.ctx.arc(screen.x, screen.y, 5 / this.scale, 0, Math.PI * 2);
                this.ctx.fill();
            }
        }
    }
    
    drawDirectionArrow(segment) {
        const mid = segment.getPointAt(0.5);
        const dir = segment.getDirectionAt(0.5);
        const screen = this.worldToScreen(mid);
        const arrowLength = 15 / this.scale;
        
        this.ctx.strokeStyle = '#ffaa00';
        this.ctx.lineWidth = 2 / this.scale;
        this.ctx.fillStyle = '#ffaa00';
        
        this.ctx.beginPath();
        this.ctx.moveTo(screen.x, screen.y);
        const endX = screen.x + dir.x * arrowLength;
        const endY = screen.y + dir.y * arrowLength;
        this.ctx.lineTo(endX, endY);
        
        // Arrowhead
        const angle = Math.atan2(dir.y, dir.x);
        const arrowSize = 5 / this.scale;
        this.ctx.lineTo(endX - arrowSize * Math.cos(angle - Math.PI / 6),
                        endY - arrowSize * Math.sin(angle - Math.PI / 6));
        this.ctx.moveTo(endX, endY);
        this.ctx.lineTo(endX - arrowSize * Math.cos(angle + Math.PI / 6),
                        endY - arrowSize * Math.sin(angle + Math.PI / 6));
        
        this.ctx.stroke();
        
        if (!segment.twoWay) {
            // Draw reverse arrow too
            this.ctx.beginPath();
            this.ctx.moveTo(screen.x, screen.y);
            const revEndX = screen.x - dir.x * arrowLength;
            const revEndY = screen.y - dir.y * arrowLength;
            this.ctx.lineTo(revEndX, revEndY);
            const revAngle = angle + Math.PI;
            this.ctx.lineTo(revEndX - arrowSize * Math.cos(revAngle - Math.PI / 6),
                           revEndY - arrowSize * Math.sin(revAngle - Math.PI / 6));
            this.ctx.moveTo(revEndX, revEndY);
            this.ctx.lineTo(revEndX - arrowSize * Math.cos(revAngle + Math.PI / 6),
                           revEndY - arrowSize * Math.sin(revAngle + Math.PI / 6));
            this.ctx.stroke();
        }
    }
    
    drawJunction(junction) {
        const screen = this.worldToScreen(junction.position);
        const radius = (junction.degree >= 3 ? CONFIG.junctionRadius * 1.5 : CONFIG.junctionRadius) / this.scale;
        
        this.ctx.fillStyle = junction.degree >= 3 ? '#ff6600' : '#ffaa00';
        this.ctx.strokeStyle = '#ffcc00';
        this.ctx.lineWidth = 2 / this.scale;
        
        this.ctx.beginPath();
        this.ctx.arc(screen.x, screen.y, radius, 0, Math.PI * 2);
        this.ctx.fill();
        this.ctx.stroke();
    }
    
    drawCar(car, selected, simpleMode) {
        const screen = this.worldToScreen(car.position);
        const angle = Math.atan2(car.direction.y, car.direction.x);
        
        this.ctx.save();
        this.ctx.translate(screen.x, screen.y);
        this.ctx.rotate(angle);
        
        if (simpleMode) {
            // Simple dot mode
            this.ctx.fillStyle = selected ? '#ffaa00' : '#00ccff';
            this.ctx.beginPath();
            this.ctx.arc(0, 0, 3 / this.scale, 0, Math.PI * 2);
            this.ctx.fill();
        } else {
            // Car shape
            const length = CONFIG.carLength / this.scale;
            const width = length * 0.4;
            
            this.ctx.fillStyle = selected ? '#ffaa00' : '#00ccff';
            this.ctx.strokeStyle = selected ? '#ffcc00' : '#0088aa';
            this.ctx.lineWidth = 1 / this.scale;
            
            this.ctx.beginPath();
            this.ctx.rect(-length / 2, -width / 2, length, width);
            this.ctx.fill();
            this.ctx.stroke();
        }
        
        this.ctx.restore();
    }
}

// ============================================================================
// Drawing Tools & Interaction
// ============================================================================

class DrawingTools {
    constructor(canvas, renderer, network, simulation) {
        this.canvas = canvas;
        this.renderer = renderer;
        this.network = network;
        this.simulation = simulation;
        
        this.tool = 'select'; // select, draw, edit, delete
        this.selectedItem = null;
        this.dragging = false;
        this.dragStart = null;
        this.currentDrawPoints = null;
        this.editingVertex = null;
        this.panning = false;
        this.panStart = null;
        
        this.history = [];
        this.historyIndex = -1;
        
        // Initialize history with empty state
        this.saveState();
        
        this.setupEventListeners();
    }
    
    setupEventListeners() {
        // Mouse events
        this.canvas.addEventListener('mousedown', (e) => this.onMouseDown(e));
        this.canvas.addEventListener('mousemove', (e) => this.onMouseMove(e));
        this.canvas.addEventListener('mouseup', (e) => this.onMouseUp(e));
        this.canvas.addEventListener('wheel', (e) => this.onWheel(e));
        this.canvas.addEventListener('contextmenu', (e) => e.preventDefault());
        
        // Keyboard
        document.addEventListener('keydown', (e) => this.onKeyDown(e));
        document.addEventListener('keyup', (e) => this.onKeyUp(e));
    }
    
    onMouseDown(e) {
        const rect = this.canvas.getBoundingClientRect();
        const screenPos = {
            x: (e.clientX - rect.left) * window.devicePixelRatio,
            y: (e.clientY - rect.top) * window.devicePixelRatio
        };
        const worldPos = this.renderer.screenToWorld(screenPos);
        
        if (e.button === 1 || (e.button === 0 && e.ctrlKey)) {
            // Middle mouse or Ctrl+click = pan
            this.panning = true;
            this.panStart = screenPos;
            return;
        }
        
        if (this.tool === 'select') {
            this.handleSelect(worldPos, e.shiftKey);
        } else if (this.tool === 'draw') {
            this.startDrawing(worldPos);
        } else if (this.tool === 'edit') {
            this.startEditing(worldPos);
        } else if (this.tool === 'delete') {
            this.handleDelete(worldPos);
        }
    }
    
    onMouseMove(e) {
        const rect = this.canvas.getBoundingClientRect();
        const screenPos = {
            x: (e.clientX - rect.left) * window.devicePixelRatio,
            y: (e.clientY - rect.top) * window.devicePixelRatio
        };
        const worldPos = this.renderer.screenToWorld(screenPos);
        
        if (this.panning && this.panStart) {
            this.renderer.pan(screenPos.x - this.panStart.x, screenPos.y - this.panStart.y);
            this.panStart = screenPos;
            return;
        }
        
        if (this.tool === 'draw' && this.currentDrawPoints && this.currentDrawPoints.length > 0) {
            // Add point if far enough from last point
            const lastPoint = this.currentDrawPoints[this.currentDrawPoints.length - 1];
            const dist = distance(worldPos, lastPoint);
            if (dist > 5 / this.renderer.scale) { // Minimum distance to add point
                this.currentDrawPoints.push(worldPos);
            }
        } else if (this.tool === 'edit' && this.editingVertex) {
            this.editingVertex.x = worldPos.x;
            this.editingVertex.y = worldPos.y;
            this.editingVertex.segment.updateLength();
            this.network.updateJunctions();
        }
    }
    
    onMouseUp(e) {
        if (this.panning) {
            this.panning = false;
            this.panStart = null;
            return;
        }
        
        if (this.tool === 'draw' && this.currentDrawPoints) {
            if (this.currentDrawPoints.length >= 2) {
                this.finishDrawing();
            } else {
                // Not enough points, cancel drawing
                this.currentDrawPoints = null;
            }
        } else if (this.tool === 'edit' && this.editingVertex) {
            this.saveState();
            this.editingVertex = null;
        }
    }
    
    onWheel(e) {
        e.preventDefault();
        const rect = this.canvas.getBoundingClientRect();
        const centerX = (e.clientX - rect.left) * window.devicePixelRatio;
        const centerY = (e.clientY - rect.top) * window.devicePixelRatio;
        const zoomFactor = e.deltaY > 0 ? 0.9 : 1.1;
        this.renderer.zoom(zoomFactor, centerX, centerY);
    }
    
    onKeyDown(e) {
        if (e.key === ' ') {
            this.panning = true;
            e.preventDefault();
        } else if (e.key === 'z' && e.ctrlKey) {
            e.preventDefault();
            this.undo();
        } else if ((e.key === 'y' || (e.key === 'z' && e.shiftKey && e.ctrlKey)) && e.ctrlKey) {
            e.preventDefault();
            this.redo();
        } else if (e.key === 'Delete' || e.key === 'Backspace') {
            if (this.selectedItem) {
                this.deleteSelected();
            }
        } else if (e.key === 's' && !e.ctrlKey) {
            this.setTool('select');
        } else if (e.key === 'd' && !e.ctrlKey) {
            this.setTool('draw');
        } else if (e.key === 'e' && !e.ctrlKey) {
            this.setTool('edit');
        }
    }
    
    onKeyUp(e) {
        if (e.key === ' ') {
            this.panning = false;
        }
    }
    
    handleSelect(worldPos, multiSelect) {
        // Try to select car first
        let selected = null;
        let minDist = 20 / this.renderer.scale;
        
        for (const car of this.simulation.cars) {
            const dist = distance(worldPos, car.position);
            if (dist < minDist) {
                minDist = dist;
                selected = car;
            }
        }
        
        // Then try segment
        if (!selected) {
            const nearest = this.network.findNearestSegment(worldPos, 20 / this.renderer.scale);
            if (nearest) {
                selected = nearest.segment;
            }
        }
        
        if (!multiSelect) {
            this.selectedItem = selected;
        } else {
            // Multi-select not implemented in MVP
            this.selectedItem = selected;
        }
    }
    
    startDrawing(worldPos) {
        this.currentDrawPoints = [worldPos];
    }
    
    finishDrawing() {
        if (this.currentDrawPoints.length < 2) return;
        
        // Check if segment has cars
        let hasCars = false;
        for (const segment of this.network.segments.values()) {
            if (segment.cars.size > 0) {
                hasCars = true;
                break;
            }
        }
        
        const segment = this.network.addSegment(this.currentDrawPoints);
        if (segment) {
            this.saveState();
            showToast("Road segment created", "success");
        } else {
            showToast("Invalid road segment (too short)", "error");
        }
        
        this.currentDrawPoints = null;
    }
    
    startEditing(worldPos) {
        if (!this.selectedItem || !(this.selectedItem instanceof RoadSegment)) return;
        
        // Find nearest vertex
        const segment = this.selectedItem;
        let minDist = Infinity;
        let nearestVertex = null;
        
        for (const vertex of segment.points) {
            const dist = distance(worldPos, vertex);
            if (dist < minDist && dist < 15 / this.renderer.scale) {
                minDist = dist;
                nearestVertex = vertex;
            }
        }
        
        if (nearestVertex) {
            // Check if segment has cars
            if (segment.cars.size > 0) {
                showToast("Cannot edit road with cars on it. Reset simulation first.", "warning");
                return;
            }
            
            this.editingVertex = {
                segment: segment,
                x: nearestVertex.x,
                y: nearestVertex.y
            };
            Object.assign(nearestVertex, this.editingVertex);
        }
    }
    
    handleDelete(worldPos) {
        const nearest = this.network.findNearestSegment(worldPos, 20 / this.renderer.scale);
        if (nearest && nearest.dist < 20 / this.renderer.scale) {
            const segment = nearest.segment;
            
            if (segment.cars.size > 0) {
                // Despawn cars on this segment
                const carsToRemove = Array.from(segment.cars);
                for (const car of carsToRemove) {
                    this.simulation.removeCar(car.id);
                }
                showToast(`Removed ${carsToRemove.length} cars from segment`, "info");
            }
            
            this.network.removeSegment(segment.id);
            this.saveState();
            this.selectedItem = null;
            showToast("Road segment deleted", "success");
        }
    }
    
    deleteSelected() {
        if (!this.selectedItem) return;
        
        if (this.selectedItem instanceof CarAgent) {
            this.simulation.removeCar(this.selectedItem.id);
            this.selectedItem = null;
            showToast("Car removed", "success");
        } else if (this.selectedItem instanceof RoadSegment) {
            const segment = this.selectedItem;
            if (segment.cars.size > 0) {
                const carsToRemove = Array.from(segment.cars);
                for (const car of carsToRemove) {
                    this.simulation.removeCar(car.id);
                }
            }
            this.network.removeSegment(segment.id);
            this.saveState();
            this.selectedItem = null;
            showToast("Road segment deleted", "success");
        }
    }
    
    setTool(tool) {
        this.tool = tool;
        document.querySelectorAll('.tool-btn').forEach(btn => btn.classList.remove('active'));
        
        const toolMap = {
            'select': 'tool-select',
            'draw': 'tool-draw',
            'edit': 'tool-edit',
            'delete': 'tool-delete'
        };
        
        const btn = document.getElementById(toolMap[tool]);
        if (btn) btn.classList.add('active');
        
        this.selectedItem = null;
        this.currentDrawPoints = null;
        this.editingVertex = null;
    }
    
    saveState() {
        // Save current network state for undo/redo
        const state = this.network.toJSON();
        
        // Remove old future states if we're in the middle of history
        if (this.historyIndex < this.history.length - 1) {
            this.history = this.history.slice(0, this.historyIndex + 1);
        }
        
        this.history.push(JSON.stringify(state));
        if (this.history.length > CONFIG.maxHistory) {
            this.history.shift();
        }
        this.historyIndex = this.history.length - 1;
    }
    
    undo() {
        if (this.historyIndex > 0) {
            this.historyIndex--;
            this.restoreState(this.history[this.historyIndex]);
        }
    }
    
    redo() {
        if (this.historyIndex < this.history.length - 1) {
            this.historyIndex++;
            this.restoreState(this.history[this.historyIndex]);
        }
    }
    
    restoreState(stateJson) {
        const data = JSON.parse(stateJson);
        const newNetwork = RoadNetwork.fromJSON(data);
        
        // Preserve car references
        for (const car of this.simulation.cars) {
            const newSegment = newNetwork.segments.get(car.currentSegment?.id);
            if (newSegment) {
                if (car.currentSegment) {
                    car.currentSegment.cars.delete(car);
                }
                car.currentSegment = newSegment;
                newSegment.cars.add(car);
            } else {
                // Car is on a removed segment - remove the car
                if (car.currentSegment) {
                    car.currentSegment.cars.delete(car);
                }
                const carIndex = this.simulation.cars.indexOf(car);
                if (carIndex >= 0) {
                    this.simulation.cars.splice(carIndex, 1);
                }
            }
        }
        
        // Update all references
        this.network = newNetwork;
        this.renderer.network = newNetwork;
        this.simulation.network = newNetwork;
        
        // Update selected item if it was a segment
        if (this.selectedItem instanceof RoadSegment) {
            const newSegment = newNetwork.segments.get(this.selectedItem.id);
            this.selectedItem = newSegment || null;
        }
    }
}

// ============================================================================
// Scenario Manager
// ============================================================================

class ScenarioManager {
    constructor(network, simulation) {
        this.network = network;
        this.simulation = simulation;
    }
    
    save() {
        const scenario = {
            network: this.network.toJSON(),
            settings: {
                desiredHeadway: CONFIG.desiredHeadway,
                globalSpeedLimit: parseFloat(document.getElementById('global-speed-limit').value),
                snapRadius: parseFloat(document.getElementById('snap-radius').value)
            },
            version: "1.0",
            timestamp: new Date().toISOString()
        };
        
        const json = JSON.stringify(scenario, null, 2);
        const blob = new Blob([json], { type: 'application/json' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `traffic-scenario-${Date.now()}.json`;
        a.click();
        URL.revokeObjectURL(url);
        
        showToast("Scenario saved", "success");
    }
    
    async load(file) {
        try {
            const text = await file.text();
            const scenario = JSON.parse(text);
            
            // Restore network
            const newNetwork = RoadNetwork.fromJSON(scenario.network);
            this.network.segments.clear();
            this.network.junctions.clear();
            
            for (const [id, segment] of newNetwork.segments) {
                this.network.segments.set(id, segment);
            }
            for (const [id, junction] of newNetwork.junctions) {
                this.network.junctions.set(id, junction);
            }
            this.network.nextSegmentId = newNetwork.nextSegmentId;
            this.network.nextJunctionId = newNetwork.nextJunctionId;
            
            // Update references
            this.simulation.network = this.network;
            
            // Restore settings if available
            if (scenario.settings) {
                if (scenario.settings.desiredHeadway) {
                    CONFIG.desiredHeadway = scenario.settings.desiredHeadway;
                    document.getElementById('desired-headway').value = scenario.settings.desiredHeadway;
                }
                if (scenario.settings.globalSpeedLimit) {
                    document.getElementById('global-speed-limit').value = scenario.settings.globalSpeedLimit;
                }
                if (scenario.settings.snapRadius) {
                    CONFIG.snapRadius = scenario.settings.snapRadius;
                    document.getElementById('snap-radius').value = scenario.settings.snapRadius;
                }
            }
            
            // Reset simulation
            this.simulation.reset();
            
            showToast("Scenario loaded", "success");
        } catch (error) {
            showToast("Failed to load scenario: " + error.message, "error");
        }
    }
}

// ============================================================================
// UI Helpers
// ============================================================================

function showToast(message, type = "info") {
    const container = document.getElementById('toast-container');
    const toast = document.createElement('div');
    toast.className = `toast ${type}`;
    toast.textContent = message;
    container.appendChild(toast);
    
    setTimeout(() => {
        toast.style.animation = 'slideIn 0.3s ease reverse';
        setTimeout(() => toast.remove(), 300);
    }, 3000);
}

// ============================================================================
// Main Application
// ============================================================================

class TrafficSimulatorApp {
    constructor() {
        this.canvas = document.getElementById('canvas');
        this.renderer = new CanvasRenderer(this.canvas);
        this.network = new RoadNetwork();
        this.simulation = new SimulationEngine(this.network);
        this.tools = new DrawingTools(this.canvas, this.renderer, this.network, this.simulation);
        this.scenarioManager = new ScenarioManager(this.network, this.simulation);
        
        this.renderer.network = this.network;
        this.setupUI();
        this.setupResize();
        this.startRenderLoop();
        
        // Show tutorial on first visit
        if (!localStorage.getItem('tutorial-seen')) {
            document.getElementById('tutorial-modal').style.display = 'flex';
        }
    }
    
    setupUI() {
        // Tools
        document.getElementById('tool-select').addEventListener('click', () => this.tools.setTool('select'));
        document.getElementById('tool-draw').addEventListener('click', () => this.tools.setTool('draw'));
        document.getElementById('tool-edit').addEventListener('click', () => this.tools.setTool('edit'));
        document.getElementById('tool-delete').addEventListener('click', () => this.tools.setTool('delete'));
        
        // Undo/Redo
        document.getElementById('btn-undo').addEventListener('click', () => this.tools.undo());
        document.getElementById('btn-redo').addEventListener('click', () => this.tools.redo());
        
        // Grid toggle
        document.getElementById('toggle-grid').addEventListener('change', (e) => {
            this.renderer.showGrid = e.target.checked;
        });
        
        // Direction toggle
        document.getElementById('toggle-direction').addEventListener('change', (e) => {
            this.renderer.showDirection = e.target.checked;
        });
        
        // Save/Load
        document.getElementById('btn-save').addEventListener('click', () => this.scenarioManager.save());
        document.getElementById('btn-load').addEventListener('click', () => {
            document.getElementById('file-input').click();
        });
        document.getElementById('file-input').addEventListener('change', async (e) => {
            const file = e.target.files[0];
            if (file) {
                await this.scenarioManager.load(file);
            }
        });
        
        // Simulation controls
        document.getElementById('btn-start').addEventListener('click', () => {
            this.simulation.start();
            document.getElementById('btn-start').style.display = 'none';
            document.getElementById('btn-pause').style.display = 'inline-block';
            document.getElementById('status-text').textContent = 'Simulation running';
        });
        
        document.getElementById('btn-pause').addEventListener('click', () => {
            this.simulation.pause();
            const isPaused = this.simulation.paused;
            document.getElementById('btn-start').style.display = isPaused ? 'inline-block' : 'none';
            document.getElementById('btn-pause').textContent = isPaused ? 'Resume' : 'Pause';
            document.getElementById('status-text').textContent = isPaused ? 'Simulation paused' : 'Simulation running';
        });
        
        document.getElementById('btn-reset').addEventListener('click', () => {
            this.simulation.reset();
            document.getElementById('btn-start').style.display = 'inline-block';
            document.getElementById('btn-pause').style.display = 'none';
            document.getElementById('status-text').textContent = 'Simulation reset';
        });
        
        document.getElementById('speed-select').addEventListener('change', (e) => {
            this.simulation.speed = parseFloat(e.target.value);
        });
        
        // Car management
        document.getElementById('btn-place-cars').addEventListener('click', () => {
            const count = parseInt(document.getElementById('car-count-input').value);
            if (this.network.segments.size === 0) {
                showToast("Draw roads first!", "error");
                return;
            }
            this.simulation.spawnCars(count);
            showToast(`Placed ${count} cars`, "success");
        });
        
        document.getElementById('btn-add-car').addEventListener('click', () => {
            if (this.network.segments.size === 0) {
                showToast("Draw roads first!", "error");
                return;
            }
            const car = this.simulation.addCar();
            if (car) {
                showToast("Car added", "success");
            }
        });
        
        // Settings
        document.getElementById('snap-radius').addEventListener('change', (e) => {
            CONFIG.snapRadius = parseFloat(e.target.value);
            this.network.updateJunctions();
        });
        
        document.getElementById('desired-headway').addEventListener('change', (e) => {
            CONFIG.desiredHeadway = parseFloat(e.target.value);
            for (const car of this.simulation.cars) {
                car.headway = CONFIG.desiredHeadway;
            }
        });
        
        // Tutorial
        document.getElementById('btn-close-tutorial').addEventListener('click', () => {
            document.getElementById('tutorial-modal').style.display = 'none';
            if (document.getElementById('dont-show-again').checked) {
                localStorage.setItem('tutorial-seen', 'true');
            }
        });
    }
    
    setupResize() {
        window.addEventListener('resize', () => {
            this.renderer.resize();
        });
        this.renderer.resize();
    }
    
    startRenderLoop() {
        let lastTime = performance.now();
        
        const loop = (currentTime) => {
            const dt = Math.min((currentTime - lastTime) / 1000, CONFIG.maxTimestep);
            lastTime = currentTime;
            
            // Update simulation
            this.simulation.update(dt);
            this.simulation.updateFPS();
            
            // Update UI
            this.updateUI();
            
            // Render
            this.renderer.render(this.network, this.simulation, this.tools.selectedItem);
            
            // Draw current drawing if in progress
            if (this.tools.currentDrawPoints && this.tools.currentDrawPoints.length > 1) {
                this.renderer.ctx.strokeStyle = '#888';
                this.renderer.ctx.lineWidth = 3 / this.renderer.scale;
                this.renderer.ctx.lineCap = 'round';
                this.renderer.ctx.beginPath();
                const start = this.renderer.worldToScreen(this.tools.currentDrawPoints[0]);
                this.renderer.ctx.moveTo(start.x, start.y);
                for (let i = 1; i < this.tools.currentDrawPoints.length; i++) {
                    const point = this.renderer.worldToScreen(this.tools.currentDrawPoints[i]);
                    this.renderer.ctx.lineTo(point.x, point.y);
                }
                this.renderer.ctx.stroke();
            }
            
            requestAnimationFrame(loop);
        };
        
        requestAnimationFrame(loop);
    }
    
    updateUI() {
        // HUD
        document.getElementById('sim-time').textContent = this.simulation.simTime.toFixed(1);
        document.getElementById('fps').textContent = Math.round(this.simulation.fps);
        document.getElementById('car-count').textContent = this.simulation.cars.length;
        
        const metrics = this.simulation.getMetrics();
        document.getElementById('mean-speed').textContent = metrics.meanSpeed.toFixed(1);
        
        // Sidebar metrics
        document.getElementById('metric-cars').textContent = metrics.count;
        document.getElementById('metric-mean-speed').textContent = metrics.meanSpeed.toFixed(1);
        document.getElementById('metric-median-speed').textContent = metrics.medianSpeed.toFixed(1);
        document.getElementById('metric-delay').textContent = metrics.avgDelay.toFixed(1);
        
        // Car info
        if (this.tools.selectedItem instanceof CarAgent) {
            const car = this.tools.selectedItem;
            const panel = document.getElementById('car-info-panel');
            const content = document.getElementById('car-info-content');
            panel.style.display = 'block';
            content.innerHTML = `
                <div>Speed: ${car.getSpeed().toFixed(1)} km/h</div>
                <div>Target: ${msToKmh(car.targetSpeed).toFixed(1)} km/h</div>
                <div>Top Speed: ${car.getTopSpeed().toFixed(1)} km/h</div>
                <div>Reaction Time: ${car.reactionTime.toFixed(2)}s</div>
                <div>Headway: ${car.headway.toFixed(1)}s</div>
            `;
        } else {
            document.getElementById('car-info-panel').style.display = 'none';
        }
    }
}

// Initialize app when DOM is ready
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', () => {
        new TrafficSimulatorApp();
    });
} else {
    new TrafficSimulatorApp();
}