Body_Kind :: enum {
    CIRCLE;
    POLYGON;
}

Circle_Data :: struct {
    radius: float;
}

Polygon_Data :: struct {
    MAX_VERTICES :: 8;
    vertices_count: int;
    vertices:       [MAX_VERTICES] Vector2;
}

Body :: struct {
    kind: Body_Kind;
    union {
        circle:  Circle_Data;
        polygon: Polygon_Data;
    }
    position:           Vector2;
    rotation:           float;
    linear_velocity:    Vector2;
    angular_velocity:   float;
    friction:           float;
    mass:               float;
    inertia:            float;
    inv_mass:           float;
    inv_inertia:        float;
    force_accumulator:  Vector2;
    torque_accumulator: float;
    userdata:           *void;
}

InitCircle :: (using body: *Body, x: float, y: float, r: float, m: float, f := 0.2) {
    kind          = .CIRCLE;
    position      = .{ x, y };
	circle.radius = r;
	mass          = m;
	inertia       = ComputeCircleInertia(m, r);
	inv_mass      = ifx mass > 0.0 then 1.0 / mass    else 0.0;
	inv_inertia   = ifx mass > 0.0 then 1.0 / inertia else 0.0;
	friction      = f;
}

InitPolygon :: (using body: *Body, x: float, y: float, local_vertices: [] Vector2, m: float, f := 0.2) {
    centroid := ComputePolygonCentroid(local_vertices);

    kind        = .POLYGON;
    position    = .{ x, y } + centroid;
    for local_vertices  polygon.vertices[it_index] = it - centroid;
    polygon.vertices_count = local_vertices.count;
	mass        = m;
	inertia     = ComputePolygonInertia(local_vertices, m);
	inv_mass    = ifx mass > 0.0 then 1.0 / mass    else 0.0;
	inv_inertia = ifx mass > 0.0 then 1.0 / inertia else 0.0;
	friction    = f;
}

InitPolygon :: (using body: *Body, world_vertices: [] Vector2, m: float, f := 0.2) {
    InitPolygon(body, 0, 0, world_vertices, m, f);
}

InitBox :: (using body: *Body, x: float, y: float, w: float, h: float, m: float, f := 0.2) {
    InitPolygon(body, x, y, .[ .{ w/2, h/2 }, .{ -w/2, h/2 }, .{ -w/2, -h/2 }, .{ w/2, -h/2 } ], m, f);
}

IsStatic :: (using body: Body) -> bool {
	return inv_mass == 0.0;
}

AddForce :: (using body: *Body, force: Vector2) {
	if IsStatic(body) return;
	force_accumulator += force;
}

AddTorque :: (using body: *Body, torque: float) {
	if IsStatic(body) return;
	torque_accumulator += torque;
}

AddForceAt :: (using body: *Body, force: Vector2, world_point: Vector2) {
    force_accumulator  += force;
    torque_accumulator += Cross(world_point - position, force);
}

ApplyLinearImpulse :: (using body: *Body, impulse: Vector2) {
	if IsStatic(body) return;
    linear_velocity += inv_mass * impulse;
}

ApplyAngularImpulse :: (using body: *Body, impulse: float) {
	if IsStatic(body) return;
    angular_velocity += inv_inertia * impulse;
}

ApplyImpulseAt :: (using body: *Body, impulse: Vector2, world_point: Vector2) {
	if IsStatic(body) return;
    linear_velocity  += inv_mass    * impulse;
    angular_velocity += inv_inertia * Cross(world_point, impulse);
}

LocalToWorld :: (using body: Body, local_point: Vector2) -> Vector2 {
    return rotate(local_point, rotation) + position;
}

WorldToLocal :: (using body: Body, world_point: Vector2) -> Vector2 {
    return rotate(world_point, -rotation) -position;
}

PolygonWorldVertices :: (using body: Body) -> [Polygon_Data.MAX_VERTICES] Vector2 {
    assert(body.kind == .POLYGON);

    world_points: [Polygon_Data.MAX_VERTICES] Vector2;
    for 0..polygon.vertices_count - 1 {
        world_points[it] = rotate(polygon.vertices[it], rotation) + position;
    }
	return world_points;
}
